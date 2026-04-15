import json
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String

try:
    from scipy.ndimage import distance_transform_cdt

    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

FRONTIER_MIN_SIZE = 3


class AutoExploreNode(Node):
    def __init__(self):
        super().__init__('auto_explore')
        self.get_logger().debug('auto_explore node has started')
        self.frontiers_pub = self.create_publisher(String, 'frontiers', 10)
        self.bfs_pub = self.create_publisher(
            String,
            'bfs_distance_transform',
            10,
        )
        self.frontiers = []
        self.raw_frontiers = []
        self.frontier_clusters = []
        self.navigation_in_progress = False
        self.map_data_subscription()
        self.nav_status_subscription()

    def map_data_subscription(self):
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10,
        )

    def nav_status_subscription(self):
        self.nav_status_subscriber = self.create_subscription(
            String,
            '/nav_status',
            self.nav_status_callback,
            10,
        )

    def nav_status_callback(self, msg):
        self.navigation_in_progress = msg.data == 'navigating'

    def map_callback(self, msg):
        self.get_logger().debug('Received map data')
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = msg.info.origin

        # Build a 2D numpy array (height x width) directly from the flat
        # occupancy grid.  Values: -1 = unknown, 0-100 = occupancy.
        self.grid = np.array(
            msg.data, dtype=np.int16,
        ).reshape(self.map_height, self.map_width)

        self.find_frontiers()
        self.bfs_distance_transform()

    def find_frontiers(self):
        if self.navigation_in_progress:
            self.get_logger().debug(
                'Navigation in progress - skipping frontier search'
            )
            return

        if not hasattr(self, 'grid'):
            self.get_logger().warning(
                'Map data not yet available; skipping frontier search.'
            )
            return

        if self.grid.size == 0:
            self.raw_frontiers = []
            self.frontier_clusters = []
            self.frontiers = []
            self.get_logger().debug('Map data is empty; no frontiers to find.')
            return

        # Vectorized frontier detection:
        # A frontier cell is free (0 <= value <= 50) AND has at least one
        # unknown (-1) neighbor among its 8 neighbors.

        grid = self.grid
        h, w = grid.shape

        # Boolean mask of free cells
        free = (grid >= 0) & (grid <= 50)

        # Boolean mask of unknown cells, padded so neighbor checks never
        # go out of bounds.  Padding with False (not unknown).
        unknown_padded = np.zeros((h + 2, w + 2), dtype=np.bool_)
        unknown_padded[1:-1, 1:-1] = (grid == -1)

        # For each of the 8 neighbors, check if at least one is unknown.
        # We shift the padded array and OR the results together.
        has_unknown_neighbor = np.zeros((h, w), dtype=np.bool_)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                # Slice out the region corresponding to the shifted neighbor.
                has_unknown_neighbor |= unknown_padded[
                    1 + dy: h + 1 + dy,
                    1 + dx: w + 1 + dx,
                ]

        frontier_mask = free & has_unknown_neighbor

        # Extract frontier coordinates as (x, y) tuples — note grid is
        # indexed [row, col] = [y, x].
        ys, xs = np.nonzero(frontier_mask)
        self.raw_frontiers = list(zip(xs.tolist(), ys.tolist()))

        # Cache the frontier mask for potential reuse
        self._frontier_mask = frontier_mask

        self.get_logger().debug(
            f'Found {len(self.raw_frontiers)} raw frontier cells'
        )

    def cluster_frontiers(self, frontiers):
        """BFS clustering of frontier cells using numpy label array."""
        if not frontiers:
            return []

        h, w = self.grid.shape

        # Build a boolean grid of frontier cells for O(1) neighbor lookups.
        frontier_grid = np.zeros((h, w), dtype=np.bool_)
        for x, y in frontiers:
            frontier_grid[y, x] = True

        # Label array: 0 = unvisited, >0 = cluster id
        labels = np.zeros((h, w), dtype=np.int32)
        cluster_id = 0
        clusters = []

        # 8-connected BFS using the numpy grid for O(1) lookups
        for x, y in frontiers:
            if labels[y, x] != 0:
                continue

            cluster_id += 1
            cluster = []
            queue = deque([(x, y)])
            labels[y, x] = cluster_id

            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for dx, dy in (
                    (-1, 0), (1, 0), (0, -1), (0, 1),
                    (-1, -1), (-1, 1), (1, -1), (1, 1),
                ):
                    nx, ny = cx + dx, cy + dy
                    if (
                        0 <= nx < w
                        and 0 <= ny < h
                        and frontier_grid[ny, nx]
                        and labels[ny, nx] == 0
                    ):
                        labels[ny, nx] = cluster_id
                        queue.append((nx, ny))

            if len(cluster) < FRONTIER_MIN_SIZE:
                continue

            representative = self.select_cluster_representative(cluster)
            clusters.append(
                (representative[0], representative[1], len(cluster))
            )

        return clusters

    def select_cluster_representative(self, cluster):
        centroid_x = sum(cell_x for cell_x, _ in cluster) / len(cluster)
        centroid_y = sum(cell_y for _, cell_y in cluster) / len(cluster)
        return max(
            cluster,
            key=lambda cell: (
                self._dt_array[cell[1], cell[0]]
                if hasattr(self, '_dt_array')
                else float('-inf'),
                -(
                    (cell[0] - centroid_x) ** 2
                    + (cell[1] - centroid_y) ** 2
                ),
                -cell[1],
                -cell[0],
            ),
        )

    def bfs_distance_transform(self):
        if self.navigation_in_progress:
            self.get_logger().debug(
                'Navigation in progress - skipping BFS distance transform'
            )
            return

        grid = self.grid
        h, w = grid.shape

        has_obstacles = np.any(grid >= 50)

        if _HAS_SCIPY and has_obstacles:
            # Obstacles/occupied cells (>= 50) get distance 0.
            # Free/unknown cells get their cityblock distance to the
            # nearest obstacle.
            # distance_transform_cdt expects: 0 = feature (obstacle),
            #                                 nonzero = region to fill.
            obstacle_mask = (grid < 50).astype(np.int32)
            dt = distance_transform_cdt(obstacle_mask, metric='cityblock')
            self._dt_array = dt.astype(np.float64)
        elif not has_obstacles:
            # No obstacles at all — every cell is infinitely far from
            # the nearest obstacle, matching original BFS behavior.
            self._dt_array = np.full((h, w), np.inf)
        else:
            # Fallback: manual BFS distance transform using numpy arrays
            self._dt_array = self._bfs_distance_numpy(grid)

        self.distance_transform = self._dt_array

        self.frontier_clusters = self.cluster_frontiers(self.raw_frontiers)
        self.frontiers = [
            (frontier_x, frontier_y)
            for frontier_x, frontier_y, _ in self.frontier_clusters
        ]
        self.get_logger().debug(
            f'Clustered {len(self.raw_frontiers)} raw frontier cells into '
            f'{len(self.frontier_clusters)} clusters; publishing '
            f'{len(self.frontiers)} clustered frontiers'
        )
        self.publish_frontiers()
        self.publish_bfs()

    def _bfs_distance_numpy(self, grid):
        """Manual BFS distance transform using numpy arrays.

        Matches the original dict-based BFS: obstacles (>= 50) start at
        distance 0, propagate outward to free/unknown cells via 4-connected
        neighbors.
        """
        h, w = grid.shape
        INF = float('inf')
        dt = np.full((h, w), INF, dtype=np.float64)

        queue = deque()
        obstacle_mask = grid >= 50
        ys, xs = np.nonzero(obstacle_mask)
        dt[obstacle_mask] = 0.0
        for x, y in zip(xs.tolist(), ys.tolist()):
            queue.append((x, y))

        while queue:
            x, y = queue.popleft()
            next_d = dt[y, x] + 1.0
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and dt[ny, nx] > next_d:
                    dt[ny, nx] = next_d
                    queue.append((nx, ny))

        return dt

    def publish_frontiers(self):
        frontiers_serializable = [list(frontier) for frontier in self.frontiers]
        self.frontiers_pub.publish(
            String(data=json.dumps(frontiers_serializable))
        )

    def publish_bfs(self):
        """Serialize the distance transform to the same JSON format as the
        original: ``{"(x, y)": distance, ...}`` where keys are
        ``str(tuple)`` and values are numeric (or Infinity).
        """
        dt = self._dt_array
        h, w = dt.shape
        flat = dt.ravel()

        # Build the dict matching original key format: "(x, y)"
        distance_serializable = {}
        idx = 0
        for y in range(h):
            for x in range(w):
                distance_serializable[f'({x}, {y})'] = flat[idx]
                idx += 1

        self.bfs_pub.publish(String(data=json.dumps(distance_serializable)))
        self.get_logger().debug(
            'Published frontiers and BFS distance transform data'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutoExploreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutdown requested (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
