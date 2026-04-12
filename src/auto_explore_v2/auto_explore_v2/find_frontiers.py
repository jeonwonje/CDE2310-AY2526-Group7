import json
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String

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
        self.map_formatted_data = {}
        self.occupancy_grid = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = msg.info.origin
        for i in range(self.map_height):
            for j in range(self.map_width):
                index = i * self.map_width + j
                self.map_formatted_data[(j, i)] = self.occupancy_grid[index]
        self.find_frontiers()
        self.bfs_distance_transform()

    def find_frontiers(self):
        if self.navigation_in_progress:
            self.get_logger().debug(
                'Navigation in progress - skipping frontier search'
            )
            return

        if not hasattr(self, 'map_formatted_data'):
            self.get_logger().warning(
                'Map data not yet available; skipping frontier search.'
            )
            return

        if not self.map_formatted_data:
            self.raw_frontiers = []
            self.frontier_clusters = []
            self.frontiers = []
            self.get_logger().debug('Map data is empty; no frontiers to find.')
            return

        self.raw_frontiers = []
        for (x, y), value in self.map_formatted_data.items():
            if 0 <= value <= 50:
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if (dx, dy) == (0, 0):
                            continue
                        neighbor = (x + dx, y + dy)
                        if (
                            neighbor in self.map_formatted_data
                            and self.map_formatted_data[neighbor] == -1
                        ):
                            self.raw_frontiers.append((x, y))
                            break
                    else:
                        continue
                    break

        self.get_logger().debug(
            f'Found {len(self.raw_frontiers)} raw frontier cells'
        )

    def cluster_frontiers(self, frontiers):
        if not frontiers:
            return []

        cell_set = set(frontiers)
        visited = set()
        clusters = []

        for seed in frontiers:
            if seed in visited:
                continue

            cluster = []
            queue = deque([seed])
            while queue:
                cell = queue.pop()
                if cell in visited:
                    continue

                visited.add(cell)
                cluster.append(cell)
                cell_x, cell_y = cell
                for delta_x, delta_y in (
                    (-1, 0),
                    (1, 0),
                    (0, -1),
                    (0, 1),
                    (-1, -1),
                    (-1, 1),
                    (1, -1),
                    (1, 1),
                ):
                    neighbor = (cell_x + delta_x, cell_y + delta_y)
                    if neighbor in cell_set and neighbor not in visited:
                        queue.append(neighbor)

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
                self.distance_transform.get(cell, float('-inf')),
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

        self.distance_transform = {}
        queue = deque()
        for (x, y), value in self.map_formatted_data.items():
            if value >= 50:
                self.distance_transform[(x, y)] = 0
                queue.append((x, y))
            else:
                self.distance_transform[(x, y)] = float('inf')

        while queue:
            x, y = queue.popleft()
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (x + dx, y + dy)
                next_distance = self.distance_transform[(x, y)] + 1
                if (
                    neighbor in self.distance_transform
                    and self.distance_transform[neighbor] > next_distance
                ):
                    self.distance_transform[neighbor] = next_distance
                    queue.append(neighbor)

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

    def publish_frontiers(self):
        frontiers_serializable = [list(frontier) for frontier in self.frontiers]
        self.frontiers_pub.publish(
            String(data=json.dumps(frontiers_serializable))
        )

    def publish_bfs(self):
        distance_serializable = {
            str(key): value for key, value in self.distance_transform.items()
        }
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
