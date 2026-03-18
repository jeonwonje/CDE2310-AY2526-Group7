#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math
import time
from collections import deque

# ── Constants ──────────────────────────────────────────────
FREE     = 0
UNKNOWN  = 1
OCCUPIED = 2

RECOMPUTE_INTERVAL = 2.0   # seconds between frontier recomputes
MIN_FRONTIER_SIZE  = 5     # discard frontiers smaller than this
BLACKLIST_RADIUS   = 0.5   # metres


class FrontierDetector(Node):

    def __init__(self):
        super().__init__('frontier_detector')

        # ── QoS: match SLAM Toolbox transient local ────
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )

        # ── Subscribers ───────────────────────────────
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, map_qos)

        self.blacklist_sub = self.create_subscription(
            Point, '/blacklist_frontier', self._blacklist_cb, 10)

        # ── Publishers ────────────────────────────────
        self.frontier_pub = self.create_publisher(
            MarkerArray, '/frontiers', 10)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/best_frontier', 10)

        # ── TF ────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── State ─────────────────────────────────────
        self.last_compute = 0.0
        self.blacklist    = []   # list of (x, y) world coords

        self.get_logger().info('Frontier detector ready')

    # ──────────────────────────────────────────────────────
    # Map callback — throttled
    # ──────────────────────────────────────────────────────
    def map_callback(self, msg):
        now = time.time()
        if now - self.last_compute < RECOMPUTE_INTERVAL:
            return
        self.last_compute = now
        self.compute_frontiers(msg)

    # ──────────────────────────────────────────────────────
    # Blacklist callback — receives failed frontiers
    # from exploration_manager via /blacklist_frontier topic
    # ──────────────────────────────────────────────────────
    def _blacklist_cb(self, msg):
        self.add_to_blacklist(msg.x, msg.y)

    # ──────────────────────────────────────────────────────
    # Get robot position in map grid coords
    # ──────────────────────────────────────────────────────
    def get_robot_map_coords(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('TF not available yet')
            return None, None, None, None

        cur_pos    = trans.transform.translation
        map_res    = msg.info.resolution
        map_origin = msg.info.origin.position

        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round((cur_pos.y - map_origin.y) / map_res)

        return grid_x, grid_y, cur_pos.x, cur_pos.y

    # ──────────────────────────────────────────────────────
    # Wavefront Frontier Detection — Yamauchi (1997)
    # Pseudocode lines annotated in comments
    # ──────────────────────────────────────────────────────
    def compute_frontiers(self, msg):
        width  = msg.info.width
        height = msg.info.height
        res    = msg.info.resolution
        origin = msg.info.origin.position

        robot_info = self.get_robot_map_coords(msg)
        if robot_info[0] is None:
            return
        grid_x, grid_y, robot_wx, robot_wy = robot_info

        # ── Convert to 2D grid ────────────────────────
        raw  = np.array(msg.data).reshape(height, width)
        grid = np.zeros_like(raw, dtype=np.int8)
        grid[raw == -1]               = UNKNOWN
        grid[raw == 0]                = FREE
        grid[(raw > 0) & (raw < 50)] = FREE
        grid[raw >= 50]               = OCCUPIED

        # ── WFD cell state labels ─────────────────────
        MAP_OPEN   = 1
        MAP_CLOSED = 2
        FRO_OPEN   = 3
        FRO_CLOSED = 4
        state = np.zeros((height, width), dtype=np.int8)

        def neighbours4(r, c):
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width:
                    yield nr, nc

        def is_frontier(r, c):
            # Frontier point = FREE cell with at least one UNKNOWN neighbour
            return grid[r, c] == FREE and any(
                grid[nr, nc] == UNKNOWN for nr, nc in neighbours4(r, c))

        frontiers = []   # collected NewFrontier lists

        # ── Outer BFS: queue_m — lines 1-3 ───────────
        queue_m = deque()
        sr, sc = grid_y, grid_x
        queue_m.append((sr, sc))
        state[sr, sc] = MAP_OPEN                                # line 3

        # ── lines 4-30 ────────────────────────────────
        while queue_m:                                          # line 4
            pr, pc = queue_m.popleft()                          # line 5

            if state[pr, pc] == MAP_CLOSED:                     # line 6
                continue                                         # line 7

            if is_frontier(pr, pc):                             # line 8

                # ── Inner BFS: queue_f — lines 9-25 ───
                queue_f = deque()
                new_frontier = []                               # line 10
                queue_f.append((pr, pc))                        # line 11
                state[pr, pc] = FRO_OPEN                        # line 12

                while queue_f:                                  # line 13
                    qr, qc = queue_f.popleft()                  # line 14

                    if state[qr, qc] in (MAP_CLOSED, FRO_CLOSED):  # line 15
                        continue                                 # line 16

                    if is_frontier(qr, qc):                     # line 17
                        new_frontier.append((qr, qc))           # line 18
                        for nr, nc in neighbours4(qr, qc):      # line 19
                            if state[nr, nc] not in (           # line 20
                                    FRO_OPEN, FRO_CLOSED, MAP_CLOSED):
                                queue_f.append((nr, nc))        # line 21
                                state[nr, nc] = FRO_OPEN        # line 22

                    state[qr, qc] = FRO_CLOSED                  # line 23

                frontiers.append(new_frontier)                  # line 24
                for fr, fc in new_frontier:                     # line 25
                    state[fr, fc] = MAP_CLOSED

            # ── Expand outer BFS — lines 26-30 ────────
            for nr, nc in neighbours4(pr, pc):                  # line 26
                if (state[nr, nc] not in (MAP_OPEN, MAP_CLOSED) # line 27
                        and grid[nr, nc] != OCCUPIED
                        and any(grid[ar, ac] == FREE
                                for ar, ac in neighbours4(nr, nc))):
                    queue_m.append((nr, nc))                    # line 28
                    state[nr, nc] = MAP_OPEN                    # line 29

            state[pr, pc] = MAP_CLOSED                          # line 30

        # ── Filter small and blacklisted frontiers ────
        valid = []
        for frontier in frontiers:
            if len(frontier) < MIN_FRONTIER_SIZE:
                continue

            rows = [c[0] for c in frontier]
            cols = [c[1] for c in frontier]
            wx = origin.x + np.mean(cols) * res
            wy = origin.y + np.mean(rows) * res

            if self.is_blacklisted(wx, wy):
                continue

            valid.append({
                'cells':    frontier,
                'centroid': (wx, wy),
                'size':     len(frontier)
            })

        if not valid:
            self.get_logger().info('No valid frontiers found')
            self.publish_frontiers([])
            return

        # ── Rank: score = 0.7*(1/dist) + 0.3*norm_size
        max_size = max(c['size'] for c in valid)
        for c in valid:
            wx, wy = c['centroid']
            dist = max(math.sqrt((wx - robot_wx)**2 +
                                 (wy - robot_wy)**2), 0.01)
            c['score'] = 0.7 * (1.0 / dist) + 0.3 * (c['size'] / max_size)

        valid.sort(key=lambda c: c['score'], reverse=True)
        best = valid[0]

        # ── Publish best frontier goal ─────────────────
        goal = PoseStamped()
        goal.header.frame_id    = 'map'
        goal.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.position.x    = best['centroid'][0]
        goal.pose.position.y    = best['centroid'][1]
        goal.pose.position.z    = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        self.get_logger().info(
            f'Best frontier: ({best["centroid"][0]:.2f}, '
            f'{best["centroid"][1]:.2f}) '
            f'size={best["size"]} score={best["score"]:.3f} '
            f'total_valid={len(valid)}')

        self.publish_frontiers(valid)

    # ──────────────────────────────────────────────────────
    # Blacklist helpers
    # ──────────────────────────────────────────────────────
    def is_blacklisted(self, wx, wy):
        for bx, by in self.blacklist:
            if math.sqrt((wx - bx)**2 + (wy - by)**2) < BLACKLIST_RADIUS:
                return True
        return False

    def add_to_blacklist(self, wx, wy):
        self.blacklist.append((wx, wy))
        self.get_logger().info(f'Blacklisted frontier at ({wx:.2f}, {wy:.2f})')

    # ──────────────────────────────────────────────────────
    # Publish MarkerArray for RViz
    # ──────────────────────────────────────────────────────
    def publish_frontiers(self, clusters):
        marker_array = MarkerArray()

        clear_marker        = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, cluster in enumerate(clusters):
            m                    = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'frontiers'
            m.id                 = i
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = cluster['centroid'][0]
            m.pose.position.y    = cluster['centroid'][1]
            m.pose.position.z    = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x            = 0.2
            m.scale.y            = 0.2
            m.scale.z            = 0.2

            # Best = green, rest = blue
            if i == 0:
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
            else:
                m.color.r = 0.0
                m.color.g = 0.0
                m.color.b = 1.0
            m.color.a = 0.8

            marker_array.markers.append(m)

        self.frontier_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
