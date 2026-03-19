#!/usr/bin/env python3
"""
Autonomous navigation + coverage monitoring for CDE2310 AMR.

Single-file consolidation of pathfinding algorithms, the AutoNav
state-machine node, and the CoverageMonitor benchmarking node.

Run as two separate entry points:
  ros2 run amr_nav auto_nav
  ros2 run amr_nav coverage_monitor

Or launch together via:
  ros2 launch amr_nav mission.launch.py
  ros2 launch amr_nav sim_exploration.launch.py

State machine
─────────────
IDLE → FINDING_TARGET → PLANNING_PATH → NAVIGATING → (loop)
   TAG_INTERRUPT → APPROACHING → ALIGNING → DELIVERING → FINDING_TARGET
   No target → RECOVERY (360° spin) → FINDING_TARGET
   Coverage complete / all stations → DONE

Topics
──────
Sub  /map              OccupancyGrid   SLAM map (gated by can_update)
Sub  /odom             Odometry        Robot pose
Sub  /scan             LaserScan       Emergency stop
Sub  /marker_detection String(JSON)    AprilTag interrupt
Sub  /delivery_status  String(JSON)    Delivery feedback
Sub  /map_closed       Bool            Coverage complete
Pub  /cmd_vel          Twist           Robot velocity
Pub  /delivery_cmd     Int32           Ball delivery (multi)
Pub  /delivery_drop_single Int32       Ball delivery (single, Station B)
Pub  /map_closed       Bool            Coverage monitor signal
Pub  /exploration_metrics String(JSON) Benchmark results
"""

import heapq
import json
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSProfile

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Int32


# ═══════════════════════════════════════════════════════════
# PATHFINDING ALGORITHMS  (pure numpy, no ROS dependency)
# ═══════════════════════════════════════════════════════════

def occupancy_to_grid(data, width, height, wall_threshold=10):
    """Convert flat OccupancyGrid data to a 2D binary grid.

    Args:
        data: Flat list of occupancy values (-1=unknown, 0-100=probability).
        width: Grid width.
        height: Grid height.
        wall_threshold: Values >= this are considered occupied.

    Returns:
        2D numpy array (height x width), 0=free, 1=occupied.
    """
    if width == 0 or height == 0:
        return np.zeros((height, width), dtype=np.int8)

    arr = np.array(data, dtype=np.int16).reshape((height, width))
    grid = np.zeros((height, width), dtype=np.int8)
    grid[(arr == -1) | (arr >= wall_threshold)] = 1
    return grid


def find_next_target(grid, visited_cells, distance_threshold=35):
    """Multi-source Dijkstra to find the most-unexplored reachable cell.

    All visited cells start as sources with cost=0. Dijkstra expands
    through FREE cells. The highest-cost reachable FREE cell that is
    NOT in the visited set is the target (furthest from all visited
    areas = most unexplored).

    Args:
        grid: 2D numpy array (0=free, 1=occupied).
        visited_cells: Set of (row, col) tuples that have been visited.
        distance_threshold: Max cost to consider.

    Returns:
        (row, col) of the target cell, or None if no target found.
    """
    if not visited_cells:
        return None

    height, width = grid.shape
    dist = np.full((height, width), np.inf)

    pq = []
    for r, c in visited_cells:
        if 0 <= r < height and 0 <= c < width and grid[r, c] == 0:
            dist[r, c] = 0
            heapq.heappush(pq, (0, r, c))

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    best_cell = None
    best_cost = -1

    while pq:
        cost, r, c = heapq.heappop(pq)

        if cost > dist[r, c]:
            continue
        if cost > distance_threshold:
            continue

        if cost > best_cost and (r, c) not in visited_cells:
            best_cost = cost
            best_cell = (r, c)

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width and grid[nr, nc] == 0:
                new_cost = cost + 1
                if new_cost < dist[nr, nc]:
                    dist[nr, nc] = new_cost
                    heapq.heappush(pq, (new_cost, nr, nc))

    return best_cell


def _precompute_wall_proximity(grid, radius):
    """Vectorised wall proximity using integral image (summed-area table)."""
    binary = (grid == 1).astype(np.int64)
    h, w = binary.shape
    padded = np.pad(binary, radius, mode='constant', constant_values=0)

    sat = np.zeros((h + 2 * radius + 1, w + 2 * radius + 1), dtype=np.int64)
    sat[1:, 1:] = np.cumsum(np.cumsum(padded, axis=0), axis=1)

    k = 2 * radius + 1
    result = sat[k:, k:] - sat[:h, k:] - sat[k:, :w] + sat[:h, :w]
    return result.astype(np.int32)


def astar_wall_penalty(grid, start, goal, wall_penalty=5, wall_cost=200):
    """A* pathfinding with wall proximity penalty.

    For each candidate cell, the number of occupied cells within a
    square neighbourhood of radius *wall_penalty* is counted and
    multiplied by *wall_cost* to increase the movement cost, naturally
    steering paths away from walls.

    Args:
        grid: 2D numpy array (0=free, 1=occupied).
        start: (row, col) start position.
        goal: (row, col) goal position.
        wall_penalty: Neighbourhood scan radius for wall detection.
        wall_cost: Cost multiplier per nearby wall cell.

    Returns:
        List of (row, col) from start to goal, or empty list if no path.
    """
    height, width = grid.shape
    sr, sc = start
    gr, gc = goal

    if not (0 <= sr < height and 0 <= sc < width):
        return []
    if not (0 <= gr < height and 0 <= gc < width):
        return []
    if grid[sr, sc] == 1 or grid[gr, gc] == 1:
        return []
    if start == goal:
        return [start]

    wall_prox = _precompute_wall_proximity(grid, wall_penalty)

    def heuristic(r, c):
        return abs(r - gr) + abs(c - gc)

    open_set = [(heuristic(sr, sc), 0, sr, sc)]
    g_score = {(sr, sc): 0}
    came_from = {}
    closed = set()
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while open_set:
        _f, g, r, c = heapq.heappop(open_set)

        if (r, c) in closed:
            continue
        closed.add((r, c))

        if r == gr and c == gc:
            path = [(r, c)]
            while (r, c) in came_from:
                r, c = came_from[(r, c)]
                path.append((r, c))
            path.reverse()
            return path

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            if grid[nr, nc] == 1 or (nr, nc) in closed:
                continue

            move_cost = 1 + int(wall_prox[nr, nc]) * wall_cost
            new_g = g + move_cost

            if new_g < g_score.get((nr, nc), float('inf')):
                g_score[(nr, nc)] = new_g
                came_from[(nr, nc)] = (r, c)
                heapq.heappush(open_set, (new_g + heuristic(nr, nc),
                                          new_g, nr, nc))

    return []


def cluster_path(path, cluster_distance=6):
    """Reduce a dense path to cluster waypoints.

    Keeps only waypoints that are at least *cluster_distance* grid cells
    apart (Manhattan distance). Always includes start and end points.

    Args:
        path: List of (row, col) tuples (dense path from A*).
        cluster_distance: Minimum spacing between waypoints.

    Returns:
        List of (row, col) cluster waypoints.
    """
    if len(path) <= 1:
        return list(path)

    clusters = [path[0]]

    for point in path[1:]:
        lr, lc = clusters[-1]
        pr, pc = point
        if abs(pr - lr) + abs(pc - lc) >= cluster_distance:
            clusters.append(point)

    if clusters[-1] != path[-1]:
        clusters.append(path[-1])

    return clusters


# ═══════════════════════════════════════════════════════════
# COVERAGE MONITOR NODE
# ═══════════════════════════════════════════════════════════

class CoverageMonitor(Node):
    """Tracks SLAM map coverage and publishes /map_closed when done."""

    def __init__(self):
        super().__init__('coverage_monitor')

        # ── Parameters ───────────────────────────────────
        self.declare_parameter('coverage_threshold', 0.95)
        self.declare_parameter('stable_count_threshold', 15)
        self.declare_parameter('check_interval', 2.0)

        self.coverage_threshold = (
            self.get_parameter('coverage_threshold').value)
        self.stable_count_threshold = (
            self.get_parameter('stable_count_threshold').value)
        check_interval = self.get_parameter('check_interval').value

        # ── State ────────────────────────────────────────
        self.latest_map = None
        self.start_time = None
        self.last_coverage = 0.0
        self.stable_count = 0
        self.completed = False

        # ── QoS ──────────────────────────────────────────
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )
        map_closed_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────
        self.map_closed_pub = self.create_publisher(
            Bool, '/map_closed', map_closed_qos)
        self.metrics_pub = self.create_publisher(
            String, '/exploration_metrics', 10)

        # ── Subscribers ──────────────────────────────────
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)

        # ── Periodic check timer ─────────────────────────
        self.create_timer(check_interval, self._check_coverage)

        self.get_logger().info(
            f'Coverage monitor started '
            f'(threshold={self.coverage_threshold}, '
            f'stable_checks={self.stable_count_threshold})')

    def _map_cb(self, msg):
        if self.start_time is None:
            self.start_time = time.monotonic()
            self.get_logger().info('First map received — timing started.')
        self.latest_map = msg

    def _check_coverage(self):
        if self.latest_map is None or self.completed:
            return

        data = self.latest_map.data
        total_cells = len(data)
        if total_cells == 0:
            return

        known = 0
        free = 0
        occupied = 0
        for v in data:
            if v != -1:
                known += 1
                if v == 0:
                    free += 1
                else:
                    occupied += 1

        coverage = known / total_cells
        elapsed = time.monotonic() - self.start_time

        self.get_logger().info(
            f'Coverage: {coverage * 100:.1f}% | '
            f'Known: {known}/{total_cells} '
            f'(free={free}, occ={occupied}) | '
            f'Elapsed: {elapsed:.1f}s')

        delta = abs(coverage - self.last_coverage)
        if delta < 0.001:
            self.stable_count += 1
        else:
            self.stable_count = 0
        self.last_coverage = coverage

        if coverage >= self.coverage_threshold:
            self.get_logger().info(
                f'Coverage threshold reached: {coverage * 100:.1f}%')
            self._complete(
                coverage, elapsed, known, free, occupied, total_cells)
        elif self.stable_count >= self.stable_count_threshold:
            self.get_logger().info(
                f'Coverage stabilised at {coverage * 100:.1f}%')
            self._complete(
                coverage, elapsed, known, free, occupied, total_cells)

    def _complete(self, coverage, elapsed, known, free, occupied, total):
        self.completed = True

        self.get_logger().info(
            f'=== Exploration complete! ==='
            f'\n  Coverage : {coverage * 100:.1f}%'
            f'\n  Elapsed  : {elapsed:.1f} seconds'
            f'\n  Known    : {known} / {total}'
            f'\n  Free     : {free}'
            f'\n  Occupied : {occupied}')

        self.map_closed_pub.publish(Bool(data=True))

        metrics = {
            'coverage_pct': round(coverage * 100, 2),
            'elapsed_seconds': round(elapsed, 2),
            'known_cells': known,
            'free_cells': free,
            'occupied_cells': occupied,
            'total_cells': total,
        }
        self.metrics_pub.publish(String(data=json.dumps(metrics)))


# ═══════════════════════════════════════════════════════════
# AUTO NAV NODE
# ═══════════════════════════════════════════════════════════

# ── States ──────────────────────────────────────────────────
IDLE = 'IDLE'
FINDING_TARGET = 'FINDING_TARGET'
PLANNING_PATH = 'PLANNING_PATH'
NAVIGATING = 'NAVIGATING'
RECOVERY = 'RECOVERY'
DONE = 'DONE'
TAG_INTERRUPT = 'TAG_INTERRUPT'
APPROACHING = 'APPROACHING'
ALIGNING = 'ALIGNING'
DELIVERING = 'DELIVERING'

# ── Station mapping ─────────────────────────────────────────
STATION_MAP = {0: 'A', 1: 'B', 2: 'C'}
BALLS_PER_STATION = 3
TOTAL_STATIONS = 3


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('rotate_speed', 1.75)
        self.declare_parameter('drive_speed', 0.15)
        self.declare_parameter('wall_threshold', 10)
        self.declare_parameter('distance_threshold', 35)
        self.declare_parameter('wall_penalty', 5)
        self.declare_parameter('wall_cost', 200)
        self.declare_parameter('cluster_distance', 6)
        self.declare_parameter('localization_tolerance', 4)
        self.declare_parameter('stop_distance', 0.25)
        self.declare_parameter('reset_visited_threshold', 500)
        self.declare_parameter('tag_approach_distance', 0.35)
        self.declare_parameter('recovery_duration', 6.0)

        self.rotate_speed = self.get_parameter('rotate_speed').value
        self.drive_speed = self.get_parameter('drive_speed').value
        self.wall_threshold = self.get_parameter('wall_threshold').value
        self.distance_threshold = (
            self.get_parameter('distance_threshold').value)
        self.wall_penalty_radius = self.get_parameter('wall_penalty').value
        self.wall_cost_mult = self.get_parameter('wall_cost').value
        self.cluster_dist = self.get_parameter('cluster_distance').value
        self.loc_tolerance = (
            self.get_parameter('localization_tolerance').value)
        self.stop_distance = self.get_parameter('stop_distance').value
        self.reset_visited_thr = (
            self.get_parameter('reset_visited_threshold').value)
        self.tag_approach_dist = (
            self.get_parameter('tag_approach_distance').value)
        self.recovery_dur = self.get_parameter('recovery_duration').value

        # ── Map state ───────────────────────────────────────
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.can_update = True
        self.grid = None

        # ── Robot state ─────────────────────────────────────
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.visited_world = set()

        # ── Navigation state ────────────────────────────────
        self.state = IDLE
        self.waypoints = []
        self.waypoint_idx = 0
        self.nav_target = None
        self.recovery_start = None
        self.no_target_count = 0

        # ── Tag / delivery state ────────────────────────────
        self.active_marker = None
        self.delivered_set = set()
        self.latest_delivery_status = None
        self.delivery_sent = False
        self.delivery_ball_idx = 0
        self.delivery_wait_start = None
        self.align_start = None

        # ── LiDAR state ────────────────────────────────────
        self.obstacle_ahead = False

        # ── QoS ─────────────────────────────────────────────
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )
        map_closed_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        # ── Publishers ──────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.delivery_cmd_pub = self.create_publisher(
            Int32, '/delivery_cmd', 10)
        self.delivery_single_pub = self.create_publisher(
            Int32, '/delivery_drop_single', 10)

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(
            String, '/marker_detection', self._marker_cb, 10)
        self.create_subscription(
            String, '/delivery_status', self._delivery_status_cb, 10)
        self.create_subscription(
            Bool, '/map_closed', self._map_closed_cb, map_closed_qos)

        # ── Main loop (10 Hz) ──────────────────────────────
        self.create_timer(0.1, self._tick)

        self.get_logger().info('AutoNav node started')

    # ════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════

    def _map_cb(self, msg):
        if not self.can_update:
            return
        self.map_data = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.grid = occupancy_to_grid(
            self.map_data, self.map_width, self.map_height,
            self.wall_threshold)

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

        res = self.map_resolution if self.map_resolution > 0 else 0.05
        wx = round(self.robot_x / res) * res
        wy = round(self.robot_y / res) * res
        self.visited_world.add((wx, wy))

    def _scan_cb(self, msg):
        if not msg.ranges:
            return
        cone_half = 15.0 * math.pi / 180.0
        min_front = float('inf')
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= cone_half and r < min_front:
                min_front = r
        self.obstacle_ahead = min_front < self.stop_distance

    def _marker_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        tag_id = data.get('tag_id')
        if tag_id is not None and tag_id not in self.delivered_set:
            self.active_marker = data
            self.get_logger().info(
                f'Tag {tag_id} detected — interrupting')

    def _delivery_status_cb(self, msg):
        try:
            self.latest_delivery_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _map_closed_cb(self, msg):
        if msg.data:
            self.get_logger().info('Map closed signal received')
            delivery_states = (
                TAG_INTERRUPT, APPROACHING, ALIGNING, DELIVERING)
            if self.state not in delivery_states:
                self._transition(DONE)

    # ════════════════════════════════════════════════════════
    # COORDINATE HELPERS
    # ════════════════════════════════════════════════════════

    def _world_to_grid(self, wx, wy):
        if self.map_resolution <= 0:
            return None, None
        col = int((wx - self.map_origin_x) / self.map_resolution)
        row = int((wy - self.map_origin_y) / self.map_resolution)
        if 0 <= col < self.map_width and 0 <= row < self.map_height:
            return col, row
        return None, None

    def _grid_to_world(self, row, col):
        wx = col * self.map_resolution + self.map_origin_x
        wy = row * self.map_resolution + self.map_origin_y
        return wx, wy

    # ════════════════════════════════════════════════════════
    # STATE MACHINE
    # ════════════════════════════════════════════════════════

    def _transition(self, new_state):
        self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def _normalize(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _tick(self):
        if (self.active_marker is not None
                and self.state not in (
                    TAG_INTERRUPT, APPROACHING, ALIGNING,
                    DELIVERING, DONE)):
            self._stop()
            self._transition(TAG_INTERRUPT)

        handler = {
            IDLE: self._idle,
            FINDING_TARGET: self._finding_target,
            PLANNING_PATH: self._planning_path,
            NAVIGATING: self._navigating,
            RECOVERY: self._recovery,
            DONE: self._done,
            TAG_INTERRUPT: self._tag_interrupt,
            APPROACHING: self._approaching,
            ALIGNING: self._aligning,
            DELIVERING: self._delivering,
        }.get(self.state)
        if handler:
            handler()

    # ── IDLE ─────────────────────────────────────────────

    def _idle(self):
        if self.grid is not None:
            self._transition(FINDING_TARGET)

    # ── FINDING_TARGET ───────────────────────────────────

    def _finding_target(self):
        if self.grid is None:
            return

        visited_grid = set()
        for wx, wy in self.visited_world:
            col, row = self._world_to_grid(wx, wy)
            if col is not None:
                visited_grid.add((row, col))

        col, row = self._world_to_grid(self.robot_x, self.robot_y)
        if col is not None:
            visited_grid.add((row, col))

        if not visited_grid:
            self._transition(RECOVERY)
            return

        target = find_next_target(
            self.grid, visited_grid, self.distance_threshold)

        if target is None:
            self.no_target_count += 1
            if self.no_target_count >= 3:
                if len(self.visited_world) > self.reset_visited_thr:
                    self.get_logger().info('Resetting visited set')
                    self.visited_world.clear()
                    self.no_target_count = 0
                    return
                self._transition(DONE)
            else:
                self._transition(RECOVERY)
            return

        self.no_target_count = 0
        self.nav_target = target
        self._transition(PLANNING_PATH)

    # ── PLANNING_PATH ────────────────────────────────────

    def _planning_path(self):
        col, row = self._world_to_grid(self.robot_x, self.robot_y)
        if col is None:
            self._transition(RECOVERY)
            return

        start = (row, col)
        path = astar_wall_penalty(
            self.grid, start, self.nav_target,
            self.wall_penalty_radius, self.wall_cost_mult)

        if not path:
            self.get_logger().warn('A* found no path — recovering')
            self._transition(RECOVERY)
            return

        clustered = cluster_path(path, self.cluster_dist)

        self.waypoints = []
        for r, c in clustered:
            wx, wy = self._grid_to_world(r, c)
            self.waypoints.append((wx, wy))

        self.waypoint_idx = 0
        self.can_update = False
        self._transition(NAVIGATING)

    # ── NAVIGATING ───────────────────────────────────────

    def _navigating(self):
        if self.waypoint_idx >= len(self.waypoints):
            self._stop()
            self.can_update = True
            self._transition(FINDING_TARGET)
            return

        if self.obstacle_ahead:
            self._stop()
            self.can_update = True
            self.get_logger().warn('Obstacle — replanning')
            self._transition(FINDING_TARGET)
            return

        wx, wy = self.waypoints[self.waypoint_idx]

        col, row = self._world_to_grid(self.robot_x, self.robot_y)
        tcol, trow = self._world_to_grid(wx, wy)
        if col is not None and tcol is not None:
            dist = abs(row - trow) + abs(col - tcol)
            if dist <= self.loc_tolerance:
                self.waypoint_idx += 1
                return

        dx = wx - self.robot_x
        dy = wy - self.robot_y
        target_yaw = math.atan2(dy, dx)
        yaw_err = self._normalize(target_yaw - self.robot_yaw)

        twist = Twist()
        if abs(yaw_err) > 0.3:
            twist.angular.z = (
                self.rotate_speed if yaw_err > 0 else -self.rotate_speed)
        else:
            twist.linear.x = self.drive_speed
            twist.angular.z = yaw_err * 1.0
        self.cmd_vel_pub.publish(twist)

    # ── RECOVERY ─────────────────────────────────────────

    def _recovery(self):
        if self.recovery_start is None:
            self.recovery_start = time.monotonic()
            self.get_logger().info('Recovery: 360 deg spin')

        if time.monotonic() - self.recovery_start >= self.recovery_dur:
            self._stop()
            self.recovery_start = None
            self.can_update = True
            self._transition(FINDING_TARGET)
            return

        twist = Twist()
        twist.angular.z = self.rotate_speed
        self.cmd_vel_pub.publish(twist)

    # ── DONE ─────────────────────────────────────────────

    def _done(self):
        self._stop()
        self.can_update = True

    # ── TAG_INTERRUPT ────────────────────────────────────

    def _tag_interrupt(self):
        if self.active_marker is None:
            self._transition(FINDING_TARGET)
            return

        tag_id = self.active_marker.get('tag_id')
        station = STATION_MAP.get(tag_id, '?')
        self.get_logger().info(
            f'Tag interrupt: tag {tag_id} -> Station {station}')
        self.can_update = True
        self._transition(APPROACHING)

    # ── APPROACHING ──────────────────────────────────────

    def _approaching(self):
        if self.active_marker is None:
            self._transition(FINDING_TARGET)
            return

        map_pose = self.active_marker.get('map_pose')
        if map_pose is None:
            self.align_start = None
            self._transition(ALIGNING)
            return

        mx, my = map_pose['x'], map_pose['y']
        dx = mx - self.robot_x
        dy = my - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self.tag_approach_dist:
            self._stop()
            self.align_start = None
            self._transition(ALIGNING)
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = self._normalize(target_yaw - self.robot_yaw)

        twist = Twist()
        if abs(yaw_err) > 0.3:
            twist.angular.z = (
                self.rotate_speed if yaw_err > 0 else -self.rotate_speed)
        else:
            twist.linear.x = self.drive_speed
            twist.angular.z = yaw_err * 1.0
        self.cmd_vel_pub.publish(twist)

    # ── ALIGNING ─────────────────────────────────────────

    def _aligning(self):
        if self.align_start is None:
            self.align_start = time.monotonic()

        if time.monotonic() - self.align_start > 10.0:
            self._stop()
            self.get_logger().warn('Alignment timeout — delivering anyway')
            self._begin_delivery()
            return

        if self.active_marker is None:
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_vel_pub.publish(twist)
            return

        camera_pose = self.active_marker.get('camera_pose', {})
        x_offset = camera_pose.get('x', 0.0)
        distance = self.active_marker.get('distance', 999.0)
        angular_err = -x_offset

        if abs(angular_err) < 0.02 and distance < 0.5:
            self._stop()
            self.get_logger().info('Aligned with marker')
            self._begin_delivery()
            return

        twist = Twist()
        twist.angular.z = max(-0.5, min(0.5, angular_err * 1.0))
        if distance > 0.3:
            twist.linear.x = 0.05
        self.cmd_vel_pub.publish(twist)

    def _begin_delivery(self):
        self.delivery_sent = False
        self.delivery_ball_idx = 0
        self.delivery_wait_start = None
        self._transition(DELIVERING)

    # ── DELIVERING ───────────────────────────────────────

    def _delivering(self):
        tag_id = (self.active_marker.get('tag_id')
                  if self.active_marker else None)
        station = STATION_MAP.get(tag_id, 'A')

        if station == 'B':
            self._deliver_dynamic()
        else:
            self._deliver_static()

    def _deliver_static(self):
        """Station A / C: drop all balls at once."""
        if not self.delivery_sent:
            msg = Int32()
            msg.data = BALLS_PER_STATION
            self.delivery_cmd_pub.publish(msg)
            self.delivery_sent = True
            self.get_logger().info(
                f'Static delivery: {BALLS_PER_STATION} balls')
            return

        if self.latest_delivery_status is not None:
            status = self.latest_delivery_status.get('status', '')
            if status in ('complete', 'partial'):
                self._mark_delivered()

    def _deliver_dynamic(self):
        """Station B: drop one ball at a time while tracking."""
        if self.active_marker is not None:
            camera_pose = self.active_marker.get('camera_pose', {})
            x_offset = camera_pose.get('x', 0.0)
            twist = Twist()
            twist.angular.z = max(-0.5, min(0.5, -1.5 * x_offset))
            self.cmd_vel_pub.publish(twist)

        if self.delivery_ball_idx >= BALLS_PER_STATION:
            self._stop()
            self._mark_delivered()
            return

        if not self.delivery_sent:
            msg = Int32()
            msg.data = self.delivery_ball_idx + 1
            self.delivery_single_pub.publish(msg)
            self.delivery_sent = True
            self.delivery_wait_start = time.monotonic()
            self.get_logger().info(
                f'Dynamic ball {self.delivery_ball_idx + 1}')
            return

        if (self.delivery_wait_start is not None
                and time.monotonic() - self.delivery_wait_start > 3.0):
            self.delivery_ball_idx += 1
            self.delivery_sent = False
            self.delivery_wait_start = None

    def _mark_delivered(self):
        tag_id = (self.active_marker.get('tag_id')
                  if self.active_marker else None)
        if tag_id is not None:
            self.delivered_set.add(tag_id)
            station = STATION_MAP.get(tag_id, '?')
            self.get_logger().info(
                f'Station {station} (tag {tag_id}) DONE. '
                f'Delivered: {self.delivered_set}')

        self.active_marker = None
        self.latest_delivery_status = None
        self.align_start = None

        if len(self.delivered_set) >= TOTAL_STATIONS:
            self.get_logger().info(
                '*** ALL STATIONS DELIVERED — MISSION COMPLETE ***')
            self._transition(DONE)
        else:
            self.can_update = True
            self._transition(FINDING_TARGET)


# ═══════════════════════════════════════════════════════════
# ENTRY POINTS
# ═══════════════════════════════════════════════════════════

def main(args=None):
    """Entry point for the auto_nav node."""
    rclpy.init(args=args)
    node = AutoNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_coverage(args=None):
    """Entry point for the coverage_monitor node."""
    rclpy.init(args=args)
    node = CoverageMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
