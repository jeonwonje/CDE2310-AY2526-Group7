#!/usr/bin/env python3
"""
Coverage monitor node for Gazebo simulation benchmarking.

Subscribes to the SLAM map, tracks exploration coverage over time,
and publishes /map_closed when coverage stabilises or hits a threshold.
Logs wall-clock timing metrics for benchmarking the wavefront frontier algorithm.
"""

import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSProfile

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String


class CoverageMonitor(Node):

    def __init__(self):
        super().__init__('coverage_monitor')

        # ── Parameters ───────────────────────────────────
        self.declare_parameter('coverage_threshold', 0.95)
        self.declare_parameter('stable_count_threshold', 15)
        self.declare_parameter('check_interval', 2.0)

        self.coverage_threshold = self.get_parameter('coverage_threshold').value
        self.stable_count_threshold = self.get_parameter('stable_count_threshold').value
        check_interval = self.get_parameter('check_interval').value

        # ── State ────────────────────────────────────────
        self.latest_map = None
        self.start_time = None
        self.last_coverage = 0.0
        self.stable_count = 0
        self.completed = False

        # ── QoS: match SLAM Toolbox transient local ──────
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────
        map_closed_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )
        self.map_closed_pub = self.create_publisher(Bool, '/map_closed', map_closed_qos)
        self.metrics_pub = self.create_publisher(String, '/exploration_metrics', 10)

        # ── Subscribers ──────────────────────────────────
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        # ── Periodic check timer ─────────────────────────
        self.timer = self.create_timer(check_interval, self.check_coverage)

        self.get_logger().info(
            f'Coverage monitor started (threshold={self.coverage_threshold}, '
            f'stable_checks={self.stable_count_threshold})')

    def map_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.monotonic()
            self.get_logger().info('First map received — timing started.')
        self.latest_map = msg

    def check_coverage(self):
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
            f'Known: {known}/{total_cells} (free={free}, occ={occupied}) | '
            f'Elapsed: {elapsed:.1f}s')

        # ── Check stability ──────────────────────────────
        delta = abs(coverage - self.last_coverage)
        if delta < 0.001:  # less than 0.1% change
            self.stable_count += 1
        else:
            self.stable_count = 0
        self.last_coverage = coverage

        # ── Completion conditions ────────────────────────
        if coverage >= self.coverage_threshold:
            self.get_logger().info(
                f'Coverage threshold reached: {coverage * 100:.1f}% >= '
                f'{self.coverage_threshold * 100:.1f}%')
            self._complete(coverage, elapsed, known, free, occupied, total_cells)
        elif self.stable_count >= self.stable_count_threshold:
            self.get_logger().info(
                f'Coverage stabilised at {coverage * 100:.1f}% for '
                f'{self.stable_count} consecutive checks.')
            self._complete(coverage, elapsed, known, free, occupied, total_cells)

    def _complete(self, coverage, elapsed, known, free, occupied, total):
        self.completed = True

        self.get_logger().info(
            f'=== Exploration complete! ==='
            f'\n  Coverage : {coverage * 100:.1f}%'
            f'\n  Elapsed  : {elapsed:.1f} seconds'
            f'\n  Known    : {known} / {total}'
            f'\n  Free     : {free}'
            f'\n  Occupied : {occupied}')

        # Publish /map_closed
        self.map_closed_pub.publish(Bool(data=True))

        # Publish metrics JSON
        metrics = {
            'coverage_pct': round(coverage * 100, 2),
            'elapsed_seconds': round(elapsed, 2),
            'known_cells': known,
            'free_cells': free,
            'occupied_cells': occupied,
            'total_cells': total,
        }
        self.metrics_pub.publish(String(data=json.dumps(metrics)))


def main(args=None):
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
