#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

import py_trees


# ═══════════════════════════════════════════════════════════
# BLACKBOARD KEYS
# ═══════════════════════════════════════════════════════════
BB_MAP_CLOSED       = 'map_closed'
BB_CURRENT_FRONTIER = 'current_frontier'

# Global node reference — lets BT leaves access ROS2 interfaces
_node_ref = None


# ═══════════════════════════════════════════════════════════
# ── CONDITION: IsMapClosed ──────────────────────────────
# ═══════════════════════════════════════════════════════════
class IsMapClosed(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('IsMapClosed?')
        self.bb = py_trees.blackboard.Client(name='IsMapClosed')
        self.bb.register_key(BB_MAP_CLOSED, access=py_trees.common.Access.READ)

    def update(self):
        if self.bb.get(BB_MAP_CLOSED):
            _node_ref.get_logger().info('Map is closed — coverage threshold met')
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


# ═══════════════════════════════════════════════════════════
# ── ACTION: MissionComplete ─────────────────────────────
# ═══════════════════════════════════════════════════════════
class MissionComplete(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('MissionComplete')

    def update(self):
        _node_ref.get_logger().info('✓ MISSION COMPLETE — map fully explored')
        return py_trees.common.Status.SUCCESS


# ═══════════════════════════════════════════════════════════
# ── ACTION: GetBestFrontier ─────────────────────────────
# Reads latest /best_frontier and writes to blackboard
# ═══════════════════════════════════════════════════════════
class GetBestFrontier(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('GetBestFrontier')
        self.bb = py_trees.blackboard.Client(name='GetBestFrontier')
        self.bb.register_key(
            BB_CURRENT_FRONTIER, access=py_trees.common.Access.WRITE)

    def update(self):
        frontier = _node_ref.latest_frontier
        if frontier is None:
            _node_ref.get_logger().warn('No frontier available')
            return py_trees.common.Status.FAILURE

        self.bb.set(BB_CURRENT_FRONTIER, frontier)
        _node_ref.get_logger().info(
            f'Frontier selected: '
            f'({frontier.pose.position.x:.2f}, {frontier.pose.position.y:.2f})')
        return py_trees.common.Status.SUCCESS


# ═══════════════════════════════════════════════════════════
# ── ACTION: NavigateToFrontier ──────────────────────────
# Sends Nav2 A* goal and waits for result
# ═══════════════════════════════════════════════════════════
class NavigateToFrontier(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('NavigateToFrontier')
        self.bb = py_trees.blackboard.Client(name='NavigateToFrontier')
        self.bb.register_key(
            BB_CURRENT_FRONTIER, access=py_trees.common.Access.READ)
        self._goal_handle = None
        self._result      = None
        self._sent_goal   = False

    def initialise(self):
        self._goal_handle = None
        self._result      = None
        self._sent_goal   = False

    def update(self):
        # ── Send goal once ──────────────────────────
        if not self._sent_goal:
            frontier = self.bb.get(BB_CURRENT_FRONTIER)
            if frontier is None:
                return py_trees.common.Status.FAILURE

            if not _node_ref.nav_client.wait_for_server(timeout_sec=2.0):
                _node_ref.get_logger().warn('Nav2 not available')
                return py_trees.common.Status.FAILURE

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = frontier
            goal_msg.pose.header.stamp = _node_ref.get_clock().now().to_msg()

            future = _node_ref.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self._goal_response_cb)
            self._sent_goal = True
            _node_ref.get_logger().info(
                f'Sending Nav2 goal: '
                f'({frontier.pose.position.x:.2f}, {frontier.pose.position.y:.2f})')
            return py_trees.common.Status.RUNNING

        # ── Wait for result ─────────────────────────
        if self._result is None:
            return py_trees.common.Status.RUNNING

        if self._result == 'success':
            # Blacklist reached frontier so we don't revisit it
            frontier = self.bb.get(BB_CURRENT_FRONTIER)
            if frontier:
                _node_ref.blacklist_frontier(
                    frontier.pose.position.x,
                    frontier.pose.position.y)
            return py_trees.common.Status.SUCCESS
        else:
            # Blacklist failed frontier
            frontier = self.bb.get(BB_CURRENT_FRONTIER)
            if frontier:
                _node_ref.blacklist_frontier(
                    frontier.pose.position.x,
                    frontier.pose.position.y)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        # Cancel goal if BT preempts this node
        if self._goal_handle is not None and self._result is None:
            _node_ref.get_logger().info('Cancelling nav goal')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            _node_ref.get_logger().warn('Nav2 rejected goal')
            self._result = 'failed'
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        if status == 4:   # SUCCEEDED
            self._result = 'success'
            _node_ref.get_logger().info('Reached frontier')
        else:
            self._result = 'failed'
            _node_ref.get_logger().warn(f'Nav2 failed, status: {status}')


# ═══════════════════════════════════════════════════════════
# ── ACTION: RecoveryRotation ────────────────────────────
# 360° spin when no frontiers remain
# ═══════════════════════════════════════════════════════════
class RecoveryRotation(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('RecoveryRotation')
        self._start_time = None
        self._duration   = 6.0   # seconds for full rotation

    def initialise(self):
        self._start_time = _node_ref.get_clock().now()
        _node_ref.get_logger().info('Recovery: no frontiers, spinning 360°')

    def update(self):
        twist = Twist()
        twist.angular.z = 1.0
        _node_ref.cmd_vel_pub.publish(twist)

        elapsed = (_node_ref.get_clock().now() -
                   self._start_time).nanoseconds / 1e9
        if elapsed >= self._duration:
            _node_ref.cmd_vel_pub.publish(Twist())   # stop
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        _node_ref.cmd_vel_pub.publish(Twist())   # always stop on exit


# ═══════════════════════════════════════════════════════════
# BEHAVIOUR TREE BUILDER
# ═══════════════════════════════════════════════════════════
def build_tree():
    """
    Tree structure:

    Fallback: ExploreUntilDone
    ├── Sequence: CompletionCheck
    │   ├── IsMapClosed?
    │   └── MissionComplete
    ├── Sequence: FrontierExploration
    │   ├── GetBestFrontier
    │   └── NavigateToFrontier
    └── RecoveryRotation
    """

    # ── Completion check ────────────────────────────
    completion_check = py_trees.composites.Sequence(
        name='CompletionCheck', memory=True)
    completion_check.add_children([
        IsMapClosed(),
        MissionComplete()
    ])

    # ── Frontier exploration ─────────────────────────
    frontier_exploration = py_trees.composites.Sequence(
        name='FrontierExploration', memory=False)
    frontier_exploration.add_children([
        GetBestFrontier(),
        NavigateToFrontier()
    ])

    # ── Root fallback ────────────────────────────────
    root = py_trees.composites.Selector(
        name='ExploreUntilDone', memory=False)
    root.add_children([
        completion_check,
        frontier_exploration,
        RecoveryRotation()
    ])

    return root


# ═══════════════════════════════════════════════════════════
# EXPLORATION MANAGER NODE
# ═══════════════════════════════════════════════════════════
class ExplorationManager(Node):

    def __init__(self):
        super().__init__('exploration_manager')

        # ── Nav2 action client ───────────────────────
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # ── Publishers ──────────────────────────────
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.blacklist_pub = self.create_publisher(
            Point, '/blacklist_frontier', 10)

        # ── Subscribers ─────────────────────────────
        self.create_subscription(
            PoseStamped, '/best_frontier',
            self._frontier_cb, 10)

        self.create_subscription(
            Bool, '/map_closed',
            self._map_closed_cb, 10)

        # ── State ────────────────────────────────────
        self.latest_frontier = None

        # ── Blackboard ──────────────────────────────
        self.bb = py_trees.blackboard.Client(name='ExplorationManager')
        self.bb.register_key(
            BB_MAP_CLOSED, access=py_trees.common.Access.WRITE)
        self.bb.set(BB_MAP_CLOSED, False)

        # ── Build BT ────────────────────────────────
        self.tree = py_trees.trees.BehaviourTree(build_tree())
        self.tree.setup(timeout=15)

        # ── Tick timer 500ms ─────────────────────────
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            'Exploration manager ready — ticking BT at 2Hz')

    def _frontier_cb(self, msg):
        self.latest_frontier = msg

    def _map_closed_cb(self, msg):
        self.bb.set(BB_MAP_CLOSED, msg.data)
        if msg.data:
            self.get_logger().info('Map closure confirmed')

    def blacklist_frontier(self, wx, wy):
        self.get_logger().warn(
            f'Blacklisting frontier ({wx:.2f}, {wy:.2f})')
        # Publish to frontier_detector so it filters this frontier out
        msg = Point()
        msg.x = wx
        msg.y = wy
        self.blacklist_pub.publish(msg)
        # Clear latest so BT waits for a fresh frontier next tick
        self.latest_frontier = None

    def _tick(self):
        self.tree.tick()


# ═══════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)

    global _node_ref
    node = ExplorationManager()
    _node_ref = node

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
