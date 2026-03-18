#!/usr/bin/env python3
"""
Mission controller for CDE2310 warehouse AMR.

Extends the exploration_manager.py BT with:
  - AprilTag detection interrupt (highest priority after completion check)
  - Station identification (A/B/C based on tag ID)
  - Delivery sequencing per station type
  - Lift API client for Station C (bonus)

BT structure:
  Fallback: MissionRoot
  |-- Seq: AllDelivered? (AllStationsComplete -> AnnounceMissionDone)
  |-- Seq: HandleDetectedMarker (interrupt branch)
  |   |-- Cond: MarkerDetected?
  |   |-- Act: CancelCurrentNavGoal
  |   |-- Act: IdentifyStation
  |   |-- Fallback: ExecuteDelivery
  |   |   |-- Seq: StationA (IsStationA -> NavigateToStation -> AlignWithMarker -> DeliverStatic)
  |   |   |-- Seq: StationB (IsStationB -> NavigateToStation -> TrackAndAlignMoving -> DeliverDynamic)
  |   |   |-- Seq: StationC (IsStationC -> NavigateToStation -> CallLiftAPI -> DeliverStatic)
  |   |-- Act: MarkStationDone
  |-- Seq: FrontierExploration (GetBestFrontier -> NavigateToFrontier)
  |-- RecoveryRotation
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import Bool, String, Int32
from nav2_msgs.action import NavigateToPose

import py_trees

try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False


# ═══════════════════════════════════════════════════════════
# BLACKBOARD KEYS
# ═══════════════════════════════════════════════════════════
BB_MAP_CLOSED       = 'map_closed'
BB_CURRENT_FRONTIER = 'current_frontier'
BB_ACTIVE_MARKER    = 'active_marker'      # dict from apriltag_detector JSON
BB_STATION_TYPE     = 'station_type'       # 'A', 'B', or 'C'
BB_DELIVERED_SET    = 'delivered_set'       # set of completed tag IDs
BB_DELIVERY_STATUS  = 'delivery_status'    # latest from delivery_sequencer

# Station mapping: tag_id -> station type
# Adjust these IDs to match the actual AprilTags used in the course
STATION_MAP = {
    0: 'A',
    1: 'B',
    2: 'C',
}

BALLS_PER_STATION = 3
TOTAL_STATIONS = 3

# Approach offset: stop this far from the marker (metres)
APPROACH_OFFSET = 0.35

# Global node reference for BT leaves
_node_ref = None


# ═══════════════════════════════════════════════════════════
# CONDITION: AllStationsComplete
# ═══════════════════════════════════════════════════════════
class AllStationsComplete(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('AllStationsComplete?')
        self.bb = py_trees.blackboard.Client(name='AllStationsComplete')
        self.bb.register_key(BB_DELIVERED_SET, access=py_trees.common.Access.READ)

    def update(self):
        delivered = self.bb.get(BB_DELIVERED_SET)
        if len(delivered) >= TOTAL_STATIONS:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


# ═══════════════════════════════════════════════════════════
# ACTION: AnnounceMissionDone
# ═══════════════════════════════════════════════════════════
class AnnounceMissionDone(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('AnnounceMissionDone')

    def update(self):
        _node_ref.get_logger().info(
            '*** ALL STATIONS DELIVERED — MISSION COMPLETE ***')
        # Stop robot
        _node_ref.cmd_vel_pub.publish(Twist())
        return py_trees.common.Status.SUCCESS


# ═══════════════════════════════════════════════════════════
# CONDITION: MarkerDetected
# ═══════════════════════════════════════════════════════════
class MarkerDetected(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('MarkerDetected?')
        self.bb = py_trees.blackboard.Client(name='MarkerDetected')
        self.bb.register_key(BB_ACTIVE_MARKER, access=py_trees.common.Access.READ)
        self.bb.register_key(BB_DELIVERED_SET, access=py_trees.common.Access.READ)

    def update(self):
        marker = self.bb.get(BB_ACTIVE_MARKER)
        if marker is None:
            return py_trees.common.Status.FAILURE

        # Skip if already delivered to this tag
        tag_id = marker.get('tag_id')
        delivered = self.bb.get(BB_DELIVERED_SET)
        if tag_id in delivered:
            return py_trees.common.Status.FAILURE

        _node_ref.get_logger().info(
            f'Marker detected: tag_id={tag_id}, interrupting exploration')
        return py_trees.common.Status.SUCCESS


# ═══════════════════════════════════════════════════════════
# ACTION: CancelCurrentNavGoal
# ═══════════════════════════════════════════════════════════
class CancelCurrentNavGoal(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('CancelCurrentNavGoal')

    def update(self):
        if _node_ref.active_goal_handle is not None:
            _node_ref.get_logger().info('Cancelling active Nav2 goal')
            _node_ref.active_goal_handle.cancel_goal_async()
            _node_ref.active_goal_handle = None
        # Stop robot movement
        _node_ref.cmd_vel_pub.publish(Twist())
        return py_trees.common.Status.SUCCESS


# ═══════════════════════════════════════════════════════════
# ACTION: IdentifyStation
# ═══════════════════════════════════════════════════════════
class IdentifyStation(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('IdentifyStation')
        self.bb = py_trees.blackboard.Client(name='IdentifyStation')
        self.bb.register_key(BB_ACTIVE_MARKER, access=py_trees.common.Access.READ)
        self.bb.register_key(BB_STATION_TYPE, access=py_trees.common.Access.WRITE)

    def update(self):
        marker = self.bb.get(BB_ACTIVE_MARKER)
        if marker is None:
            return py_trees.common.Status.FAILURE

        tag_id = marker.get('tag_id')
        station = STATION_MAP.get(tag_id, None)
        if station is None:
            _node_ref.get_logger().warn(
                f'Unknown tag_id {tag_id}, not in station map')
            return py_trees.common.Status.FAILURE

        self.bb.set(BB_STATION_TYPE, station)
        _node_ref.get_logger().info(
            f'Identified station {station} from tag {tag_id}')
        return py_trees.common.Status.SUCCESS


# ═══════════════════════════════════════════════════════════
# CONDITION: IsStation{A,B,C}
# ═══════════════════════════════════════════════════════════
class IsStationType(py_trees.behaviour.Behaviour):
    def __init__(self, station_type):
        super().__init__(f'IsStation{station_type}?')
        self._station_type = station_type
        self.bb = py_trees.blackboard.Client(name=f'IsStation{station_type}')
        self.bb.register_key(BB_STATION_TYPE, access=py_trees.common.Access.READ)

    def update(self):
        if self.bb.get(BB_STATION_TYPE) == self._station_type:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


# ═══════════════════════════════════════════════════════════
# ACTION: NavigateToStation
# Navigate to the marker pose with an approach offset
# ═══════════════════════════════════════════════════════════
class NavigateToStation(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('NavigateToStation')
        self.bb = py_trees.blackboard.Client(name='NavigateToStation')
        self.bb.register_key(BB_ACTIVE_MARKER, access=py_trees.common.Access.READ)
        self._goal_handle = None
        self._result = None
        self._sent_goal = False

    def initialise(self):
        self._goal_handle = None
        self._result = None
        self._sent_goal = False

    def update(self):
        if not self._sent_goal:
            marker = self.bb.get(BB_ACTIVE_MARKER)
            if marker is None:
                return py_trees.common.Status.FAILURE

            map_pose = marker.get('map_pose')
            if map_pose is None:
                # Fall back to navigating toward the camera direction
                _node_ref.get_logger().warn(
                    'No map_pose for marker, using robot forward approach')
                return py_trees.common.Status.SUCCESS

            # Compute approach point: offset back from marker along
            # robot→marker direction
            try:
                robot_trans = _node_ref.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time())
                rx = robot_trans.transform.translation.x
                ry = robot_trans.transform.translation.y
            except Exception:
                rx, ry = 0.0, 0.0

            mx, my = map_pose['x'], map_pose['y']
            dx = mx - rx
            dy = my - ry
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > APPROACH_OFFSET:
                # Approach point: APPROACH_OFFSET metres before marker
                scale = (dist - APPROACH_OFFSET) / dist
                ax = rx + dx * scale
                ay = ry + dy * scale
            else:
                ax, ay = rx, ry  # already close enough

            # Compute yaw facing the marker
            yaw = math.atan2(my - ay, mx - ax)

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = _node_ref.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = ax
            goal_msg.pose.pose.position.y = ay
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

            if not _node_ref.nav_client.wait_for_server(timeout_sec=2.0):
                _node_ref.get_logger().warn('Nav2 not available')
                return py_trees.common.Status.FAILURE

            future = _node_ref.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self._goal_response_cb)
            self._sent_goal = True
            _node_ref.get_logger().info(
                f'Navigating to station at ({ax:.2f}, {ay:.2f})')
            return py_trees.common.Status.RUNNING

        if self._result is None:
            return py_trees.common.Status.RUNNING

        if self._result == 'success':
            _node_ref.get_logger().info('Arrived at station')
            return py_trees.common.Status.SUCCESS
        else:
            _node_ref.get_logger().warn('Failed to reach station')
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self._goal_handle is not None and self._result is None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._result = 'failed'
            return
        self._goal_handle = goal_handle
        _node_ref.active_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        self._result = 'success' if status == 4 else 'failed'


# ═══════════════════════════════════════════════════════════
# ACTION: AlignWithMarker
# Visual servo: centre AprilTag in camera image by rotating
# ═══════════════════════════════════════════════════════════
class AlignWithMarker(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('AlignWithMarker')
        self._start_time = None
        self._timeout = 10.0  # seconds
        self._aligned = False

    def initialise(self):
        self._start_time = _node_ref.get_clock().now()
        self._aligned = False

    def update(self):
        elapsed = (
            _node_ref.get_clock().now() - self._start_time
        ).nanoseconds / 1e9

        if elapsed > self._timeout:
            _node_ref.cmd_vel_pub.publish(Twist())
            _node_ref.get_logger().warn('Alignment timeout')
            return py_trees.common.Status.SUCCESS  # proceed anyway

        # Use latest marker detection to align
        marker = _node_ref.latest_marker_data
        if marker is None:
            # No current detection — slowly rotate to find it
            twist = Twist()
            twist.angular.z = 0.3
            _node_ref.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING

        camera_pose = marker.get('camera_pose', {})
        x_offset = camera_pose.get('x', 0.0)
        distance = marker.get('distance', 999.0)

        # Proportional control to centre tag in image
        angular_error = -x_offset  # negative because camera x is right
        twist = Twist()

        if abs(angular_error) < 0.02 and distance < 0.5:
            # Aligned and close enough
            _node_ref.cmd_vel_pub.publish(Twist())
            _node_ref.get_logger().info('Aligned with marker')
            return py_trees.common.Status.SUCCESS

        # Rotate to centre
        kp_angular = 1.0
        twist.angular.z = kp_angular * angular_error
        twist.angular.z = max(-0.5, min(0.5, twist.angular.z))

        # Creep forward if not close enough
        if distance > 0.3:
            twist.linear.x = 0.05

        _node_ref.cmd_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        _node_ref.cmd_vel_pub.publish(Twist())


# ═══════════════════════════════════════════════════════════
# ACTION: DeliverBallsStatic (Station A)
# Sends timed ball drop commands
# ═══════════════════════════════════════════════════════════
class DeliverBallsStatic(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('DeliverBallsStatic')
        self._sent = False

    def initialise(self):
        self._sent = False

    def update(self):
        if not self._sent:
            # Send delivery command for 3 balls
            msg = Int32()
            msg.data = BALLS_PER_STATION
            _node_ref.delivery_cmd_pub.publish(msg)
            self._sent = True
            _node_ref.get_logger().info(
                f'Sent delivery command: {BALLS_PER_STATION} balls (static)')
            return py_trees.common.Status.RUNNING

        # Check delivery status
        status = _node_ref.latest_delivery_status
        if status is None:
            return py_trees.common.Status.RUNNING

        s = status.get('status', '')
        if s in ('complete', 'partial'):
            _node_ref.get_logger().info(f'Static delivery done: {s}')
            _node_ref.latest_delivery_status = None
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


# ═══════════════════════════════════════════════════════════
# ACTION: TrackAndAlignMoving (Station B — moving rail)
# Continuous visual servo to track moving marker
# ═══════════════════════════════════════════════════════════
class TrackAndAlignMoving(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('TrackAndAlignMoving')
        self._start_time = None
        self._timeout = 30.0  # seconds
        self._aligned_count = 0
        self._required_aligned = 5  # consecutive aligned ticks

    def initialise(self):
        self._start_time = _node_ref.get_clock().now()
        self._aligned_count = 0

    def update(self):
        elapsed = (
            _node_ref.get_clock().now() - self._start_time
        ).nanoseconds / 1e9

        if elapsed > self._timeout:
            _node_ref.cmd_vel_pub.publish(Twist())
            _node_ref.get_logger().warn('Moving track timeout')
            return py_trees.common.Status.SUCCESS

        marker = _node_ref.latest_marker_data
        if marker is None:
            self._aligned_count = 0
            twist = Twist()
            twist.angular.z = 0.3
            _node_ref.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING

        camera_pose = marker.get('camera_pose', {})
        x_offset = camera_pose.get('x', 0.0)

        twist = Twist()
        angular_error = -x_offset
        kp = 1.5  # higher gain for tracking

        if abs(angular_error) < 0.03:
            self._aligned_count += 1
        else:
            self._aligned_count = 0

        twist.angular.z = max(-0.8, min(0.8, kp * angular_error))
        _node_ref.cmd_vel_pub.publish(twist)

        if self._aligned_count >= self._required_aligned:
            _node_ref.cmd_vel_pub.publish(Twist())
            _node_ref.get_logger().info('Aligned with moving marker')
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        _node_ref.cmd_vel_pub.publish(Twist())


# ═══════════════════════════════════════════════════════════
# ACTION: DeliverBallsDynamic (Station B)
# Drop balls one at a time while tracking
# ═══════════════════════════════════════════════════════════
class DeliverBallsDynamic(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('DeliverBallsDynamic')
        self._ball_idx = 0
        self._sent_current = False
        self._wait_start = None

    def initialise(self):
        self._ball_idx = 0
        self._sent_current = False
        self._wait_start = None

    def update(self):
        if self._ball_idx >= BALLS_PER_STATION:
            _node_ref.get_logger().info('Dynamic delivery complete')
            return py_trees.common.Status.SUCCESS

        # Track marker while delivering
        marker = _node_ref.latest_marker_data
        if marker is not None:
            camera_pose = marker.get('camera_pose', {})
            x_offset = camera_pose.get('x', 0.0)
            twist = Twist()
            twist.angular.z = max(-0.5, min(0.5, -1.5 * x_offset))
            _node_ref.cmd_vel_pub.publish(twist)

        if not self._sent_current:
            msg = Int32()
            msg.data = self._ball_idx + 1
            _node_ref.delivery_single_pub.publish(msg)
            self._sent_current = True
            self._wait_start = _node_ref.get_clock().now()
            _node_ref.get_logger().info(
                f'Dropping ball {self._ball_idx + 1} (dynamic)')
            return py_trees.common.Status.RUNNING

        # Wait for delivery + settle time
        elapsed = (
            _node_ref.get_clock().now() - self._wait_start
        ).nanoseconds / 1e9
        if elapsed > 3.0:  # 3 seconds per ball
            self._ball_idx += 1
            self._sent_current = False
            self._wait_start = None

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        _node_ref.cmd_vel_pub.publish(Twist())


# ═══════════════════════════════════════════════════════════
# ACTION: CallLiftAPI (Station C — bonus)
# ═══════════════════════════════════════════════════════════
class CallLiftAPI(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('CallLiftAPI')
        self._state = 'IDLE'
        self._start_time = None
        self._timeout = 60.0

    def initialise(self):
        self._state = 'SUMMON'
        self._start_time = _node_ref.get_clock().now()

    def update(self):
        if not HAS_REQUESTS:
            _node_ref.get_logger().warn(
                'requests library not available, skipping lift API')
            return py_trees.common.Status.SUCCESS

        elapsed = (
            _node_ref.get_clock().now() - self._start_time
        ).nanoseconds / 1e9
        if elapsed > self._timeout:
            _node_ref.get_logger().warn('Lift API timeout')
            return py_trees.common.Status.FAILURE

        lift_url = _node_ref.lift_api_url

        if self._state == 'SUMMON':
            try:
                resp = requests.post(
                    f'{lift_url}/summon', json={'floor': 1}, timeout=5.0)
                if resp.ok:
                    _node_ref.get_logger().info('Lift summoned')
                    self._state = 'WAIT_ARRIVAL'
            except Exception as e:
                _node_ref.get_logger().warn(f'Lift summon failed: {e}')
            return py_trees.common.Status.RUNNING

        elif self._state == 'WAIT_ARRIVAL':
            try:
                resp = requests.get(f'{lift_url}/status', timeout=5.0)
                if resp.ok:
                    data = resp.json()
                    if data.get('arrived', False):
                        _node_ref.get_logger().info('Lift arrived')
                        self._state = 'ENTER'
            except Exception:
                pass
            return py_trees.common.Status.RUNNING

        elif self._state == 'ENTER':
            # Drive forward into the lift
            twist = Twist()
            twist.linear.x = 0.1
            _node_ref.cmd_vel_pub.publish(twist)
            time.sleep(3.0)
            _node_ref.cmd_vel_pub.publish(Twist())

            try:
                requests.post(
                    f'{lift_url}/go', json={'floor': 2}, timeout=5.0)
            except Exception:
                pass
            self._state = 'RIDE'
            return py_trees.common.Status.RUNNING

        elif self._state == 'RIDE':
            try:
                resp = requests.get(f'{lift_url}/status', timeout=5.0)
                if resp.ok:
                    data = resp.json()
                    if data.get('floor') == 2 and data.get('door_open', False):
                        _node_ref.get_logger().info('Arrived at floor 2')
                        # Drive out of lift
                        twist = Twist()
                        twist.linear.x = 0.1
                        _node_ref.cmd_vel_pub.publish(twist)
                        time.sleep(3.0)
                        _node_ref.cmd_vel_pub.publish(Twist())
                        return py_trees.common.Status.SUCCESS
            except Exception:
                pass
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        _node_ref.cmd_vel_pub.publish(Twist())


# ═══════════════════════════════════════════════════════════
# ACTION: MarkStationDone
# ═══════════════════════════════════════════════════════════
class MarkStationDone(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('MarkStationDone')
        self.bb = py_trees.blackboard.Client(name='MarkStationDone')
        self.bb.register_key(BB_ACTIVE_MARKER, access=py_trees.common.Access.WRITE)
        self.bb.register_key(BB_DELIVERED_SET, access=py_trees.common.Access.WRITE)
        self.bb.register_key(BB_STATION_TYPE, access=py_trees.common.Access.READ)

    def update(self):
        marker = _node_ref.latest_marker_data
        if marker is not None:
            tag_id = marker.get('tag_id')
            delivered = self.bb.get(BB_DELIVERED_SET)
            delivered.add(tag_id)
            self.bb.set(BB_DELIVERED_SET, delivered)
            station = self.bb.get(BB_STATION_TYPE)
            _node_ref.get_logger().info(
                f'Station {station} (tag {tag_id}) marked DONE. '
                f'Delivered: {delivered}')

        # Clear active marker so exploration resumes
        self.bb.set(BB_ACTIVE_MARKER, None)
        _node_ref.latest_marker_data = None
        return py_trees.common.Status.SUCCESS


# ═══════════════════════════════════════════════════════════
# REUSED FROM exploration_manager.py
# ═══════════════════════════════════════════════════════════

class IsMapClosed(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('IsMapClosed?')
        self.bb = py_trees.blackboard.Client(name='IsMapClosed')
        self.bb.register_key(BB_MAP_CLOSED, access=py_trees.common.Access.READ)

    def update(self):
        if self.bb.get(BB_MAP_CLOSED):
            _node_ref.get_logger().info('Map is closed')
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


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
            f'Frontier: ({frontier.pose.position.x:.2f}, '
            f'{frontier.pose.position.y:.2f})')
        return py_trees.common.Status.SUCCESS


class NavigateToFrontier(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('NavigateToFrontier')
        self.bb = py_trees.blackboard.Client(name='NavigateToFrontier')
        self.bb.register_key(
            BB_CURRENT_FRONTIER, access=py_trees.common.Access.READ)
        self._goal_handle = None
        self._result = None
        self._sent_goal = False

    def initialise(self):
        self._goal_handle = None
        self._result = None
        self._sent_goal = False

    def update(self):
        if not self._sent_goal:
            frontier = self.bb.get(BB_CURRENT_FRONTIER)
            if frontier is None:
                return py_trees.common.Status.FAILURE

            if not _node_ref.nav_client.wait_for_server(timeout_sec=2.0):
                _node_ref.get_logger().warn('Nav2 not available')
                return py_trees.common.Status.FAILURE

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = frontier

            future = _node_ref.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self._goal_response_cb)
            self._sent_goal = True
            _node_ref.get_logger().info(
                f'Nav2 goal: ({frontier.pose.position.x:.2f}, '
                f'{frontier.pose.position.y:.2f})')
            return py_trees.common.Status.RUNNING

        if self._result is None:
            return py_trees.common.Status.RUNNING

        if self._result == 'success':
            frontier = self.bb.get(BB_CURRENT_FRONTIER)
            if frontier:
                _node_ref.blacklist_frontier(
                    frontier.pose.position.x, frontier.pose.position.y)
            return py_trees.common.Status.SUCCESS
        else:
            frontier = self.bb.get(BB_CURRENT_FRONTIER)
            if frontier:
                _node_ref.blacklist_frontier(
                    frontier.pose.position.x, frontier.pose.position.y)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self._goal_handle is not None and self._result is None:
            _node_ref.get_logger().info('Cancelling frontier nav goal')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._result = 'failed'
            return
        self._goal_handle = goal_handle
        _node_ref.active_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        self._result = 'success' if status == 4 else 'failed'


class RecoveryRotation(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__('RecoveryRotation')
        self._start_time = None
        self._duration = 6.0

    def initialise(self):
        self._start_time = _node_ref.get_clock().now()
        _node_ref.get_logger().info('Recovery: spinning 360 deg')

    def update(self):
        twist = Twist()
        twist.angular.z = 1.0
        _node_ref.cmd_vel_pub.publish(twist)

        elapsed = (
            _node_ref.get_clock().now() - self._start_time
        ).nanoseconds / 1e9
        if elapsed >= self._duration:
            _node_ref.cmd_vel_pub.publish(Twist())
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        _node_ref.cmd_vel_pub.publish(Twist())


# ═══════════════════════════════════════════════════════════
# BUILD THE FULL MISSION BT
# ═══════════════════════════════════════════════════════════
def build_mission_tree():
    """
    Fallback: MissionRoot
    |-- Seq: AllDelivered
    |-- Seq: HandleDetectedMarker (interrupt)
    |   |-- MarkerDetected?
    |   |-- CancelCurrentNavGoal
    |   |-- IdentifyStation
    |   |-- Fallback: ExecuteDelivery
    |   |   |-- Seq: StationA
    |   |   |-- Seq: StationB
    |   |   |-- Seq: StationC
    |   |-- MarkStationDone
    |-- Seq: FrontierExploration
    |-- RecoveryRotation
    """

    # ── All delivered check ────────────────────────────
    all_delivered = py_trees.composites.Sequence(
        name='AllDelivered', memory=True)
    all_delivered.add_children([
        AllStationsComplete(),
        AnnounceMissionDone(),
    ])

    # ── Station A delivery ─────────────────────────────
    station_a = py_trees.composites.Sequence(
        name='StationA', memory=True)
    station_a.add_children([
        IsStationType('A'),
        NavigateToStation(),
        AlignWithMarker(),
        DeliverBallsStatic(),
    ])

    # ── Station B delivery ─────────────────────────────
    station_b = py_trees.composites.Sequence(
        name='StationB', memory=True)
    station_b.add_children([
        IsStationType('B'),
        NavigateToStation(),
        TrackAndAlignMoving(),
        DeliverBallsDynamic(),
    ])

    # ── Station C delivery (bonus) ─────────────────────
    station_c = py_trees.composites.Sequence(
        name='StationC', memory=True)
    station_c.add_children([
        IsStationType('C'),
        NavigateToStation(),
        CallLiftAPI(),
        DeliverBallsStatic(),
    ])

    # ── Delivery fallback ──────────────────────────────
    execute_delivery = py_trees.composites.Selector(
        name='ExecuteDelivery', memory=False)
    execute_delivery.add_children([station_a, station_b, station_c])

    # ── Marker interrupt sequence ──────────────────────
    handle_marker = py_trees.composites.Sequence(
        name='HandleDetectedMarker', memory=True)
    handle_marker.add_children([
        MarkerDetected(),
        CancelCurrentNavGoal(),
        IdentifyStation(),
        execute_delivery,
        MarkStationDone(),
    ])

    # ── Frontier exploration ───────────────────────────
    frontier_exploration = py_trees.composites.Sequence(
        name='FrontierExploration', memory=False)
    frontier_exploration.add_children([
        GetBestFrontier(),
        NavigateToFrontier(),
    ])

    # ── Root ───────────────────────────────────────────
    root = py_trees.composites.Selector(
        name='MissionRoot', memory=False)
    root.add_children([
        all_delivered,
        handle_marker,
        frontier_exploration,
        RecoveryRotation(),
    ])

    return root


# ═══════════════════════════════════════════════════════════
# MISSION CONTROLLER NODE
# ═══════════════════════════════════════════════════════════
class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        # ── ROS parameters ──────────────────────────────
        self.declare_parameter('lift_api_url', 'http://localhost:8080/lift')
        self.lift_api_url = self.get_parameter('lift_api_url').value

        # ── Nav2 action client ───────────────────────────
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # ── TF ───────────────────────────────────────────
        import tf2_ros as _tf2
        self.tf_buffer = _tf2.Buffer()
        self.tf_listener = _tf2.TransformListener(self.tf_buffer, self)

        # ── Publishers ───────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.blacklist_pub = self.create_publisher(
            Point, '/blacklist_frontier', 10)
        self.delivery_cmd_pub = self.create_publisher(
            Int32, '/delivery_cmd', 10)
        self.delivery_single_pub = self.create_publisher(
            Int32, '/delivery_drop_single', 10)

        # ── Subscribers ──────────────────────────────────
        self.create_subscription(
            PoseStamped, '/best_frontier', self._frontier_cb, 10)
        self.create_subscription(
            Bool, '/map_closed', self._map_closed_cb, 10)
        self.create_subscription(
            String, '/marker_detection', self._marker_detection_cb, 10)
        self.create_subscription(
            String, '/delivery_status', self._delivery_status_cb, 10)

        # ── State ────────────────────────────────────────
        self.latest_frontier = None
        self.latest_marker_data = None
        self.latest_delivery_status = None
        self.active_goal_handle = None

        # ── Blackboard ───────────────────────────────────
        self.bb = py_trees.blackboard.Client(name='MissionController')
        self.bb.register_key(
            BB_MAP_CLOSED, access=py_trees.common.Access.WRITE)
        self.bb.register_key(
            BB_ACTIVE_MARKER, access=py_trees.common.Access.WRITE)
        self.bb.register_key(
            BB_DELIVERED_SET, access=py_trees.common.Access.WRITE)

        self.bb.set(BB_MAP_CLOSED, False)
        self.bb.set(BB_ACTIVE_MARKER, None)
        self.bb.set(BB_DELIVERED_SET, set())

        # ── Build BT ────────────────────────────────────
        self.tree = py_trees.trees.BehaviourTree(build_mission_tree())
        self.tree.setup(timeout=15)

        # ── Tick timer (2 Hz) ────────────────────────────
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            'Mission controller ready — BT ticking at 2Hz')

    def _frontier_cb(self, msg):
        self.latest_frontier = msg

    def _map_closed_cb(self, msg):
        self.bb.set(BB_MAP_CLOSED, msg.data)

    def _marker_detection_cb(self, msg):
        """Receive marker detection from apriltag_detector."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        tag_id = data.get('tag_id')
        delivered = self.bb.get(BB_DELIVERED_SET)

        # Only interrupt if this tag hasn't been delivered
        if tag_id not in delivered:
            self.bb.set(BB_ACTIVE_MARKER, data)
            self.latest_marker_data = data
            self.get_logger().info(
                f'Marker {tag_id} stored on blackboard')

    def _delivery_status_cb(self, msg):
        try:
            self.latest_delivery_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def blacklist_frontier(self, wx, wy):
        self.get_logger().warn(
            f'Blacklisting frontier ({wx:.2f}, {wy:.2f})')
        msg = Point()
        msg.x = wx
        msg.y = wy
        self.blacklist_pub.publish(msg)
        self.latest_frontier = None

    def _tick(self):
        self.tree.tick()


# ═══════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)

    global _node_ref
    node = MissionController()
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
