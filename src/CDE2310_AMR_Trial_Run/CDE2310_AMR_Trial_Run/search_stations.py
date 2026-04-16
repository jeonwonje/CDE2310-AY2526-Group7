#!/usr/bin/env python3
import rclpy
import json
import math
from collections import deque
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

class SearchServer(Node):
    def __init__(self):
        super().__init__('search_server')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        
        # --- SEARCH CONFIGURATION ---
        # Offsets (delta_x, delta_y) relative to the robot's STARTING position
        #self.relative_search_offsets = [(0.4, 1.6), (2.6, 3.5)] <-- for gazebo
        #self.relative_search_offsets = [(0.3, 0.7), (1.1, 1.7)]
        self.relative_search_offsets = [(-0.75, -0.3), (1.25, 2.7)]
        
        # Internal variables to hold the calculated global coordinates
        self.absolute_search_zones = []
        self.initial_pose_captured = False
        
        # Max distance (meters) to search for a safe pixel if hardcoded is inside a wall
        self.max_safe_search_radius = 0.6 #changed from 1.5 
        
        # Spin configuration
        self.spin_velocity = 0.5  # rad/s
        self.spin_duration = 13.0 # seconds (0.5 rad/s * 13s > 2*pi)

        # --- ROBUSTNESS PARAMETERS ---
        self.max_nav_retries = 3
        self.arrival_tolerance = 0.4 # Meters. Max acceptable distance from goal to trigger spin
        
        # Tracking variables
        self.current_zone_retries = 0
        self.current_safe_x = 0.0
        self.current_safe_y = 0.0
        # ----------------------------

        self.state = 'IDLE'
        self.map_msg = None
        self.search_queue = []
        self.current_goal_handle = None
        
        # ROS 2 Interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        self.command_sub = self.create_subscription(String, '/mission_command', self.command_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().debug("Search Server initialized.")

    def map_callback(self, msg):
        self.map_msg = msg

    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            action = data.get('action')
            
            if action == 'START_SEARCH' and self.state == 'IDLE':
                docked_tags = data.get('docked_tags', []) # Extract the docked list
                self.get_logger().info("Search sequence initiated.")
                self.prepare_search_queue(docked_tags) # Pass it to the queue builder
                
            elif action == 'ABORT_SEARCH':
                self.get_logger().warn("Search ABORTED by Coordinator. Tag likely found!")
                self.cancel_active_search()
        except json.JSONDecodeError:
            pass

    def send_status(self, status):
        msg = {'sender': 'searcher', 'status': status, 'data': None}
        self.status_pub.publish(String(data=json.dumps(msg)))

    def cancel_active_search(self):
        self.state = 'IDLE'
        self.search_queue.clear()
        
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
            
        # Stop spinning if we were spinning
        self.cmd_vel_pub.publish(Twist())

    def prepare_search_queue(self, docked_tags):
        """Prunes zones near already-docked tags, then sorts the rest by proximity to robot."""
        valid_zones = list(self.absolute_search_zones)
        
        # 1. Pruning Phase
        for d_tag in docked_tags:
            try:
                # Where is the tag we already serviced?
                t = self.tf_buffer.lookup_transform('map', d_tag, rclpy.time.Time())
                tx, ty = t.transform.translation.x, t.transform.translation.y
                
                # Find the hardcoded zone closest to this serviced tag
                if valid_zones:
                    closest_zone = min(valid_zones, key=lambda z: math.hypot(z[0] - tx, z[1] - ty))
                    valid_zones.remove(closest_zone)
                    self.get_logger().debug(f"Pruned zone {closest_zone} (Near already docked tag {d_tag}).")
            except TransformException:
                self.get_logger().warn(f"Could not locate docked tag {d_tag} to prune zones. Skipping prune.")

        # 2. Sorting Phase
        try:
            # Where is the robot right now?
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            
            # Since we removed the places we DON'T want to go, we want to go to the 
            # closest REMAINING valid zone to save time. (Notice reverse=False)
            self.search_queue = sorted(
                valid_zones,
                key=lambda zone: math.hypot(zone[0] - rx, zone[1] - ry)
            )
            self.get_logger().debug(f"Final valid search queue generated: {self.search_queue}")
            self.execute_next_zone()
            
        except TransformException:
            self.get_logger().warn("Could not get robot pose. Using default valid zone order.")
            self.search_queue = valid_zones
            self.execute_next_zone()

    def execute_next_zone(self):
        if not self.search_queue:
            self.get_logger().error("All zones searched. Tag not found.")
            self.send_status('SEARCH_FAILED')
            self.state = 'IDLE'
            return

        # Reset retries for the new zone
        self.current_zone_retries = 0
        target_x, target_y = self.search_queue.pop(0)
        
        if self.map_msg is None:
            self.get_logger().warn("No map received yet. Blindly sending hardcoded goal.")
            safe_x, safe_y = target_x, target_y
        else:
            safe_x, safe_y = self.generate_safe_goal(target_x, target_y)

        # Save these so we can retry them if Nav2 fails
        self.current_safe_x = safe_x
        self.current_safe_y = safe_y

        self.get_logger().debug(f"Dispatching Nav2 to safe goal: ({safe_x:.2f}, {safe_y:.2f})")
        self.send_nav2_goal(safe_x, safe_y)

    def generate_safe_goal(self, target_x, target_y):
        """Uses BFS on the OccupancyGrid to pull the goal out of walls/obstacles."""
        res = self.map_msg.info.resolution
        orig_x = self.map_msg.info.origin.position.x
        orig_y = self.map_msg.info.origin.position.y
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        
        # Helper: World to Grid
        def world_to_grid(wx, wy):
            gx = int((wx - orig_x) / res)
            gy = int((wy - orig_y) / res)
            return gx, gy
            
        # Helper: Grid to World
        def grid_to_world(gx, gy):
            wx = orig_x + (gx * res) + (res / 2.0)
            wy = orig_y + (gy * res) + (res / 2.0)
            return wx, wy

        start_gx, start_gy = world_to_grid(target_x, target_y)
        
        # Boundary check
        if not (0 <= start_gx < width and 0 <= start_gy < height):
            return target_x, target_y # Out of bounds, return original

        data = self.map_msg.data
        start_idx = start_gy * width + start_gx
        
        # If the cell is free (0 to 49), it's already safe
        if 0 <= data[start_idx] < 50:
            return target_x, target_y

        self.get_logger().debug("Hardcoded goal is inside an obstacle. Finding nearest safe pixel...")
        
        # BFS Setup
        queue = deque([(start_gx, start_gy)])
        visited = set([(start_gx, start_gy)])
        max_cells = int(self.max_safe_search_radius / res)
        
        while queue:
            cx, cy = queue.popleft()
            
            # Check if this cell is free
            idx = cy * width + cx
            if 0 <= data[idx] < 50:
                # Found a safe cell!
                new_wx, new_wy = grid_to_world(cx, cy)
                self.get_logger().debug(f"Found safe cell at grid ({cx},{cy}). Shifted goal.")
                return new_wx, new_wy
                
            # Expand neighbors (8-way)
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        if (nx, ny) not in visited:
                            # Distance limit check
                            if math.hypot(nx - start_gx, ny - start_gy) <= max_cells:
                                visited.add((nx, ny))
                                queue.append((nx, ny))
                                
        self.get_logger().warn("Could not find safe space within radius. Risking original goal.")
        return target_x, target_y

    def send_nav2_goal(self, x, y):
        self.nav_client.wait_for_server()
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0 # Orientation doesn't matter for search
        
        goal_msg = NavigateToPose.Goal(pose=goal)
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)
        self.state = 'NAVIGATING'

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Search goal rejected by Nav2.")
            self.execute_next_zone() # Try the next zone instead of failing
            return
            
        self.current_goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        self.current_goal_handle = None
        
        if self.state != 'NAVIGATING': 
            return # Ensure we weren't aborted by the Coordinator
            
        try:
            # 1. Verify actual position
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            
            dist_to_goal = math.hypot(self.current_safe_x - rx, self.current_safe_y - ry)
            
            # 2. Evaluate Success based on physics, not Nav2's status code
            if dist_to_goal <= self.arrival_tolerance:
                self.get_logger().info(f"Arrived at Search Zone (Error: {dist_to_goal:.2f}m). Executing 360 Sweep.")
                self.spin_start_time = self.get_clock().now()
                self.state = 'SPINNING'
            else:
                self.get_logger().warn(f"Nav2 finished, but robot is {dist_to_goal:.2f}m away from target.")
                
                # 3. Retry Logic
                self.current_zone_retries += 1
                if self.current_zone_retries < self.max_nav_retries:
                    self.get_logger().debug(f"Retrying current zone (Attempt {self.current_zone_retries + 1}/{self.max_nav_retries})...")
                    self.send_nav2_goal(self.current_safe_x, self.current_safe_y)
                else:
                    self.get_logger().error("Max retries reached. Zone unreachable. Moving to next target.")
                    self.execute_next_zone()
                    
        except TransformException as e:
            self.get_logger().error(f"TF Error checking arrival distance: {e}. Skipping to next zone.")
            self.execute_next_zone()

    def tick(self):
        # 1. The Startup Snapshot
        # Capture the robot's exact starting position on the map before it explores
        if not self.initial_pose_captured:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                start_x = t.transform.translation.x
                start_y = t.transform.translation.y
                
                # Calculate the absolute map coordinates based on your relative offsets
                for dx, dy in self.relative_search_offsets:
                    # Depending on your starting orientation, you may need to apply a 
                    # rotation matrix here. But assuming the robot always starts facing 
                    # "forward" along the map's X-axis:
                    self.absolute_search_zones.append((start_x + dx, start_y + dy))
                    
                self.initial_pose_captured = True
                self.get_logger().debug(f"Startup Pose Captured. Absolute search zones locked: {self.absolute_search_zones}")
            except TransformException:
                pass # The TF tree takes a few seconds to build on startup. Keep trying.
                return

        # 2. Existing Spin Logic
        if self.state == 'SPINNING':
            cmd = Twist()
            cmd.angular.z = self.spin_velocity
            self.cmd_vel_pub.publish(cmd)
            
            elapsed = (self.get_clock().now() - self.spin_start_time).nanoseconds / 1e9
            if elapsed > self.spin_duration:
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("Sweep complete. Tag not seen.")
                self.state = 'IDLE'
                self.execute_next_zone()

def main(args=None):
    rclpy.init(args=args)
    node = SearchServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()