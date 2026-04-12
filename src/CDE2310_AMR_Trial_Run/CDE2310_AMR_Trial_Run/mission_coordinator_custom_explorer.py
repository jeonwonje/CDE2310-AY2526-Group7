#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from action_msgs.msg import GoalStatus
import tf2_geometry_msgs

from std_srvs.srv import SetBool

class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')
        
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # --- CONFIGURATION PARAMETERS ---
        # Change these to adjust docking behavior
        self.staging_x_offset = 0.5    # Distance (m) in front of tag for Nav2 staging
        self.refinement_cutoff = 0.7  # Distance (m) to tag where Nav2 stops updating goal
        self.stop_distance = 0.3       # Final distance (m) to stop during visual servoing
        
        self.k_linear = 0.5            # Servoing linear gain
        self.k_angular = 2.0           # Servoing angular gain
        self.refinement_interval = 1.5 # How often (s) to update Nav2 goal
        # -------------------------------

        # State Machine
        self.state = 'EXPLORING'
        self.docked_tags = set()  
        self.kill_time = None 

        self.target_tags = ['tag36h11:0', 'tag36h11:2']
        self.current_target_tag = None
        self.current_nav2_frame = None
        
        # Refinement Trackers
        self.last_goal_update_time = self.get_clock().now()
        self.current_goal_handle = None
        
        # ROS 2 Interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.toggle_explore_client = self.create_client(SetBool, 'toggle_exploration')
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Control Timers
        self.main_loop_timer = self.create_timer(0.1, self.tick)

        while not self.toggle_explore_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /toggle_exploration service...')

        self.start_exploration()

    def start_exploration(self):
        self.get_logger().info("Starting/Resuming exploration...")
        req = SetBool.Request()
        req.data = True
        self.toggle_explore_client.call_async(req)
        self.state = 'EXPLORING'

    def tick(self):
        if self.state == 'EXPLORING':
            self.check_for_apriltag()
        elif self.state == 'KILLING_EXPLORATION':
            self.handle_kill_sequence()
        elif self.state == 'NAV_TO_STAGING':
            self.refine_staging_pose()
        elif self.state == 'VISUAL_SERVOING':
            self.execute_visual_servoing()

    def check_for_apriltag(self):
        for tag in self.target_tags:
            if tag in self.docked_tags:
                continue # Skip tags we have already serviced
                
            try:
                t = self.tf_buffer.lookup_transform('map', tag, rclpy.time.Time())
                
                # Lock onto this tag
                self.current_target_tag = tag
                tag_id = tag.split(':')[-1] # Extracts '0' or '2'
                self.current_nav2_frame = f'nav2_dock_target_{tag_id}'
                
                self.get_logger().info(f"Target {tag} Detected. Pausing exploration.")
                
                req = SetBool.Request()
                req.data = False
                self.toggle_explore_client.call_async(req)
                
                self.kill_time = self.get_clock().now()
                self.state = 'KILLING_EXPLORATION'
                return # Exit the function so we only process one tag at a time
                
            except TransformException:
                pass

    def handle_kill_sequence(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        elapsed = (self.get_clock().now() - self.kill_time).nanoseconds / 1e9
        
        if elapsed > 2.5:
            self.state = 'NAV_TO_STAGING'
            self.last_goal_update_time = self.get_clock().now() - rclpy.duration.Duration(seconds=5.0)

    def refine_staging_pose(self):
        now = self.get_clock().now()
        if (now - self.last_goal_update_time).nanoseconds / 1e9 < self.refinement_interval:
            return

        try:
            t_base = self.tf_buffer.lookup_transform('base_link', self.current_target_tag, rclpy.time.Time())
            dist_to_tag = math.sqrt(t_base.transform.translation.x**2 + t_base.transform.translation.y**2)
            
            # Use refinement_cutoff parameter
            if dist_to_tag < self.refinement_cutoff and self.current_goal_handle is not None:
                return

            if self.tf_buffer.can_transform('map', self.current_nav2_frame, rclpy.time.Time()):
                staging_pose = PoseStamped()
                staging_pose.header.frame_id = self.current_nav2_frame
                staging_pose.header.stamp = rclpy.time.Time().to_msg() 
                
                # Use staging_x_offset parameter
                staging_pose.pose.position.x = self.staging_x_offset 
                staging_pose.pose.orientation.z = 1.0 
                staging_pose.pose.orientation.w = 0.0 

                global_staging_pose = self.tf_buffer.transform(staging_pose, 'map')
                global_staging_pose.pose.position.z = 0.0
                q = global_staging_pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                
                global_staging_pose.pose.orientation.z = math.sin(yaw / 2.0)
                global_staging_pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                self.nav_client.wait_for_server()
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = global_staging_pose
                
                self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
                self.send_goal_future.add_done_callback(self.staging_goal_response_callback)
                self.last_goal_update_time = now
        except TransformException:
            pass

    def staging_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self.current_goal_handle is None:
                self.start_exploration()
            return
        self.current_goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(
            lambda future, gh=goal_handle: self.staging_result_callback(future, gh)
        )

    def staging_result_callback(self, future, goal_handle):
        if self.current_goal_handle and goal_handle != self.current_goal_handle:
            return
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.current_goal_handle = None
            self.state = 'VISUAL_SERVOING'
        else:
            self.current_goal_handle = None
            self.start_exploration()

    def execute_visual_servoing(self):
        msg = Twist()
        try:
            t = self.tf_buffer.lookup_transform('base_link', self.current_target_tag, rclpy.time.Time())
            distance_x = t.transform.translation.x
            offset_y = t.transform.translation.y
            
            msg.angular.z = max(min(self.k_angular * offset_y, 0.4), -0.4) 
            
            # Use stop_distance parameter
            error_distance = distance_x - self.stop_distance
            
            if error_distance > 0.05:
                msg.linear.x = max(min(self.k_linear * error_distance, 0.15), 0.0) 
                self.cmd_vel_pub.publish(msg)
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                self.get_logger().info("Docked!")
                self.docked_tags.add(self.current_target_tag)
                self.state = 'DOCKING_COMPLETE'
                self.post_docking_routine()
        except TransformException:
            self.cmd_vel_pub.publish(Twist()) 

    def post_docking_routine(self):
        self.get_logger().info(f"Test docking complete for {self.current_target_tag}. Simulating 3-second wait...")
        time.sleep(3.0) 
        msg = Twist()
        msg.linear.x = -0.2
        self.cmd_vel_pub.publish(msg)
        time.sleep(1.0)
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)
        self.start_exploration()

def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
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
