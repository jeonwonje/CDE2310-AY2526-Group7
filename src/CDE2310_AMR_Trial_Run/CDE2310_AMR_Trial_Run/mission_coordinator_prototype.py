#!/usr/bin/env python3
import rclpy
import os
import signal
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
import subprocess
import time
import math

from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from action_msgs.msg import GoalStatus
from tf2_geometry_msgs import do_transform_pose

class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')
        
        # Force the node to use Gazebo's clock
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # State Machine
        self.state = 'EXPLORING'
        self.docked_tags = set()  
        self.kill_time = None 
        
        # Refinement Trackers
        self.last_goal_update_time = self.get_clock().now()
        self.current_goal_handle = None
        
        # ROS 2 Interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Control Timers
        self.main_loop_timer = self.create_timer(0.1, self.tick)
        
        self.explore_process = None
        
        # Tuning Constants
        self.k_linear = 0.5
        self.k_angular = 2.0
        self.stop_distance = 0.2

        self.start_exploration()

    def start_exploration(self):
        self.get_logger().info("Starting explore_lite with custom params...")
        self.explore_process = subprocess.Popen([
            'ros2', 'run', 'explore_lite', 'explore',
            '--ros-args', 
            '--params-file', '/home/shashwat/colcon_ws/src/m-explore-ros2/explore/config/params_costmap.yaml',
            '-p', 'use_sim_time:=True'
        ], preexec_fn=os.setsid) 
        self.state = 'EXPLORING'

    def tick(self):
        """Main state machine loop running at 10Hz"""
        if self.state == 'EXPLORING':
            self.check_for_apriltag()
            
        elif self.state == 'KILLING_EXPLORATION':
            self.handle_kill_sequence()
            
        elif self.state == 'NAV_TO_STAGING':
            self.refine_staging_pose()
            
        elif self.state == 'VISUAL_SERVOING':
            self.execute_visual_servoing()

    def check_for_apriltag(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'tag36h11:0', rclpy.time.Time())
            
            if 'tag36h11:0' in self.docked_tags:
                return 
                
            self.get_logger().info("AprilTag Detected! Intercepting mission...")
            
            if self.explore_process:
                os.killpg(os.getpgid(self.explore_process.pid), signal.SIGTERM)
            
            self.kill_time = self.get_clock().now()
            self.state = 'KILLING_EXPLORATION'
            
        except TransformException:
            pass 

    def handle_kill_sequence(self):
        """Buffer state to wait for explore_lite to die and Nav2 to settle"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        
        elapsed = (self.get_clock().now() - self.kill_time).nanoseconds / 1e9
        
        if elapsed > 2.5:
            self.get_logger().info("Exploration fully terminated. Nav2 is now idle.")
            
            if self.explore_process:
                self.explore_process.poll() 
                self.explore_process = None
                
            self.state = 'NAV_TO_STAGING'
            # Reset the timer so the first refinement triggers instantly
            self.last_goal_update_time = self.get_clock().now() - rclpy.duration.Duration(seconds=5.0)

    def refine_staging_pose(self):
        now = self.get_clock().now()
        
        # Only recalculate every 1.5 seconds to prevent path-planner stuttering
        if (now - self.last_goal_update_time).nanoseconds / 1e9 < 1.5:
            return

        try:
            # 1. Check distance. If we are within 2.5m of the tag, stop refining!
            # Let Nav2 finish the final approach without moving the goalpost.
            t_base = self.tf_buffer.lookup_transform('base_link', 'tag36h11:0', rclpy.time.Time())
            dist_to_tag = math.sqrt(t_base.transform.translation.x**2 + t_base.transform.translation.y**2)
            
            if dist_to_tag < 2.5 and self.current_goal_handle is not None:
                return

            # 2. If tag is visible and we are far, refine the goal
            if self.tf_buffer.can_transform('map', 'nav2_dock_target', rclpy.time.Time()):
                
                staging_pose = PoseStamped()
                staging_pose.header.frame_id = 'nav2_dock_target'
                staging_pose.header.stamp = rclpy.time.Time().to_msg() 
                staging_pose.pose.position.x = 1.2 
                staging_pose.pose.orientation.z = 1.0 
                staging_pose.pose.orientation.w = 0.0 

                global_staging_pose = self.tf_buffer.transform(staging_pose, 'map')
                
                global_staging_pose.pose.position.z = 0.0
                q = global_staging_pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                
                global_staging_pose.pose.orientation.x = 0.0
                global_staging_pose.pose.orientation.y = 0.0
                global_staging_pose.pose.orientation.z = math.sin(yaw / 2.0)
                global_staging_pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                self.get_logger().info(f"Refining Goal -> X: {global_staging_pose.pose.position.x:.2f}, Y: {global_staging_pose.pose.position.y:.2f}")
                
                self.nav_client.wait_for_server()
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = global_staging_pose
                
                self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
                self.send_goal_future.add_done_callback(self.staging_goal_response_callback)
                
                self.last_goal_update_time = now
                
        except TransformException:
            pass # Tag momentarily lost, keep driving to the last known goal

    def staging_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected staging goal.")
            # Only abort if we have NO active goals
            if self.current_goal_handle is None:
                self.start_exploration()
            return

        # Track the active goal handle so we know which callbacks to trust
        self.current_goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        
        # Pass the goal_handle into the callback using a lambda
        self.get_result_future.add_done_callback(
            lambda future, gh=goal_handle: self.staging_result_callback(future, gh)
        )

    def staging_result_callback(self, future, goal_handle):
        # IGNORING PREEMPTION: If this callback is from an old goal we replaced, ignore it!
        if self.current_goal_handle and goal_handle != self.current_goal_handle:
            return

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached staging area perfectly. Switching to Visual Servoing.")
            self.current_goal_handle = None
            self.state = 'VISUAL_SERVOING'
        else:
            self.get_logger().warn(f"Nav2 Staging Goal Aborted/Canceled (Status {status}). Retrying exploration.")
            self.current_goal_handle = None
            self.start_exploration()

    def execute_visual_servoing(self):
        msg = Twist()
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'tag36h11:0', rclpy.time.Time())
            
            distance_x = t.transform.translation.x
            offset_y = t.transform.translation.y
            
            raw_angular_cmd = self.k_angular * offset_y
            msg.angular.z = max(min(raw_angular_cmd, 0.4), -0.4) 
            
            error_distance = distance_x - self.stop_distance
            
            if error_distance > 0.05:
                raw_linear_cmd = self.k_linear * error_distance
                msg.linear.x = max(min(raw_linear_cmd, 0.15), 0.0) 
                self.cmd_vel_pub.publish(msg)
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                
                self.get_logger().info("Successfully Docked!")
                self.docked_tags.add('tag36h11:0')
                self.state = 'DOCKING_COMPLETE'
                self.post_docking_routine()

        except TransformException:
            self.cmd_vel_pub.publish(Twist()) 

    def post_docking_routine(self):
        time.sleep(3.0) 
        msg = Twist()
        msg.linear.x = -0.2
        self.cmd_vel_pub.publish(msg)
        time.sleep(2.0)
        
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)
        
        self.get_logger().info("Resuming Exploration...")
        self.start_exploration()

def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected...')
    finally:
        if node.explore_process:
            node.get_logger().info('Cleaning up background processes...')
            try:
                os.killpg(os.getpgid(node.explore_process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass 
                
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
