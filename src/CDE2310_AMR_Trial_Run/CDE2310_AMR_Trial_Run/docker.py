#!/usr/bin/env python3
import rclpy
import json
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
import tf2_geometry_msgs  

def euler_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class DockingServer(Node):
    """
        DockingServer: Discrete Geometric Visual Servoing Node

        --- GEOMETRIC DOCKING PARAMETERS TUNING GUIDE ---
        This node abandons continuous PID curves in favor of a discrete state machine. 
        Tuning is now primarily about geometric boundaries, not mathematical gains.

        1. The Geometry Limits (The "Runway")
        - intercept_ratio: (e.g., 0.6) The dynamic lookahead. If staging is 0.75m, the robot 
          aims to hit the centerline at 0.45m. Increase (0.7) for a sharper, earlier intercept. 
          Decrease (0.4) for a shallower, later intercept.
        - abort_ratio: (e.g., 0.3) The hard safety boundary. If the robot closes within 30% 
          of the staging distance and still hasn't cleared the Y-error, it aborts the slant 
          to prevent crashing into the tag.

        2. State Transition Tolerances
        - intercept_y_tolerance: (m) How close to the centerline the robot must be to consider 
          State 1 (INTERCEPT) complete. Keep this tight (0.02).
        - square_yaw_tolerance: (rad) How perfectly parallel the nose must be to the tag to 
          consider State 2 (SQUARE_UP) complete. 0.05 rad is ~3 degrees.
        - max_allowed_y_error: (m) The final gatekeeper evaluated in State 3. If Y-error is 
          higher than this after squaring up, it triggers a backup instead of plunging.

        3. Speeds and Gains
        - slow_linear_speed: (m/s) Constant forward speed used during INTERCEPT and FINAL_PLUNGE. 
          Keep this low (0.03) to minimize encoder slip and camera blur.
        - k_angular: Single proportional gain for all rotations (Square Up and tracking). 
          Increase if rotation is sluggish; decrease if the robot overshoots the centerline.
        - max_angular_speed: Absolute cap (rad/s) to prevent violent jerks.

        4. Nav2 Staging & Refinement
        - staging_distance: Meters in front of the tag where Nav2 drops the robot.
        - stop_distance: The exact distance (meters) from the camera lens to the tag to halt.
        - refinement_interval / refinement_cutoff: Limits how often the node spams Nav2 with 
          updated goals during the initial approach.
        - fallback_staging_offset: Distance subtracted if Nav2 rejects the initial staging goal.

        5. Robustness & Fallbacks
        - sensor_drop_tolerance: How long (seconds) to coast on memory during a camera flicker.
        - max_docking_time: Hard stopwatch (seconds) for the entire sequence to prevent hanging.
        - max_retries: Number of allowed backup attempts before settling.
        - backup_speed / backup_duration: Controls the exact distance of the safe retreat.
        """
    def __init__(self):
        super().__init__('docking_server')
        
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        
        # --- DOCKING PARAMETERS ---
        self.staging_distance = 0.40       
        self.stop_distance = 0.10        

        # --- ROBUSTNESS PARAMETERS ---
        self.fallback_staging_offset = 0.15
        self.max_docking_time = 180.0 #changed from 240.0        
        self.sensor_drop_tolerance = 1.0    
        
        # --- REFINEMENT PARAMETERS ---
        self.refinement_interval = 4.0    
        self.refinement_cutoff = 0.7     
        
        # --- GEOMETRIC DOCKING PARAMETERS ---
        self.intercept_ratio = 0.7        
        self.abort_ratio = 0.3            
        
        self.intercept_y_tolerance = 0.03 
        self.square_yaw_tolerance = 0.05  
        
        # --- SPEEDS AND GAINS ---
        self.slow_linear_speed = 0.03     
        self.k_angular = 2.0              
        self.max_angular_speed = 0.25
        
        # --- RETRY PARAMETERS ---
        self.max_retries = 3
        self.max_allowed_y_error = 0.4 #changed from 0.5   
        self.backup_speed = -0.1         
        self.backup_duration = 2.0  #changed from 3.0    

        # --- UNDOCKING PARAMETERS ---
        self.undock_distance = 0.20        
        self.undock_speed = -0.15         

        # Internal State Machine
        self.state = 'IDLE'
        self.target_tag = None
        self.target_frame = None
        self.current_retries = 0
        
        # Kinematics Memory
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw_error = 0.0
        
        self.tag_x_base = 0.0
        self.tag_y_base = 0.0
        self.tag_yaw_base = 0.0
        
        self.saved_target_pose_odom = None
        self.last_tag_sight_time = None
        self.is_blind = False
        self.initial_x_at_geometry = 0.0
        self.last_goal_update_time = None
        self.fallback_attempted = False
        
        # ROS 2 Interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        self.command_sub = self.create_subscription(String, '/mission_command', self.command_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.05, self.tick) 
        self.get_logger().debug("Geometric Docking Server initialized.")

    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            action = data.get('action')
            
            if action == 'START_DOCKING' and self.state == 'IDLE':
                self.target_tag = data.get('target')
                tag_id = self.target_tag.split(':')[-1]
                self.target_frame = f'nav2_dock_target_{tag_id}'
                
                self.current_retries = 0
                self.saved_target_pose_odom = None
                self.docking_start_time = self.get_clock().now()
                self.fallback_attempted = False
                self.last_goal_update_time = None
                
                # Check if we are ALREADY close enough to bypass Nav2 staging entirely!
                try:
                    if self.tf_buffer.can_transform('base_link', self.target_frame, rclpy.time.Time()):
                        t_base = self.tf_buffer.lookup_transform('base_link', self.target_frame, rclpy.time.Time())
                        dist_to_tag = math.hypot(t_base.transform.translation.x, t_base.transform.translation.y)
                        
                        # If robot is within 0.20m of the staging goal, skip Nav2
                        if dist_to_tag < (self.staging_distance + 0.20):
                            self.get_logger().info(f"Target found at {dist_to_tag:.2f}m. Extremely close to staging distance. Bypassing Nav2!")
                            self.state = 'COMPUTE_GEOMETRY'
                            return
                except TransformException:
                    pass
                
                self.get_logger().info(f"Initiating Nav2 Staging for {self.target_frame}.")
                self.state = 'NAV_TO_STAGING'
                self.send_staging_goal(self.staging_distance)
                
            elif action == 'START_UNDOCKING' and self.state == 'IDLE':
                self.undock_start_time = self.get_clock().now()
                self.state = 'UNDOCKING'
                self.get_logger().debug(f"Initiating Undocking Sequence. Target distance: {self.undock_distance}m")
                
            elif action == 'ABORT':
                self.abort_docking_sequence(send_status=False)
        except json.JSONDecodeError:
            pass

    def send_status(self, status, data=None):
        msg = {'sender': 'docker', 'status': status, 'data': data}
        self.status_pub.publish(String(data=json.dumps(msg)))

    def abort_docking_sequence(self, send_status=True):
        self.state = 'IDLE'
        self.docking_start_time = None
        self.cmd_vel_pub.publish(Twist())
        if send_status:
            self.send_status('DOCKING_FAILED', self.target_tag)

    def handle_nav_failure(self):
        if not self.fallback_attempted:
            self.fallback_attempted = True
            new_dist = self.staging_distance - self.fallback_staging_offset
            self.get_logger().warn(f"Nav2 staging goal rejected/failed. Attempting fallback at {new_dist:.2f}m...")
            self.send_staging_goal(new_dist)
        else:
            self.get_logger().error("Nav2 rejected fallback staging goal. Aborting dock.")
            self.abort_docking_sequence()

    def send_staging_goal(self, target_distance):
        try:
            if self.tf_buffer.can_transform('map', self.target_frame, rclpy.time.Time()):
                staging_pose = PoseStamped()
                staging_pose.header.frame_id = self.target_frame
                staging_pose.pose.position.x = target_distance 
                staging_pose.pose.orientation.z = 1.0 
                staging_pose.pose.orientation.w = 0.0 
                
                try:
                    transform = self.tf_buffer.lookup_transform('map', self.target_frame, rclpy.time.Time())
                    global_pose = tf2_geometry_msgs.do_transform_pose(staging_pose.pose, transform)
                    
                    global_staging_pose = PoseStamped()
                    global_staging_pose.header.frame_id = 'map'
                    global_staging_pose.header.stamp = self.get_clock().now().to_msg()
                    global_staging_pose.pose = global_pose
                except TransformException as e:
                    self.get_logger().warn(f"Transformation failed: {e}. Retrying...")
                    return
                
                global_staging_pose.pose.position.z = 0.0
                
                yaw = euler_from_quaternion(global_staging_pose.pose.orientation)
                global_staging_pose.pose.orientation.x = 0.0
                global_staging_pose.pose.orientation.y = 0.0
                global_staging_pose.pose.orientation.z = math.sin(yaw / 2.0)
                global_staging_pose.pose.orientation.w = math.cos(yaw / 2.0)

                self.nav_client.wait_for_server()
                goal_msg = NavigateToPose.Goal(pose=global_staging_pose)
                self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
                self.send_goal_future.add_done_callback(self.nav_goal_response_callback)
                
                self.last_goal_update_time = self.get_clock().now()
            else:
                self.get_logger().debug(f"Waiting for TF propagation: {self.target_frame} -> map...")
        except TransformException as e:
            self.get_logger().warn(f"TF exception: {e}. Waiting for tree...")

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.handle_nav_failure()
            return
            
        # 1. Store the CURRENT active goal handle
        self.current_goal_handle = goal_handle
        
        self.get_result_future = goal_handle.get_result_async()
        
        # 2. Pass the specific handle into the callback using a lambda
        self.get_result_future.add_done_callback(
            lambda fut, gh=goal_handle: self.nav_result_callback(fut, gh)
        )

    def nav_result_callback(self, future, goal_handle):
        # 3. The Gatekeeper: Ignore callbacks from old goals that were preempted by refinement
        if self.current_goal_handle and goal_handle != self.current_goal_handle:
            return

        self.current_goal_handle = None 
        
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().debug("Staging complete. Beginning Geometric Servoing.")
            self.state = 'COMPUTE_GEOMETRY'
        else:
            self.handle_nav_failure()

    def refine_staging_pose(self):
        if self.last_goal_update_time is None:
            # TF wasn't ready during the initial sweep, so keep attempting to send the first goal
            current_target_dist = self.staging_distance - self.fallback_staging_offset if self.fallback_attempted else self.staging_distance
            self.send_staging_goal(current_target_dist)
            return
            
        now = self.get_clock().now()
        elapsed = (now - self.last_goal_update_time).nanoseconds / 1e9
        
        if elapsed < self.refinement_interval:
            return

        try:
            t_base = self.tf_buffer.lookup_transform('base_link', self.target_frame, rclpy.time.Time())
            dist_to_tag = math.hypot(t_base.transform.translation.x, t_base.transform.translation.y)
            
            if dist_to_tag < self.refinement_cutoff:
                return 

            current_target_dist = self.staging_distance - self.fallback_staging_offset if self.fallback_attempted else self.staging_distance
            self.get_logger().debug(f"Refining Staging Pose (Tag Distance: {dist_to_tag:.2f}m)")
            self.send_staging_goal(current_target_dist)
            
        except TransformException:
            pass

    # =====================================================================
    # THE KINEMATICS ENGINE
    # =====================================================================
    def update_kinematics(self):
        now = self.get_clock().now()
        try:
            # 1. Grab the latest available TF
            t_base = self.tf_buffer.lookup_transform('base_link', self.target_frame, rclpy.time.Time())
            
            # 2. ALWAYS SAVE MEMORY FIRST
            # Even if the tag is 0.9s old, its global position in 'odom' is valid. 
            # We must memorize it before we do any age-rejection.
            try:
                self.saved_target_pose_odom = self.tf_buffer.lookup_transform('odom', self.target_frame, rclpy.time.Time())
            except TransformException:
                pass 

            # 3. NOW CHECK AGE FOR LIVE SERVOING
            transform_time = rclpy.time.Time.from_msg(t_base.header.stamp)
            age = abs((now - transform_time).nanoseconds / 1e9)
            if age > self.sensor_drop_tolerance:
                raise TransformException(f"Stale TF. Tag lost {age:.2f}s ago.")

            self.last_tag_sight_time = now
            self.is_blind = False
            
            # 4. Extract live vision coordinates
            tx = t_base.transform.translation.x
            ty = t_base.transform.translation.y
            tyaw = euler_from_quaternion(t_base.transform.rotation)
            self._calculate_true_errors(tx, ty, tyaw)
            return True

        except TransformException as e:
            if self.saved_target_pose_odom is None:
                if self.state != 'NAV_TO_STAGING':
                    self.get_logger().error(f"BLIND: No tag memory exists to fall back on. Reason: {e}")
                return False
                
            try:
                target_in_odom = PoseStamped()
                target_in_odom.header.frame_id = 'odom'
                target_in_odom.pose.position.x = self.saved_target_pose_odom.transform.translation.x
                target_in_odom.pose.position.y = self.saved_target_pose_odom.transform.translation.y
                target_in_odom.pose.orientation = self.saved_target_pose_odom.transform.rotation

                t_odom_to_base = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
                
                target_pose_base = tf2_geometry_msgs.do_transform_pose(target_in_odom.pose, t_odom_to_base)
                
                tx = target_pose_base.position.x
                ty = target_pose_base.position.y
                tyaw = euler_from_quaternion(target_pose_base.orientation)
                
                self.is_blind = True
                self._calculate_true_errors(tx, ty, tyaw)
                return True

            except TransformException:
                return False

    def _calculate_true_errors(self, tx, ty, tyaw):
        """Mathematically decouples heading from lateral offset."""
        self.tag_x_base = tx
        self.tag_y_base = ty
        self.tag_yaw_base = tyaw
        
        # True distance to tag along its centerline
        self.current_x = - (tx * math.cos(-tyaw) - ty * math.sin(-tyaw))
        # True perpendicular distance from the centerline
        self.current_y = - (tx * math.sin(-tyaw) + ty * math.cos(-tyaw))
        
        yaw_err = math.pi + tyaw
        self.current_yaw_error = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

    # =====================================================================
    # THE STATE MACHINE
    # =====================================================================
    def tick(self):
        if self.state == 'IDLE':
            return

        # 1. Master Stopwatch Guard (RESTORED)
        if self.docking_start_time:
            elapsed_total = (self.get_clock().now() - self.docking_start_time).nanoseconds / 1e9
            if elapsed_total > self.max_docking_time:
                self.get_logger().error(f"Hard timeout ({self.max_docking_time}s) reached. Robot stuck. Forcing abort.")
                self.abort_docking_sequence()
                return

        # 2. Always update eyes, even while Nav2 is driving
        kinematics_ok = self.update_kinematics()

        # 3. Handle Nav2 Staging State
        if self.state == 'NAV_TO_STAGING':
            # Failsafe: If Nav2 completely hangs on a staging goal (e.g. planner deadlock)
            if self.last_goal_update_time:
                nav_duration = (self.get_clock().now() - self.last_goal_update_time).nanoseconds / 1e9
                if nav_duration > 60.0:
                    self.get_logger().warn("Nav2 Staging DEADLOCKED for 60 seconds! Forcing manual geometric servoing intercept.")
                    if self.current_goal_handle:
                        self.current_goal_handle.cancel_goal_async()
                        self.current_goal_handle = None
                    self.state = 'COMPUTE_GEOMETRY'
                    return

            self.refine_staging_pose()
            return # Let Nav2 finish its job. Do not execute servoing logic below.

        # 4. Halt if totally blind and no memory exists during Servoing
        if not kinematics_ok:
            if self.state != 'RETRY_BACKUP':
                self.cmd_vel_pub.publish(Twist()) 
            return

        # DEBUG VISIBILITY
        self.get_logger().debug(f"State: {self.state} | X_Err: {self.current_x:.2f} | Y_Err: {self.current_y:.2f} | Yaw: {self.current_yaw_error:.2f}", throttle_duration_sec=1.0)

        cmd = Twist()

        if self.state == 'COMPUTE_GEOMETRY':
            self.initial_x_at_geometry = self.current_x
            self.state = 'INTERCEPT'
            
        elif self.state == 'INTERCEPT':
            lookahead_x = self.initial_x_at_geometry * self.intercept_ratio
            
            if abs(self.current_y) < self.intercept_y_tolerance:
                self.state = 'SQUARE_UP'
                self.cmd_vel_pub.publish(Twist())
                return
                
            if self.current_x < (self.staging_distance * self.abort_ratio):
                self.get_logger().warn("Hit abort boundary during intercept. Squaring up.")
                self.state = 'SQUARE_UP'
                self.cmd_vel_pub.publish(Twist())
                return

            # Pure Pursuit Target "Carrot" on the centerline
            carrot_x = self.tag_x_base + lookahead_x * math.cos(self.tag_yaw_base)
            carrot_y = self.tag_y_base + lookahead_x * math.sin(self.tag_yaw_base)
            
            target_heading = math.atan2(carrot_y, carrot_x)
            
            cmd.linear.x = self.slow_linear_speed
            cmd.angular.z = max(min(self.k_angular * target_heading, self.max_angular_speed), -self.max_angular_speed)
            self.cmd_vel_pub.publish(cmd)

        elif self.state == 'SQUARE_UP':
            cmd.linear.x = 0.0
            target_ang_vel = self.k_angular * self.current_yaw_error
            cmd.angular.z = max(min(target_ang_vel, self.max_angular_speed), -self.max_angular_speed)
            
            if abs(self.current_yaw_error) < self.square_yaw_tolerance:
                self.cmd_vel_pub.publish(Twist())
                self.state = 'EVALUATE_POSITION'
            else:
                self.cmd_vel_pub.publish(cmd)

        elif self.state == 'EVALUATE_POSITION':
            if abs(self.current_y) <= self.max_allowed_y_error:
                self.get_logger().debug("Alignment verified. Commencing Final Plunge.")
                self.state = 'FINAL_PLUNGE'
            else:
                if self.current_retries < self.max_retries:
                    self.current_retries += 1
                    
                    # --- THE GEOMETRIC RUNWAY CHECK ---
                    # The safe distance to restart an intercept is exactly where 
                    # the robot originally planned to finish its lateral movement.
                    safe_retry_distance = self.staging_distance * self.intercept_ratio
                    
                    if self.current_x >= safe_retry_distance:
                        self.get_logger().warn(f"Y-error high ({self.current_y:.2f}m), but runway is sufficient ({self.current_x:.2f}m). Retrying immediately.")
                        self.state = 'COMPUTE_GEOMETRY'
                    else:
                        self.get_logger().warn(f"Y-error high ({self.current_y:.2f}m). Runway too short ({self.current_x:.2f}m). Backing up to {safe_retry_distance:.2f}m.")
                        self.backup_start_time = self.get_clock().now()
                        self.state = 'RETRY_BACKUP'
                else:
                    self.get_logger().warn("Out of retries. Plunging anyway.")
                    self.state = 'FINAL_PLUNGE'

        elif self.state == 'RETRY_BACKUP':
            cmd.linear.x = self.backup_speed
            self.cmd_vel_pub.publish(cmd)
            
            elapsed = (self.get_clock().now() - self.backup_start_time).nanoseconds / 1e9
            
            # --- DYNAMIC REVERSE CUTOFF ---
            safe_retry_distance = self.staging_distance * self.intercept_ratio
            
            if self.current_x >= safe_retry_distance or elapsed > self.backup_duration:
                self.cmd_vel_pub.publish(Twist())
                self.state = 'COMPUTE_GEOMETRY'
            
            elapsed = (self.get_clock().now() - self.backup_start_time).nanoseconds / 1e9
            if elapsed > self.backup_duration:
                self.cmd_vel_pub.publish(Twist())
                self.state = 'COMPUTE_GEOMETRY'

        elif self.state == 'FINAL_PLUNGE':
            x_error = self.current_x - self.stop_distance
            
            if x_error <= 0.01:
                self.get_logger().info("DOCKING SEQUENCE COMPLETE.")
                self.cmd_vel_pub.publish(Twist())
                self.send_status('DOCKING_COMPLETE', self.target_tag)
                self.docking_start_time = None
                self.state = 'IDLE'
            else:
                cmd.linear.x = self.slow_linear_speed
                # actively fight lateral drift by steering at the tag center
                target_heading = math.atan2(self.tag_y_base, self.tag_x_base)
                cmd.angular.z = max(min(self.k_angular * target_heading, self.max_angular_speed), -self.max_angular_speed)
                self.cmd_vel_pub.publish(cmd)

        elif self.state == 'UNDOCKING':
            cmd.linear.x = self.undock_speed
            self.cmd_vel_pub.publish(cmd)
            
            duration = self.undock_distance / abs(self.undock_speed)
            elapsed = (self.get_clock().now() - self.undock_start_time).nanoseconds / 1e9
            
            if elapsed > duration:
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("Undocking complete.")
                self.send_status('UNDOCKING_COMPLETE')
                self.state = 'IDLE'


def main(args=None):
    rclpy.init(args=args)
    node = DockingServer()
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