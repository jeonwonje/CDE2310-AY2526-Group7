#!/usr/bin/env python3
import rclpy
import json
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

ROBOT_NAME = "Boots"

class MissionCoordinator(Node):
    def __init__(self):
        """
        MissionCoordinator: Central State Machine and Goal Dispatcher

        --- COORDINATOR TUNING GUIDE ---

        1. Robustness & Blacklisting
        - blacklist_timeout (float, default: 30.0): Time in seconds to ignore a tag 
          after a failed docking attempt. 
          * Tuning: If set too short (< 15s), the robot might turn exploration back on, 
            spin around, see the tag again from the exact same bad angle, and enter 
            an infinite loop. If set too long (> 90s), the robot might finish the entire 
            maze and trigger the Search phase while the tag is still blacklisted.
        - delivery_timeout (float, default: 90.0): Time in seconds to wait patiently
          in DELIVERING state before forcefully abandoning the tag and resuming mission.
        2. TF Freshness
          stale_tf_threshold (float, default: 0.5): Max age of a transform (s) to be 
          considered "visible." 
          * Tuning: If the tag detections are flickering, increase slightly (e.g., 0.8s). 
            If the robot is triggering on "ghosts" after moving, decrease (e.g., 0.3s).
        """
        super().__init__('mission_coordinator')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        
        self.declare_parameter('enable_delivery', True)
        self.enable_delivery = self.get_parameter('enable_delivery').get_parameter_value().bool_value
        
        # --- TIMEOUT PARAMETERS ---
        self.declare_parameter('master_mission_timeout', 1200.0)
        self.master_mission_timeout = self.get_parameter('master_mission_timeout').get_parameter_value().double_value
        self.declare_parameter('initial_exploration_timeout', 300.0)
        self.initial_exploration_timeout = self.get_parameter('initial_exploration_timeout').get_parameter_value().double_value
        self.declare_parameter('delivery_timeout', 90.0)
        self.delivery_timeout = self.get_parameter('delivery_timeout').get_parameter_value().double_value
        
        # --- ROBUSTNESS PARAMETERS ---
        self.blacklist_timeout = 30.0
        self.blacklisted_tags = {} # Format: {'tag_name': expiration_time_object}
        self.exploration_completed = False # Tracks if we have exhausted the map
        self.stale_tf_threshold = 0.5
        # -----------------------------

        # Target Tracking
        self.target_tags = ['tag36h11:0', 'tag36h11:2']
        self.docked_tags = set()
        self.active_search_tag = None
        
        # State Machine
        self.state = 'INIT'
        self.timeout_search_active = False
        self.exploration_active_duration = 0.0
        self.delivery_active_duration = 0.0
        self.active_delivery_target_backup = None
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Pub / Sub for Node Communication
        self.command_pub = self.create_publisher(String, '/mission_command', 10)
        self.status_sub = self.create_subscription(String, '/mission_status', self.status_callback, 10)
        
        # Exploration Service
        self.toggle_explore_client = self.create_client(SetBool, 'toggle_exploration')
        while not self.toggle_explore_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug(f"{ROBOT_NAME} is waiting for /toggle_exploration service...")
            
        # Main Loop
        self.timer = self.create_timer(0.1, self.tick)
        
        self.mission_start_time = None
        self.last_timer_time = None
        self.timeout_timer = self.create_timer(1.0, self.check_timeouts)
        
        self.start_exploration()

    def check_timeouts(self):
        if self.state in ['INIT', 'MISSION_COMPLETE', 'MISSION_TIMED_OUT']:
            return
            
        now = self.get_clock().now()
        if self.last_timer_time is None:
            self.last_timer_time = now
            return
            
        dt = (now - self.last_timer_time).nanoseconds / 1e9
        self.last_timer_time = now
        
        # 1. Check Master Mission Timeout
        if self.mission_start_time is not None:
            mission_elapsed = (now - self.mission_start_time).nanoseconds / 1e9
            if mission_elapsed > self.master_mission_timeout:
                self.get_logger().error(f"{ROBOT_NAME} has reached MASTER MISSION TIMEOUT ({self.master_mission_timeout}s). {ROBOT_NAME} is aborting!")
                self.state = 'MISSION_TIMED_OUT'
                self.stop_exploration()
                self.send_command('ABORT_SEARCH')
                return

        # 2. Check Exploration Phase Timeout
        missing_tags = [t for t in self.target_tags if t not in self.docked_tags]
        if self.state == 'EXPLORING' and missing_tags:
            self.exploration_active_duration += dt
            if self.exploration_active_duration > self.initial_exploration_timeout:
                self.get_logger().warn(f"{ROBOT_NAME}'s exploration timeout ({self.initial_exploration_timeout}s) was reached. {ROBOT_NAME} is forcing the search phase.")
                self.timeout_search_active = True
                self.stop_exploration()
                self.resume_mission()

        # 3. Check Delivery Phase Timeout
        if self.state == 'DELIVERING':
            self.delivery_active_duration += dt
            if self.delivery_active_duration > self.delivery_timeout:
                target = getattr(self, 'active_delivery_target_backup', 'unknown')
                self.get_logger().error(f"{ROBOT_NAME}'s delivery timeout reached ({self.delivery_timeout}s). Hardware stalled checking target {target}. {ROBOT_NAME} is forcibly marking it complete and abandoning it.")
                self.docked_tags.add(target)
                self.state = 'UNDOCKING'
                self.send_command('START_UNDOCKING')

    def send_command(self, action, target=None, extra_data=None):
        """Helper to send JSON commands to the subsidiary nodes."""
        cmd = {'action': action, 'target': target}
        if extra_data:
            cmd.update(extra_data)
        self.command_pub.publish(String(data=json.dumps(cmd)))
        self.get_logger().debug(f"{ROBOT_NAME} is sending command: {action} | Target: {target}")

    def start_exploration(self):
        if self.mission_start_time is None:
            self.mission_start_time = self.get_clock().now()
        req = SetBool.Request()
        req.data = True
        self.toggle_explore_client.call_async(req)
        self.state = 'EXPLORING'
        self.get_logger().info(f"{ROBOT_NAME}'s State -> EXPLORING")

    def stop_exploration(self):
        req = SetBool.Request()
        req.data = False
        self.toggle_explore_client.call_async(req)

    def tick(self):
        # The Coordinator's only active job is to watch the TF tree like a hawk
        # if it is in a state where finding a tag matters.
        if self.state in ['EXPLORING', 'SEARCHING']:
            self.monitor_tf_for_tags()

    def monitor_tf_for_tags(self):
        now = self.get_clock().now()
        
        for tag in self.target_tags:
            if tag in self.docked_tags:
                continue
            
            # 1. Blacklist Check
            if tag in self.blacklisted_tags:
                if now < self.blacklisted_tags[tag]:
                    continue
                else:
                    self.get_logger().debug(f"{ROBOT_NAME}'s blacklist expired for {tag}. The tag is eligible for {ROBOT_NAME} again.")
                    del self.blacklisted_tags[tag]
                
            try:
                # 2. Fetch the latest transform
                t = self.tf_buffer.lookup_transform('base_link', tag, rclpy.time.Time())
                
                # 3. Freshness Check (The Fix)
                # Convert the message stamp to a ROS Time object
                msg_time = rclpy.time.Time.from_msg(t.header.stamp)
                age = (now - msg_time).nanoseconds / 1e9
                
                if age < self.stale_tf_threshold:
                    self.get_logger().info(f"{ROBOT_NAME} acquired Target {tag} (Age: {age:.3f}s)")
                    
                    if self.state == 'EXPLORING':
                        self.stop_exploration()
                    elif self.state == 'SEARCHING':
                        self.send_command('ABORT_SEARCH')
                        
                    self.state = 'DOCKING'
                    self.send_command('START_DOCKING', target=tag)
                    return 
                
            except TransformException:
                pass

    def status_callback(self, msg):
        """Listens to the Docker, Searcher, and Explorer nodes to advance the state machine."""
        try:
            status_data = json.loads(msg.data)
            sender = status_data.get('sender')
            status = status_data.get('status')
            data = status_data.get('data') # 'data' contains the tag name from the Docker
            
            # 1. Handle Exploration Completion
            if sender == 'explorer' and status == 'EXPLORATION_COMPLETE':
                if self.state == 'EXPLORING':
                    self.exploration_completed = True
                    self.resume_mission()
                    
            # 2. Handle Docking Success
            elif sender == 'docker' and status == 'DOCKING_COMPLETE':
                if self.state == 'DOCKING':
                    if not self.enable_delivery:
                        self.docked_tags.add(data)
                        self.get_logger().info(f"{ROBOT_NAME} successfully docked at {data}. Delivery disabled -> {ROBOT_NAME} is UNDOCKING.")
                        self.state = 'UNDOCKING'
                        self.send_command('START_UNDOCKING')
                    else:
                        self.state = 'DELIVERING'
                        self.delivery_active_duration = 0.0
                        self.active_delivery_target_backup = data
                        self.get_logger().info(f"{ROBOT_NAME} is docked at {data}. {ROBOT_NAME} is sending the START_DELIVERY command.")
                        self.send_command('START_DELIVERY', target=data)
                        
            # 2.5a Handle Ball Fired Confirmation
            elif sender == 'deliverer' and status == 'BALL_FIRED':
                self.get_logger().info(f"{ROBOT_NAME} has confirmed BALL FIRED — {data}")

            # 2.5b Handle Delivery Success
            elif sender == 'deliverer' and status == 'DELIVERY_COMPLETE':
                if self.state == 'DELIVERING':
                    self.docked_tags.add(data)
                    self.get_logger().info(f"{ROBOT_NAME} has successfully fully serviced and delivered to {data}.")
                    self.state = 'UNDOCKING'
                    self.send_command('START_UNDOCKING')
                    
            # 2.75 Handle Undocking Success
            elif sender == 'docker' and status == 'UNDOCKING_COMPLETE':
                if self.state == 'UNDOCKING':
                    self.get_logger().info(f"{ROBOT_NAME} completed undocking. {ROBOT_NAME} is completely cleared from the tag.")
                    self.resume_mission()
                    
            # 3. Handle Docking Failure (THE ROBUSTNESS CATCH)
            elif sender == 'docker' and status == 'DOCKING_FAILED':
                if self.state == 'DOCKING':
                    # Calculate expiration time
                    expiration = self.get_clock().now() + rclpy.duration.Duration(seconds=self.blacklist_timeout)
                    self.blacklisted_tags[data] = expiration
                    
                    self.get_logger().error(f"{ROBOT_NAME} failed docking at {data}. {ROBOT_NAME} has blacklisted it for {self.blacklist_timeout}s.")
                    self.resume_mission() # Force the robot to keep moving
                    
            # 4. Handle Search Completion (Spin finished, tag still not found)
            elif sender == 'searcher' and status == 'SEARCH_FAILED':
                if self.state == 'SEARCHING':
                    self.get_logger().error(f"{ROBOT_NAME}'s search failed for {self.active_search_tag}.")
                    self.docked_tags.add(self.active_search_tag) # Mark as done to prevent infinite loops
                    self.resume_mission()

        except json.JSONDecodeError:
            self.get_logger().error(f"{ROBOT_NAME} received a completely malformed status message!")

    def resume_mission(self):
        """Intelligently routes the robot to its next task based on global state."""
        missing_tags = [t for t in self.target_tags if t not in self.docked_tags]
        
        if not missing_tags:
            self.timeout_search_active = False
            if self.exploration_completed:
                self.state = 'MISSION_COMPLETE'
                self.get_logger().info(f"{ROBOT_NAME} shouts MISSION ACCOMPLISHED! All target tags are processed and the map is complete.")
            else:
                self.get_logger().info(f"{ROBOT_NAME} processed all target tags, but the map is not complete. {ROBOT_NAME} is resuming exploration to finish the map.")
                self.start_exploration()
            return
            
        if self.exploration_completed or getattr(self, 'timeout_search_active', False):
            self.state = 'SEARCHING'
            self.active_search_tag = missing_tags[0] # Pick the next missing tag
            self.get_logger().warn(f"{ROBOT_NAME} is resuming the Search Phase for {self.active_search_tag}.")
            self.send_command(
                'START_SEARCH', 
                target=self.active_search_tag, 
                extra_data={'docked_tags': list(self.docked_tags)}
            )
        else:
            # Map isn't finished yet. Go back to exploring.
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