import rclpy
from rclpy.node import Node
import threading
import time
import json
from std_msgs.msg import String
from std_srvs.srv import Trigger
from apriltag_msgs.msg import AprilTagDetectionArray

TARGET_TAG_ID = 3

class DeliveryServer(Node):
    def __init__(self):
        super().__init__('delivery_server')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        
        self.active_delivery_target = None
        
        # Cooldown management for dynamic station
        self.on_cooldown = False
        self.cooldown_seconds = 4.0
        self.cooldown_timer = None
        
        # Internal Shot Tracker
        self.dynamic_shots_fired = 0
        self.max_dynamic_shots = 3
        
        # Trigger Service Client
        self.fire_client = self.create_client(Trigger, '/fire_ball')
        
        # Subscriptions and Publishers
        self.command_sub = self.create_subscription(String, '/mission_command', self.command_callback, 10)
        self.detection_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        
        self.get_logger().debug('Delivery Server is ready.')

    def command_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            if cmd.get('action') == 'START_DELIVERY':
                target = cmd.get('target')
                self.get_logger().info(f"Received START_DELIVERY for {target}")
                
                if target == 'tag36h11:0':
                    # Static station protocol
                    self.active_delivery_target = target
                    thread = threading.Thread(target=self.static_delivery_sequence, daemon=True)
                    thread.start()
                
                elif target == 'tag36h11:2':
                    # Dynamic station protocol
                    self.active_delivery_target = target
                    self.dynamic_shots_fired = 0
                    self.get_logger().debug("Dynamic delivery activated. Scanning for target tag 3...")

        except json.JSONDecodeError:
            self.get_logger().error("Malformed JSON in /mission_command")

    def static_delivery_sequence(self):
        self.get_logger().debug('Static delivery sequence started')
        
        self.call_fire()
        time.sleep(4)
        
        self.call_fire()
        time.sleep(6)
        
        self.call_fire()
        
        # Finish delivery
        self.complete_delivery()

    def detection_callback(self, msg: AprilTagDetectionArray):
        if self.active_delivery_target != 'tag36h11:2':
            return
            
        if self.on_cooldown:
            return
            
        for detection in msg.detections:
            if detection.id == TARGET_TAG_ID:
                self.get_logger().info(f'Tag ID {TARGET_TAG_ID} sighted passing by! Triggering launch.')
                self.on_cooldown = True
                threading.Thread(target=self.handle_dynamic_fire, daemon=True).start()
                break

    def handle_dynamic_fire(self):
        success, out_of_balls = self.call_fire()
        
        if success:
            self.dynamic_shots_fired += 1
            self.get_logger().info(f"Shot successfully fired. Count: {self.dynamic_shots_fired}/{self.max_dynamic_shots}")
            
        # The physical RPi node lacks "Out of balls" telemetry, so we use the counter
        if out_of_balls or self.dynamic_shots_fired >= self.max_dynamic_shots:
            self.complete_delivery()
        else:
            self._start_cooldown()

    def call_fire(self):
        """ Blocking call to the hardware fire service """
        if not self.fire_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Hardware fire service not available!")
            return False, False
            
        request = Trigger.Request()
        future = self.fire_client.call_async(request)
        
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        
        response = future.result()
        if response:
            self.get_logger().debug(f"RPi Status: {response.message}")
            out_of_balls = ('Out of balls' in response.message)
            return response.success, out_of_balls
        else:
            self.get_logger().error("Failed to call RPi shooter service")
            return False, False

    def _start_cooldown(self):
        self.get_logger().debug(f"Target placed on {self.cooldown_seconds}s cooldown to prevent double firing.")
        self.cooldown_timer = self.create_timer(self.cooldown_seconds, self._end_cooldown)

    def _end_cooldown(self):
        self.on_cooldown = False
        self.cooldown_timer.cancel()
        self.destroy_timer(self.cooldown_timer)
        self.cooldown_timer = None
        self.get_logger().debug("Target cooldown cleared. Ready for next pass.")

    def complete_delivery(self):
        self.get_logger().info(f'Delivery sequence complete for {self.active_delivery_target}')
        
        msg = {'sender': 'deliverer', 'status': 'DELIVERY_COMPLETE', 'data': self.active_delivery_target}
        self.status_pub.publish(String(data=json.dumps(msg)))
        
        self.active_delivery_target = None


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
