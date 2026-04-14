
# THIS IS ON THE PI

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import time
import json
import RPi.GPIO as GPIO

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray

TARGET_TAG_ID = 3

# --- HARDWARE CONSTANTS ---
SERVO_PIN = 12
PWM_FREQ = 50
CCW_DUTY = 10.0
CW_DUTY = 5.0
TIME_PER_REV = 0.87

class IntegratedDeliveryServer(Node):
    def __init__(self):
        super().__init__('delivery_server')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        
        # --- TARGETING "CROSSHAIRS" PARAMETERS ---
        # Assuming 640x480 resolution, 320 is the exact mathematical center.
        self.crosshair_center_x = 320.0 
        
        # How many pixels left/right of the center the tag can be to trigger a shot.
        # A tolerance of 40 means it fires when the tag is between x=280 and x=360.
        self.fire_window_tolerance = 50.0 
        
        # Hardware State
        self.is_firing = False
        self.lock = threading.Lock()
        self._shutdown_requested = False

        # Mission State
        self.active_delivery_target = None
        self.on_cooldown = False
        self.cooldown_seconds = 4.0
        self.cooldown_timer = None
        self.dynamic_shots_fired = 0
        self.max_dynamic_shots = 3

        # Subscriptions and Publishers
        self.command_sub = self.create_subscription(String, '/mission_command', self.command_callback, 10)
        self.detection_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        
        # --- HARDWARE INITIALIZATION ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
        self.pwm.start(7.5)
        time.sleep(0.5)

        self.get_logger().info('Startup Initialization: Resetting plunger...')
        self._reset()
        self.get_logger().info('Startup Initialization: Preloading first ball...')
        self._preload()
        
        self.get_logger().info('Integrated Delivery Server is ready.')

    # ==========================================
    # HARDWARE CONTROL METHODS
    # ==========================================
    def _fire(self):
        if self._shutdown_requested: return
        self.pwm.ChangeDutyCycle(CCW_DUTY)
        time.sleep(TIME_PER_REV * 1.5 * 0.30)
        self.pwm.ChangeDutyCycle(0)
        time.sleep(0.3)

    def _reset(self):
        if self._shutdown_requested: return
        self.pwm.ChangeDutyCycle(CW_DUTY)
        time.sleep(TIME_PER_REV * 2)
        self.pwm.ChangeDutyCycle(0)

    def _preload(self):
        if self._shutdown_requested: return
        time.sleep(0.2)
        self.pwm.ChangeDutyCycle(CCW_DUTY)
        time.sleep(TIME_PER_REV * 1.5 * 0.45)
        self.pwm.ChangeDutyCycle(0)

    def attempt_fire(self) -> bool:
        with self.lock:
            if self.is_firing:
                self.get_logger().warn('Hardware is busy - ignoring trigger')
                return False
            self.is_firing = True

        self.get_logger().info('FIRING!')
        threading.Thread(target=self._fire_sequence, daemon=True).start()
        return True

    def _fire_sequence(self):
        try:
            self._fire()
            self._reset()
            self._preload()
        finally:
            with self.lock:
                self.is_firing = False

    # ==========================================
    # MISSION LOGIC METHODS
    # ==========================================
    def command_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            if cmd.get('action') == 'START_DELIVERY':
                target = cmd.get('target')
                self.get_logger().info(f"Received START_DELIVERY for {target}")
                
                if target == 'tag36h11:0':
                    self.active_delivery_target = target
                    threading.Thread(target=self.static_delivery_sequence, daemon=True).start()
                
                elif target == 'tag36h11:2':
                    self.active_delivery_target = target
                    self.dynamic_shots_fired = 0
                    self.get_logger().info(f"Dynamic delivery activated. Waiting for Tag 3 near pixel X={self.crosshair_center_x}...")

        except json.JSONDecodeError:
            self.get_logger().error("Malformed JSON in /mission_command")

    def static_delivery_sequence(self):
        self.get_logger().info('Static delivery sequence started')
        
        self.attempt_fire()
        time.sleep(4.0)
        
        self.attempt_fire()
        time.sleep(6.0)
        
        self.attempt_fire()
        time.sleep(4.0)
        
        self.complete_delivery()

    # ==========================================
    # TARGETING LOGIC
    # ==========================================
    def detection_callback(self, msg: AprilTagDetectionArray):
        if self.active_delivery_target != 'tag36h11:2':
            return
            
        if self.on_cooldown:
            return
            
        for detection in msg.detections:
            if detection.id == TARGET_TAG_ID:
                # Extract the horizontal pixel center of the tag
                tag_x = detection.centre.x 
                
                # Calculate how far the tag is from our ideal crosshair
                pixel_error = abs(tag_x - self.crosshair_center_x)
                
                if pixel_error <= self.fire_window_tolerance:
                    self.get_logger().info(f'Tag 3 in crosshairs (X={tag_x:.1f}). Triggering launch!')
                    self.on_cooldown = True
                    threading.Thread(target=self.handle_dynamic_fire, daemon=True).start()
                else:
                    # Provide telemetry so you can tune your robot physically
                    self.get_logger().debug(f'Tag 3 visible but out of bounds (X={tag_x:.1f}). Target is {pixel_error:.1f}px away from center.')
                break

    def handle_dynamic_fire(self):
        success = self.attempt_fire()
        
        if success:
            self.dynamic_shots_fired += 1
            self.get_logger().info(f"Shot count: {self.dynamic_shots_fired}/{self.max_dynamic_shots}")
            
        if self.dynamic_shots_fired >= self.max_dynamic_shots:
            time.sleep(4.0) 
            self.complete_delivery()
        else:
            self._start_cooldown()

    def _start_cooldown(self):
        self.get_logger().info(f"Target placed on {self.cooldown_seconds}s cooldown.")
        self.cooldown_timer = self.create_timer(self.cooldown_seconds, self._end_cooldown)

    def _end_cooldown(self):
        self.on_cooldown = False
        self.cooldown_timer.cancel()
        self.destroy_timer(self.cooldown_timer)
        self.cooldown_timer = None
        self.get_logger().info("Cooldown cleared. Ready for next pass.")

    def complete_delivery(self):
        self.get_logger().info(f'Delivery sequence complete for {self.active_delivery_target}')
        msg = {'sender': 'deliverer', 'status': 'DELIVERY_COMPLETE', 'data': self.active_delivery_target}
        self.status_pub.publish(String(data=json.dumps(msg)))
        self.active_delivery_target = None

    def destroy_node(self):
        self.get_logger().info("Node killed. Forcing hardware reset...")
        self._shutdown_requested = True
        time.sleep(0.1) 
        
        self.pwm.ChangeDutyCycle(CW_DUTY)
        time.sleep(TIME_PER_REV * 2)
        self.pwm.ChangeDutyCycle(0)
        
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedDeliveryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()