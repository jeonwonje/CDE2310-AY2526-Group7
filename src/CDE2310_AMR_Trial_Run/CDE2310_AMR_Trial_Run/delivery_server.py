import rclpy
from rclpy.node import Node
import threading
import time
import json
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray

import RPi.GPIO as GPIO

TARGET_TAG_ID = 3

# Servo constants
SERVO_PIN = 12
PWM_FREQ = 50
CCW_DUTY = 10.0
CW_DUTY = 5.0
TIME_PER_REV = 0.87


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

        # GPIO setup
        self.is_firing = False
        self.fire_lock = threading.Lock()

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
        self.pwm.start(0)
        time.sleep(0.5)

        # Startup preload: retract plunger + seat first ball
        self.get_logger().info('Startup preload: retracting plunger...')
        self.pwm.ChangeDutyCycle(CW_DUTY)
        time.sleep(TIME_PER_REV * 2)
        self.pwm.ChangeDutyCycle(0)
        time.sleep(0.2)
        self.get_logger().info('Startup preload: seating first ball...')
        self.pwm.ChangeDutyCycle(CCW_DUTY)
        time.sleep(TIME_PER_REV * 1.5 * 0.45)
        self.pwm.ChangeDutyCycle(0)

        # Subscriptions and Publishers
        self.command_sub = self.create_subscription(
            String, '/mission_command', self.command_callback, 10)
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)

        self.get_logger().info('Delivery Server ready (shooter preloaded).')

    def command_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            if cmd.get('action') == 'START_DELIVERY':
                target = cmd.get('target')
                self.get_logger().info(f"Received START_DELIVERY for {target}")

                if target == 'tag36h11:0':
                    self.active_delivery_target = target
                    thread = threading.Thread(
                        target=self.static_delivery_sequence, daemon=True)
                    thread.start()

                elif target == 'tag36h11:2':
                    self.active_delivery_target = target
                    self.dynamic_shots_fired = 0
                    self.get_logger().debug(
                        "Dynamic delivery activated. "
                        "Scanning for target tag 3...")

        except json.JSONDecodeError:
            self.get_logger().error("Malformed JSON in /mission_command")

    def static_delivery_sequence(self):
        self.get_logger().debug('Static delivery sequence started')

        self.fire_ball()
        time.sleep(4)

        self.fire_ball()
        time.sleep(6)

        self.fire_ball()

        self.complete_delivery()

    def detection_callback(self, msg: AprilTagDetectionArray):
        if self.active_delivery_target != 'tag36h11:2':
            return

        if self.on_cooldown:
            return

        for detection in msg.detections:
            if detection.id == TARGET_TAG_ID:
                self.get_logger().info(
                    f'Tag ID {TARGET_TAG_ID} sighted! Triggering launch.')
                self.on_cooldown = True
                threading.Thread(
                    target=self.handle_dynamic_fire, daemon=True).start()
                break

    def handle_dynamic_fire(self):
        success = self.fire_ball()

        if success:
            self.dynamic_shots_fired += 1
            self.get_logger().info(
                f"Shot fired. Count: {self.dynamic_shots_fired}"
                f"/{self.max_dynamic_shots}")

        if self.dynamic_shots_fired >= self.max_dynamic_shots:
            self.complete_delivery()
        else:
            self._start_cooldown()

    def fire_ball(self) -> bool:
        """Directly actuate the servo to fire a ball."""
        with self.fire_lock:
            if self.is_firing:
                self.get_logger().warn('Already firing - ignoring trigger')
                return False
            self.is_firing = True

        self.get_logger().info('Firing!')
        try:
            # 1. Fire: short CCW push (ball already seated)
            self.pwm.ChangeDutyCycle(CCW_DUTY)
            time.sleep(TIME_PER_REV * 1.5 * 0.30)
            self.pwm.ChangeDutyCycle(0)
            time.sleep(0.3)
            # 2. Retract: CW pull plunger back
            self.pwm.ChangeDutyCycle(CW_DUTY)
            time.sleep(TIME_PER_REV * 2)
            self.pwm.ChangeDutyCycle(0)
            # 3. Preload: nudge next ball into barrel
            time.sleep(0.2)
            self.pwm.ChangeDutyCycle(CCW_DUTY)
            time.sleep(TIME_PER_REV * 1.5 * 0.45)
            self.pwm.ChangeDutyCycle(0)
        finally:
            with self.fire_lock:
                self.is_firing = False
        return True

    def _start_cooldown(self):
        self.get_logger().debug(
            f"Cooldown {self.cooldown_seconds}s to prevent double firing.")
        self.cooldown_timer = self.create_timer(
            self.cooldown_seconds, self._end_cooldown)

    def _end_cooldown(self):
        self.on_cooldown = False
        self.cooldown_timer.cancel()
        self.destroy_timer(self.cooldown_timer)
        self.cooldown_timer = None
        self.get_logger().debug("Cooldown cleared. Ready for next pass.")

    def complete_delivery(self):
        self.get_logger().info(
            f'Delivery complete for {self.active_delivery_target}')

        msg = {
            'sender': 'deliverer',
            'status': 'DELIVERY_COMPLETE',
            'data': self.active_delivery_target
        }
        self.status_pub.publish(String(data=json.dumps(msg)))

        self.active_delivery_target = None

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


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
