import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import RPi.GPIO as GPIO

SERVO_PIN = 12
PWM_FREQ = 50
CCW_DUTY = 10.0
CW_DUTY = 5.0
TIME_PER_REV = 0.87


class RPIShooterNode(Node):
    def __init__(self):
        super().__init__('rpi_shooter_node')

        self.is_firing = False
        self.lock = threading.Lock()

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

        self.create_service(Trigger, '/fire_ball', self.service_callback)

        self.get_logger().info('RPi Shooter ready (preloaded).')

    def service_callback(self, request, response):
        fired = self._attempt_fire()
        response.success = fired
        response.message = 'Fired!' if fired else 'Already firing, please wait'
        return response

    def _attempt_fire(self) -> bool:
        with self.lock:
            if self.is_firing:
                self.get_logger().warn('Already firing - ignoring trigger')
                return False
            self.is_firing = True

        self.get_logger().info('Firing!')
        thread = threading.Thread(target=self._fire_sequence, daemon=True)
        thread.start()
        return True

    def _fire_sequence(self):
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
            with self.lock:
                self.is_firing = False

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RPIShooterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
