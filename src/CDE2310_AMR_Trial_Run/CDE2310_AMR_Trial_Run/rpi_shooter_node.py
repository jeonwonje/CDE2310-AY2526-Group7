import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import RPi.GPIO as GPIO

SERVO_PIN = 12
PWM_FREQ = 50       
CCW_DUTY = 10.0     
TIME_PER_REV = 0.87 
MAX_BALLS = 3

class RPIShooterNode(Node):
    def __init__(self):
        super().__init__('rpi_shooter_node')

        self.balls_remaining = MAX_BALLS
        self.is_firing = False
        self.lock = threading.Lock()

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
        self.pwm.start(0)

        # Reactive Trigger Server
        self.create_service(Trigger, '/fire_ball', self.service_callback)

        self.get_logger().info(
            f'RPi Hardware Shooter ready. Balls loaded: {self.balls_remaining}'
        )

    def service_callback(self, request, response):
        fired = self._attempt_fire()
        response.success = fired
        if fired:
            response.message = f'Fired! Balls remaining: {self.balls_remaining}'
        elif self.balls_remaining == 0:
            response.message = 'Out of balls'
        else:
            response.message = 'Already firing, please wait'
        return response

    def _attempt_fire(self) -> bool:
        with self.lock:
            if self.is_firing:
                self.get_logger().warn('Already firing - ignoring trigger')
                return False
            if self.balls_remaining <= 0:
                self.get_logger().warn('No balls remaining!')
                return False
            self.is_firing = True
            self.balls_remaining -= 1

        self.get_logger().info(
            f'Firing! Balls remaining after this shot: {self.balls_remaining}'
        )

        thread = threading.Thread(target=self._fire_sequence, daemon=True)
        thread.start()
        return True

    def _fire_sequence(self):
        try:
            self.pwm.ChangeDutyCycle(CCW_DUTY)
            time.sleep(TIME_PER_REV)
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
