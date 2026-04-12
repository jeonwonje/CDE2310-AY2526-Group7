import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

SERVO_PIN = 12
TIME_PER_REV = 0.87
CCW = 10.0

class ShooterService(Node):
    def __init__(self):
        super().__init__('shooter_service')
        self.srv = self.create_service(Trigger, 'shoot', self.shoot_callback)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, 50)
        self.pwm.start(0)
        
        self.get_logger().info('Shooter service ready')

    def shoot_callback(self, request, response):
        try:
            self.get_logger().info('Firing sequence started')

            # "setup" for firing - this will not launch 
            self.fire_round()

            self.get_logger().info('Round 1/3')
            self.fire_round()
            time.sleep(4)

            self.get_logger().info('Round 2/3')
            self.fire_round()
            time.sleep(6)

            self.get_logger().info('Round 3/3')
            self.fire_round()

            self.get_logger().info('Done!')
            response.success = True
            response.message = 'All 3 rounds fired successfully'

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def fire_round(self):
        self.pwm.ChangeDutyCycle(CCW)
        time.sleep(TIME_PER_REV)
        self.pwm.ChangeDutyCycle(0)

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ShooterService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()