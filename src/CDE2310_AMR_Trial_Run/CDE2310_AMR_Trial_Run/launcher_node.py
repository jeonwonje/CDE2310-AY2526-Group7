import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger

TIME_PER_REV = 0.87
MAX_BALLS = 3


class LauncherNode(Node):
    """Mock launcher for simulation testing (no GPIO)."""

    def __init__(self):
        super().__init__('launcher_node')

        self.balls_remaining = MAX_BALLS
        self.is_firing = False
        self.lock = threading.Lock()

        self.create_subscription(Bool, '/fire', self.fire_callback, 10)
        self.create_service(Trigger, '/fire_ball', self.service_callback)

        self.count_pub = self.create_publisher(Int32, '/balls_remaining', 10)
        self.create_timer(1.0, self.publish_count)

        self.get_logger().info(
            f'Mock launcher ready (preload simulated). Balls: {self.balls_remaining}'
        )

    def fire_callback(self, msg: Bool):
        if msg.data:
            self._attempt_fire()

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
            # 1. Fire: short CCW push
            time.sleep(TIME_PER_REV * 1.5 * 0.30)
            time.sleep(0.3)
            # 2. Retract: CW pull plunger back
            time.sleep(TIME_PER_REV * 2)
            # 3. Preload: nudge next ball into barrel
            time.sleep(0.2)
            time.sleep(TIME_PER_REV * 1.5 * 0.45)
        finally:
            with self.lock:
                self.is_firing = False

    def publish_count(self):
        msg = Int32()
        msg.data = self.balls_remaining
        self.count_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LauncherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
