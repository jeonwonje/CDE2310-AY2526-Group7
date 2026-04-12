import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Bool

TARGET_TAG_ID = 3

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        self.fire_pub = self.create_publisher(Bool, '/fire', 10)
        self.on_cooldown = False
        self.cooldown_seconds = 2.0
        self.cooldown_timer = None
        self.get_logger().info('AprilTag detector running, watching for tag ID 3...')

    def detection_callback(self, msg: AprilTagDetectionArray):
        if self.on_cooldown:
            return
        for detection in msg.detections:
            if detection.id == TARGET_TAG_ID:
                self.get_logger().info(f'Tag ID {TARGET_TAG_ID} detected! Sending fire signal')
                fire_msg = Bool()
                fire_msg.data = True
                self.fire_pub.publish(fire_msg)
                self._start_cooldown()
                break

    def _start_cooldown(self):
        self.on_cooldown = True
        self.cooldown_timer = self.create_timer(self.cooldown_seconds, self._end_cooldown)

    def _end_cooldown(self):
        self.on_cooldown = False
        self.cooldown_timer.cancel()
        self.destroy_timer(self.cooldown_timer)
        self.cooldown_timer = None

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()