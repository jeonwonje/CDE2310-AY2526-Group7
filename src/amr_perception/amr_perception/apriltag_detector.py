#!/usr/bin/env python3
"""
AprilTag detector node for CDE2310 warehouse mission.

Subscribes to RPi camera images, detects tag36h11 AprilTag markers,
computes their 6-DOF pose via solvePnP, transforms to map frame,
and publishes marker ID + pose for the mission controller.
"""

import json
import math

import numpy as np
import cv2
import apriltag

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('apriltag_detector')

        # ── ROS parameters ──────────────────────────────
        self.declare_parameter('marker_size', 0.16)  # metres (tag side length)
        self.declare_parameter('detection_rate', 10.0)  # Hz throttle
        # Default RPi Camera V2 intrinsics (override with calibration)
        self.declare_parameter('fx', 620.0)
        self.declare_parameter('fy', 620.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])

        self.marker_size = self.get_parameter('marker_size').value
        rate = self.get_parameter('detection_rate').value
        self.min_interval = 1.0 / rate if rate > 0 else 0.0

        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value
        self.camera_matrix = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.dist_coeffs = np.array(
            self.get_parameter('dist_coeffs').value, dtype=np.float64)

        # Use calibration from /camera_info if available
        self.camera_info_received = False
        self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self._camera_info_cb, 10)

        # ── AprilTag detector ────────────────────────────
        self.detector = apriltag.Detector(
            options=apriltag.DetectorOptions(families='tag36h11'))

        # ── CV bridge ────────────────────────────────────
        self.bridge = CvBridge()

        # ── TF ───────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publishers ───────────────────────────────────
        self.marker_pub = self.create_publisher(
            String, '/marker_detection', 10)
        self.debug_img_pub = self.create_publisher(
            Image, '/apriltag_debug_image', 1)

        # ── Subscriber ───────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw',
            self._image_cb, 10)

        # ── Throttle state ───────────────────────────────
        self.last_detect_time = 0.0

        # ── 3D object points for solvePnP ────────────────
        # Tag corners in tag frame (centred at tag, z=0)
        half = self.marker_size / 2.0
        self.obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        self.get_logger().info(
            f'AprilTag detector ready (marker_size={self.marker_size}m, '
            f'rate={rate}Hz)')

    def _camera_info_cb(self, msg):
        """Update camera intrinsics from calibration topic."""
        if not self.camera_info_received:
            K = msg.k
            self.camera_matrix = np.array([
                [K[0], K[1], K[2]],
                [K[3], K[4], K[5]],
                [K[6], K[7], K[8]],
            ], dtype=np.float64)
            if len(msg.d) >= 5:
                self.dist_coeffs = np.array(msg.d[:5], dtype=np.float64)
            self.camera_info_received = True
            self.get_logger().info('Camera intrinsics updated from /camera_info')

    def _image_cb(self, msg):
        """Process incoming camera image for AprilTag detection."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_detect_time < self.min_interval:
            return
        self.last_detect_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        for det in detections:
            tag_id = det.tag_id
            corners = det.corners  # 4x2 array

            # ── solvePnP for 6-DOF pose ──────────────────
            img_points = corners.astype(np.float64)
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, img_points,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)

            if not success:
                continue

            distance = float(np.linalg.norm(tvec))

            # ── Build pose in camera_optical_frame ────────
            camera_pose = PoseStamped()
            camera_pose.header.frame_id = 'camera_optical_frame'
            camera_pose.header.stamp = self.get_clock().now().to_msg()
            camera_pose.pose.position.x = float(tvec[0][0])
            camera_pose.pose.position.y = float(tvec[1][0])
            camera_pose.pose.position.z = float(tvec[2][0])

            # Convert rvec to quaternion
            rot_mat, _ = cv2.Rodrigues(rvec)
            qw, qx, qy, qz = self._rotation_matrix_to_quat(rot_mat)
            camera_pose.pose.orientation.x = qx
            camera_pose.pose.orientation.y = qy
            camera_pose.pose.orientation.z = qz
            camera_pose.pose.orientation.w = qw

            # ── Transform to map frame ────────────────────
            map_pose = self._transform_to_map(camera_pose)

            # ── Publish detection ─────────────────────────
            detection_msg = String()
            det_data = {
                'tag_id': tag_id,
                'distance': round(distance, 3),
                'camera_pose': {
                    'x': round(float(tvec[0][0]), 4),
                    'y': round(float(tvec[1][0]), 4),
                    'z': round(float(tvec[2][0]), 4),
                },
            }
            if map_pose is not None:
                det_data['map_pose'] = {
                    'x': round(map_pose.pose.position.x, 4),
                    'y': round(map_pose.pose.position.y, 4),
                    'z': round(map_pose.pose.position.z, 4),
                }
            detection_msg.data = json.dumps(det_data)
            self.marker_pub.publish(detection_msg)

            self.get_logger().info(
                f'Detected tag {tag_id} at distance {distance:.2f}m')

            # ── Annotate debug image ──────────────────────
            self._draw_detection(cv_image, det, distance)

        # ── Publish debug image ───────────────────────────
        if self.debug_img_pub.get_subscription_count() > 0:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                self.debug_img_pub.publish(debug_msg)
            except Exception:
                pass

    def _transform_to_map(self, camera_pose):
        """Transform a pose from camera_optical_frame to map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_optical_frame',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        # Manual transform application
        from tf2_geometry_msgs import do_transform_pose_stamped
        try:
            map_pose = do_transform_pose_stamped(camera_pose, transform)
            return map_pose
        except Exception:
            return None

    def _draw_detection(self, image, detection, distance):
        """Draw tag outline and ID on image."""
        corners = detection.corners.astype(int)
        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(image, pt1, pt2, (0, 255, 0), 2)

        center = detection.center.astype(int)
        cv2.putText(image, f'ID:{detection.tag_id} d:{distance:.2f}m',
                     (center[0] - 40, center[1] - 15),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    @staticmethod
    def _rotation_matrix_to_quat(R):
        """Convert 3x3 rotation matrix to quaternion (w, x, y, z)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return w, x, y, z


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
