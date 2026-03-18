#!/usr/bin/env python3
"""
Delivery sequencer node for CDE2310 warehouse mission.

Handles ball delivery mechanism via serial communication to OpenCR/Arduino.
Subscribes to delivery commands, sends serial commands, publishes status.

NOTE: This node runs on the RPi (serial connection to OpenCR).
      Adjust serial port and protocol to match your firmware.
"""

import json
import time

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class DeliverySequencer(Node):

    def __init__(self):
        super().__init__('delivery_sequencer')

        # ── ROS parameters ──────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('ack_timeout', 5.0)  # seconds
        self.declare_parameter('drop_delay', 2.0)  # seconds between balls

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.ack_timeout = self.get_parameter('ack_timeout').value
        self.drop_delay = self.get_parameter('drop_delay').value

        # ── Serial connection ────────────────────────────
        self.ser = None
        self._open_serial()

        # ── State ────────────────────────────────────────
        self.delivering = False
        self.balls_delivered = 0

        # ── Publishers ───────────────────────────────────
        self.status_pub = self.create_publisher(
            String, '/delivery_status', 10)

        # ── Subscribers ──────────────────────────────────
        self.create_subscription(
            Int32, '/delivery_cmd',
            self._delivery_cmd_cb, 10)

        # Single ball drop command (for dynamic delivery)
        self.create_subscription(
            Int32, '/delivery_drop_single',
            self._drop_single_cb, 10)

        self.get_logger().info(
            f'Delivery sequencer ready (port={self.serial_port})')

    def _open_serial(self):
        """Open serial connection to OpenCR/Arduino."""
        try:
            self.ser = serial.Serial(
                self.serial_port, self.baud_rate, timeout=1.0)
            time.sleep(2.0)  # wait for Arduino reset
            self.get_logger().info(f'Serial opened: {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(
                f'Cannot open serial {self.serial_port}: {e}')
            self.ser = None

    def _publish_status(self, status, ball_num=0, detail=''):
        """Publish delivery status as JSON."""
        msg = String()
        msg.data = json.dumps({
            'status': status,
            'ball_num': ball_num,
            'balls_delivered': self.balls_delivered,
            'detail': detail,
        })
        self.status_pub.publish(msg)

    def _send_serial_cmd(self, cmd):
        """Send command string over serial and wait for ACK."""
        if self.ser is None:
            self.get_logger().error('Serial not connected')
            return False

        try:
            self.ser.write(f'{cmd}\n'.encode())
            self.ser.flush()
            self.get_logger().info(f'Sent serial: {cmd}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            return False

        # Wait for ACK
        start = time.time()
        while time.time() - start < self.ack_timeout:
            try:
                if self.ser.in_waiting > 0:
                    response = self.ser.readline().decode().strip()
                    if 'ACK' in response or 'OK' in response:
                        self.get_logger().info(f'ACK received: {response}')
                        return True
                    elif 'ERR' in response or 'FAIL' in response:
                        self.get_logger().warn(f'Error response: {response}')
                        return False
            except serial.SerialException:
                break
            time.sleep(0.05)

        self.get_logger().warn(f'ACK timeout for command: {cmd}')
        return True  # proceed anyway — some firmware doesn't ACK

    def _delivery_cmd_cb(self, msg):
        """
        Handle delivery command for N balls.
        msg.data = number of balls to deliver (typically 3).
        """
        num_balls = msg.data
        if self.delivering:
            self.get_logger().warn('Already delivering, ignoring command')
            return

        self.delivering = True
        self._publish_status('starting', detail=f'Dropping {num_balls} balls')
        self.get_logger().info(f'Starting delivery of {num_balls} balls')

        success_count = 0
        for i in range(1, num_balls + 1):
            cmd = f'DROP{i}'
            self._publish_status('dropping', ball_num=i)

            ok = self._send_serial_cmd(cmd)
            if ok:
                success_count += 1
                self.balls_delivered += 1
                self._publish_status('dropped', ball_num=i)
            else:
                self._publish_status('drop_failed', ball_num=i)

            # Delay between balls (except after last)
            if i < num_balls:
                time.sleep(self.drop_delay)

        status = 'complete' if success_count == num_balls else 'partial'
        self._publish_status(status,
                             detail=f'{success_count}/{num_balls} delivered')
        self.get_logger().info(
            f'Delivery done: {success_count}/{num_balls} balls')
        self.delivering = False

    def _drop_single_cb(self, msg):
        """Drop a single ball (for dynamic station B tracking)."""
        ball_num = msg.data
        cmd = f'DROP{ball_num}'
        self._publish_status('dropping', ball_num=ball_num)

        ok = self._send_serial_cmd(cmd)
        if ok:
            self.balls_delivered += 1
            self._publish_status('dropped', ball_num=ball_num)
        else:
            self._publish_status('drop_failed', ball_num=ball_num)

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DeliverySequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
