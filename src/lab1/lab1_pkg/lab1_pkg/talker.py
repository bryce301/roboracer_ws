#!/usr/bin/env python3

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.declare_parameter('v', 0.0)
        self.declare_parameter('d', 0.0)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, 'drive', 10)

    def publish_drive(self):
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.drive.speed = self.get_parameter('v').value
        message.drive.steering_angle = self.get_parameter('d').value
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    talker = Talker()

    try:
        while rclpy.ok():
            rclpy.spin_once(talker, timeout_sec=0.0)
            talker.publish_drive()
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
