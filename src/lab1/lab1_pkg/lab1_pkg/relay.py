#!/usr/bin/env python3

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node


class Relay(Node):

    def __init__(self):
        super().__init__('relay')
        self.publisher = self.create_publisher(
            AckermannDriveStamped, 'drive_relay', 10)
        self.subscription = self.create_subscription(
            AckermannDriveStamped, 'drive', self.relay_drive, 10)

    def relay_drive(self, message):
        relayed_message = AckermannDriveStamped()
        relayed_message.header = message.header
        relayed_message.drive.speed = message.drive.speed * 3.0
        relayed_message.drive.steering_angle = (
            message.drive.steering_angle * 3.0)
        self.publisher.publish(relayed_message)


def main(args=None):
    rclpy.init(args=args)
    relay = Relay()

    try:
        rclpy.spin(relay)
    except KeyboardInterrupt:
        pass
    finally:
        relay.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
