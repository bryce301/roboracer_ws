#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        self.ttc_threshold = self.declare_parameter('ttc_threshold', 1.0).value
        # TODO: create ROS subscribers and publishers.
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_subscription = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback,
            qos_profile_sensor_data)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ranges = np.asarray(scan_msg.ranges, dtype=float)
        angles = scan_msg.angle_min + np.arange(ranges.size) *scan_msg.angle_increment
        closing_speeds = self.speed* np.cos(angles)

        valid = (
            np.isfinite(ranges)
            & (ranges > 0.0)
            & (ranges >= scan_msg.range_min)
            & (ranges <= scan_msg.range_max)
            & np.isfinite(closing_speeds)
            & (closing_speeds > 1e-6)
        )
        ttc = np.full(ranges.shape, np.inf)
        np.divide(ranges, closing_speeds, out=ttc, where=valid)

        # TODO: publish command to brake
        if np.any(ttc < self.ttc_threshold):
            brake_msg = AckermannDriveStamped()
            brake_msg.header.stamp = self.get_clock().now().to_msg()
            brake_msg.drive.speed = 0.0
            self.drive_publisher.publish(brake_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
