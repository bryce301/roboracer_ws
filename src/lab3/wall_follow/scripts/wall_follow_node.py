#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.scan_sub = self.create_subscription(
            LaserScan, lidarscan_topic, self.scan_callback,
            qos_profile_sensor_data
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10
        )

        self.kp = 1.0
        self.kd = 0.1
        self.ki = 0.01
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.integral_limit = 1.0
        self.max_steering_angle = math.radians(25.0)
        self.max_speed = 5.0
        self.desired_distance = 1.0
        self.lookahead_distance = 1.0
        self.wall_beam_angle = math.radians(45.0)

    def get_range(self, scan, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        if not scan.ranges or not all(math.isfinite(value) for value in (
            angle, scan.angle_min, scan.angle_max, scan.angle_increment,
            scan.range_min, scan.range_max
        )):
            return None
        if (scan.angle_increment <= 0.0 or
                scan.angle_max < scan.angle_min or
                scan.range_min < 0.0 or scan.range_max <= scan.range_min):
            return None
        if not scan.angle_min <= angle <= scan.angle_max:
            return None

        index = int(math.floor(
            (angle - scan.angle_min) / scan.angle_increment + 0.5
        ))
        if not 0 <= index < len(scan.ranges):
            return None

        distance = float(scan.ranges[index])
        if (not math.isfinite(distance) or distance <= 0.0 or
                not scan.range_min <= distance <= scan.range_max):
            return None
        return distance

    def get_error(self, scan, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        if (not math.isfinite(dist) or dist <= 0.0 or
                not math.isfinite(self.lookahead_distance) or
                self.lookahead_distance < 0.0 or
                not 0.0 < self.wall_beam_angle < math.pi / 2):
            return None

        a = self.get_range(scan, self.wall_beam_angle)
        b = self.get_range(scan, math.pi / 2)
        if a is None or b is None:
            return None

        def sampled_angle(angle):
            index = int(math.floor(
                (angle - scan.angle_min) / scan.angle_increment + 0.5
            ))
            return scan.angle_min + index * scan.angle_increment

        angle_a = sampled_angle(self.wall_beam_angle)
        angle_b = sampled_angle(math.pi / 2)
        ax, ay = a * math.cos(angle_a), a * math.sin(angle_a)
        bx, by = b * math.cos(angle_b), b * math.sin(angle_b)
        dx, dy = ax - bx, ay - by
        if dx <= 0.0 or math.hypot(dx, dy) < 1e-6:
            return None

        alpha = math.atan2(dy, dx)
        current_distance = by * math.cos(alpha) - bx * math.sin(alpha)
        future_distance = (
            current_distance + self.lookahead_distance * math.sin(alpha)
        )
        return future_distance - dist

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        now = self.get_clock().now()
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = now.to_msg()
        if (error is None or not math.isfinite(error) or
                not math.isfinite(velocity) or velocity <= 0.0):
            self.integral = 0.0
            self.prev_error = None
            self.prev_time = None
            self.drive_pub.publish(drive_msg)
            return

        derivative = 0.0
        candidate_integral = self.integral
        if self.prev_time is not None:
            dt = (now.nanoseconds - self.prev_time) * 1e-9
            if 1e-6 < dt <= 0.5:
                derivative = (error - self.prev_error) / dt
                candidate_integral = max(-self.integral_limit, min(
                    self.integral_limit, self.integral + error * dt
                ))
            else:
                self.integral = 0.0
                candidate_integral = 0.0

        output = (self.kp * error + self.ki * candidate_integral +
                  self.kd * derivative)
        if abs(output) <= self.max_steering_angle or output * error < 0.0:
            self.integral = candidate_integral
        output = (self.kp * error + self.ki * self.integral +
                  self.kd * derivative)
        angle = max(-self.max_steering_angle, min(self.max_steering_angle, output))
        if abs(angle) < math.radians(10.0):
            speed = 5.0
        elif abs(angle) < math.radians(20.0):
            speed = 1.2
        else:
            speed = 0.5

        drive_msg.drive.steering_angle = float(angle)
        drive_msg.drive.speed = float(min(speed, velocity, self.max_speed))
        self.prev_error = error
        self.prev_time = now.nanoseconds
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        left_range = self.get_range(msg, math.pi / 2)
        left_text = 'unavailable' if left_range is None else f'{left_range:.3f} m'
        error = self.get_error(msg, self.desired_distance)
        error_text = 'unavailable' if error is None else f'{error:+.3f} m'
        self.get_logger().info(
            f'/scan: ranges={len(msg.ranges)}, '
            f'angle_min={msg.angle_min:.4f} rad, '
            f'angle_max={msg.angle_max:.4f} rad, '
            f'angle_increment={msg.angle_increment:.6f} rad, '
            f'left_range={left_text}, wall_error={error_text}',
            throttle_duration_sec=2.0
        )
        self.pid_control(error, self.max_speed)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    try:
        rclpy.spin(wall_follow_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            wall_follow_node.pid_control(None, 0.0)
        wall_follow_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
