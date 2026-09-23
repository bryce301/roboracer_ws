#!/usr/bin/python3
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ReactiveFollowGap(Node):
    """Reactive obstacle avoidance controller."""

    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.scan_sub = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, qos_profile_sensor_data
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10
        )
        self.max_range = float(self.declare_parameter('max_range', 6.0).value)
        self.front_half_angle = float(
            self.declare_parameter('front_half_angle', np.pi / 2).value
        )
        self.bubble_radius = float(
            self.declare_parameter('bubble_radius', 0.45).value
        )
        self.bubble_count = int(
            self.declare_parameter('bubble_count', 1).value
        )
        self.planning_range = float(
            self.declare_parameter('planning_range', 3.0).value
        )
        self.simple_planning_range = float(
            self.declare_parameter('simple_planning_range', 6.0).value
        )
        self.disparity_threshold = float(
            self.declare_parameter('disparity_threshold', 0.5).value
        )
        self.vehicle_half_width = float(
            self.declare_parameter('vehicle_half_width', 0.16).value
        )
        self.safety_margin = float(
            self.declare_parameter('safety_margin', 0.05).value
        )
        self.min_gap_width = float(
            self.declare_parameter('min_gap_width', 0.4).value
        )
        self.max_speed = float(self.declare_parameter('max_speed', 1.2).value)
        self.min_speed = float(self.declare_parameter('min_speed', 0.4).value)
        self.turn_slowdown = float(
            self.declare_parameter('turn_slowdown', 0.65).value
        )
        self.simple_turn_slowdown = float(
            self.declare_parameter('simple_turn_slowdown', 0.35).value
        )
        self.clearance_speed_gain = float(
            self.declare_parameter('clearance_speed_gain', 0.6).value
        )
        self.clearance_half_angle = float(
            self.declare_parameter(
                'clearance_half_angle', np.deg2rad(10)
            ).value
        )
        self.max_deceleration = float(
            self.declare_parameter('max_deceleration', 100.0).value
        )
        self.steering_time_constant = float(
            self.declare_parameter('steering_time_constant', 0.0).value
        )
        self.max_lateral_accel = float(
            self.declare_parameter('max_lateral_accel', 3.0).value
        )
        self.wheelbase = float(
            self.declare_parameter('wheelbase', 0.33).value
        )
        self.slow_clearance = float(
            self.declare_parameter('slow_clearance', 0.75).value
        )
        self.fast_clearance = float(
            self.declare_parameter('fast_clearance', 3.0).value
        )
        self.slow_speed = float(
            self.declare_parameter('slow_speed', 0.4).value
        )
        self.side_clearance = float(
            self.declare_parameter('side_clearance', 0.25).value
        )
        self.best_point_window = int(
            self.declare_parameter('best_point_window', 9).value
        )
        self.complex_obstacle_distance = float(
            self.declare_parameter('complex_obstacle_distance', 2.0).value
        )
        self.complex_close_ratio = float(
            self.declare_parameter('complex_close_ratio', 0.35).value
        )
        self.complex_min_clusters = int(
            self.declare_parameter('complex_min_clusters', 3).value
        )
        self.complex_min_disparities = int(
            self.declare_parameter('complex_min_disparities', 4).value
        )
        self.complex_min_gaps = int(
            self.declare_parameter('complex_min_gaps', 3).value
        )
        self.complex_enter_score = int(
            self.declare_parameter('complex_enter_score', 2).value
        )
        self.complex_exit_score = int(
            self.declare_parameter('complex_exit_score', 1).value
        )
        self.complex_enter_time = float(
            self.declare_parameter('complex_enter_time', 0.08).value
        )
        self.complex_exit_time = float(
            self.declare_parameter('complex_exit_time', 0.8).value
        )
        self.simple_steering_tau = float(
            self.declare_parameter('simple_steering_tau', 0.10).value
        )
        self.complex_steering_tau = float(
            self.declare_parameter('complex_steering_tau', 0.05).value
        )
        self.simple_steering_rate = float(
            self.declare_parameter('simple_steering_rate', 4.0).value
        )
        self.complex_steering_rate = float(
            self.declare_parameter('complex_steering_rate', 6.0).value
        )
        self.max_steering = float(
            self.declare_parameter('max_steering', 0.34).value
        )
        self.stop_distance = float(
            self.declare_parameter('stop_distance', 0.45).value
        )
        self.recovery_speed = float(
            self.declare_parameter('recovery_speed', 0.15).value
        )
        self.recovery_min_gap_width = float(
            self.declare_parameter('recovery_min_gap_width', 0.25).value
        )
        self.min_motion_speed = float(
            self.declare_parameter('min_motion_speed', 0.05).value
        )
        self.max_invalid_run = int(
            self.declare_parameter('max_invalid_run', 3).value
        )
        values = (self.max_range, self.front_half_angle, self.bubble_radius,
                  self.planning_range, self.simple_planning_range,
                  self.disparity_threshold,
                  self.vehicle_half_width, self.safety_margin,
                  self.min_gap_width, self.max_speed, self.min_speed,
                  self.turn_slowdown, self.simple_turn_slowdown,
                  self.clearance_speed_gain,
                  self.clearance_half_angle,
                  self.max_deceleration, self.steering_time_constant,
                  self.max_lateral_accel,
                  self.wheelbase, self.slow_clearance,
                  self.fast_clearance, self.slow_speed,
                  self.side_clearance, self.complex_obstacle_distance,
                  self.complex_close_ratio, self.complex_enter_time,
                  self.complex_exit_time, self.simple_steering_tau,
                  self.complex_steering_tau, self.simple_steering_rate,
                  self.complex_steering_rate,
                  self.max_steering, self.stop_distance,
                  self.recovery_speed, self.recovery_min_gap_width,
                  self.min_motion_speed)
        if (not np.all(np.isfinite(values)) or self.max_range <= 0
                or not 0 < self.front_half_angle <= np.pi
                or self.bubble_radius < 0
                or self.bubble_count < 1
                or not 0 < self.planning_range <= self.max_range
                or not self.planning_range <= self.simple_planning_range
                <= self.max_range
                or self.disparity_threshold < 0
                or self.vehicle_half_width <= 0 or self.safety_margin < 0
                or self.min_gap_width < 0
                or not 0 < self.min_speed <= self.max_speed
                or not 0 <= self.turn_slowdown <= 1
                or not 0 <= self.simple_turn_slowdown <= 1
                or self.clearance_speed_gain <= 0
                or not 0 < self.clearance_half_angle <= self.front_half_angle
                or self.max_deceleration <= 0
                or self.steering_time_constant < 0
                or self.max_lateral_accel <= 0 or self.wheelbase <= 0
                or not self.stop_distance < self.slow_clearance
                or not self.slow_clearance < self.fast_clearance
                or self.fast_clearance > self.max_range
                or not self.min_motion_speed <= self.slow_speed
                <= self.max_speed
                or self.side_clearance < 0
                or not 0 < self.complex_obstacle_distance <= self.max_range
                or not 0 <= self.complex_close_ratio <= 1
                or self.complex_min_clusters < 1
                or self.complex_min_disparities < 1
                or self.complex_min_gaps < 1
                or not 1 <= self.complex_enter_score <= 4
                or not 0 <= self.complex_exit_score
                < self.complex_enter_score
                or self.complex_enter_time < 0 or self.complex_exit_time < 0
                or self.simple_steering_tau < 0
                or self.complex_steering_tau < 0
                or self.simple_steering_rate <= 0
                or self.complex_steering_rate <= 0
                or self.max_steering <= 0 or self.stop_distance < 0
                or not 0 < self.recovery_speed <= self.max_speed
                or not 0 < self.recovery_min_gap_width <= self.min_gap_width
                or not 0 < self.min_motion_speed <= self.recovery_speed
                or self.max_invalid_run < 0
                or self.best_point_window < 1):
            raise ValueError('Invalid controller parameters')
        now = self.get_clock().now()
        self.last_scan_time = now
        self.last_control_time = now
        self.control_mode = 'simple'
        self.mode_candidate_since_ns = None
        self.last_steering = 0.0
        self.last_control_time_ns = None
        self.scan_watchdog = self.create_timer(0.1, self.check_scan_timeout)

    def preprocess_lidar(self, ranges):
        """Keep obstacles visible while bounding unusable and distant readings."""
        proc_ranges = np.asarray(ranges, dtype=np.float64).copy()
        proc_ranges[np.isnan(proc_ranges) | (proc_ranges <= 0)] = 0.0
        proc_ranges[np.isposinf(proc_ranges)] = self.max_range
        proc_ranges[np.isneginf(proc_ranges)] = 0.0
        np.minimum(proc_ranges, self.max_range, out=proc_ranges)
        invalid = proc_ranges <= 0
        changes = np.diff(
            np.concatenate(([False], invalid, [False])).astype(int)
        )
        starts = np.flatnonzero(changes == 1)
        ends = np.flatnonzero(changes == -1)
        for start, end in zip(starts, ends):
            if (end - start <= self.max_invalid_run and start > 0
                    and end < len(proc_ranges)):
                proc_ranges[start:end] = np.linspace(
                    proc_ranges[start - 1], proc_ranges[end],
                    end - start + 2,
                )[1:-1]
        return proc_ranges

    def apply_safety_bubble(self, ranges, angle_increment,
                            planning_limit=None):
        """Zero rays that pass within bubble_radius of the closest obstacle."""
        free_ranges = np.asarray(ranges, dtype=np.float64).copy()
        valid = np.flatnonzero(np.isfinite(free_ranges) & (free_ranges > 0))
        if valid.size == 0:
            return free_ranges

        # Baseline FTG bubble used for the verified 42--45 second laps.
        original = free_ranges.copy()
        padded = np.pad(original, 1, constant_values=np.inf)
        local_minima = valid[
            (original[valid] <= padded[valid])
            & (original[valid] <= padded[valid + 2])
        ]
        order = local_minima[np.argsort(original[local_minima])]
        covered = np.zeros(len(free_ranges), dtype=bool)
        bubbles = 0
        for closest_index in order:
            if covered[closest_index]:
                continue
            closest_distance = float(original[closest_index])
            if closest_distance >= self.max_range:
                break
            half_angle = np.arcsin(
                min(1.0, self.bubble_radius / closest_distance)
            )
            half_width = int(np.ceil(half_angle / angle_increment))
            start = max(0, int(closest_index) - half_width)
            end = min(len(free_ranges), int(closest_index) + half_width + 1)
            free_ranges[start:end] = 0.0
            covered[start:end] = True
            bubbles += 1
            if bubbles >= self.bubble_count:
                break
        return free_ranges

    def extend_disparities(self, ranges, angle_increment):
        """Mask vehicle-width arcs behind LiDAR depth discontinuities."""
        extended = np.asarray(ranges, dtype=np.float64).copy()
        if extended.size < 2:
            return extended

        original = extended.copy()
        left = original[:-1]
        right = original[1:]
        disparities = np.flatnonzero(
            (left > 0)
            & (right > 0)
            & (np.abs(left - right) >= self.disparity_threshold)
        )
        clearance_width = self.vehicle_half_width + self.safety_margin
        for index in disparities:
            if left[index] < right[index]:
                near_distance = left[index]
                direction = 1
                first_far = index + 1
            else:
                near_distance = right[index]
                direction = -1
                first_far = index
            half_angle = np.arcsin(min(1.0, clearance_width / near_distance))
            ray_count = int(np.ceil(half_angle / angle_increment))
            if direction > 0:
                start = first_far
                end = min(len(extended), first_far + ray_count)
            else:
                start = max(0, first_far - ray_count + 1)
                end = first_far + 1
            extended[start:end] = 0.0
        return extended

    @staticmethod
    def count_true_runs(mask):
        """Count contiguous true regions in a Boolean array."""
        mask = np.asarray(mask, dtype=bool)
        if mask.size == 0:
            return 0
        return int(np.count_nonzero(mask & ~np.r_[False, mask[:-1]]))

    @staticmethod
    def count_separated_indices(indices, separation=4):
        """Count groups of nearby event indices as one physical feature."""
        indices = np.asarray(indices, dtype=int)
        if indices.size == 0:
            return 0
        return 1 + int(np.count_nonzero(np.diff(indices) > separation))

    def environment_complexity(self, ranges, angles, angle_increment):
        """Return a robust obstacle-complexity score and its components."""
        front_mask = np.abs(angles) <= self.front_half_angle
        front = np.asarray(ranges[front_mask], dtype=np.float64)
        valid = front > 0
        if not np.any(valid):
            return 4, {
                'close_ratio': 1.0,
                'clusters': self.complex_min_clusters,
                'disparities': self.complex_min_disparities,
                'gaps': self.complex_min_gaps,
            }

        close = valid & (front < self.complex_obstacle_distance)
        close_ratio = float(np.count_nonzero(close) / np.count_nonzero(valid))
        clusters = self.count_true_runs(close)
        disparity_indices = np.flatnonzero(
            (front[:-1] > 0)
            & (front[1:] > 0)
            & (np.abs(np.diff(front)) >= self.disparity_threshold)
        )
        disparities = self.count_separated_indices(disparity_indices)

        planning = np.zeros_like(ranges, dtype=np.float64)
        planning[front_mask] = np.minimum(
            ranges[front_mask], self.planning_range
        )
        extended = self.extend_disparities(planning, angle_increment)
        masked = self.apply_safety_bubble(
            extended, angle_increment, self.planning_range
        )
        gaps = len(self.find_gaps(masked))
        score = sum((
            close_ratio >= self.complex_close_ratio,
            clusters >= self.complex_min_clusters,
            disparities >= self.complex_min_disparities,
            gaps >= self.complex_min_gaps,
        ))
        return int(score), {
            'close_ratio': close_ratio,
            'clusters': clusters,
            'disparities': disparities,
            'gaps': gaps,
        }

    def update_control_mode(self, score, now_ns):
        """Apply asymmetric time hysteresis to the simple/complex mode."""
        if self.control_mode == 'simple':
            evidence = score >= self.complex_enter_score
            hold_time = self.complex_enter_time
            next_mode = 'complex'
        else:
            evidence = score <= self.complex_exit_score
            hold_time = self.complex_exit_time
            next_mode = 'simple'

        if not evidence:
            self.mode_candidate_since_ns = None
            return False
        if self.mode_candidate_since_ns is None:
            self.mode_candidate_since_ns = now_ns
            return False
        elapsed = (now_ns - self.mode_candidate_since_ns) * 1e-9
        if elapsed < hold_time:
            return False
        self.control_mode = next_mode
        self.mode_candidate_since_ns = None
        self.get_logger().info(f'Control mode: {self.control_mode}')
        return True

    def find_max_gap(self, free_space_ranges):
        """Return inclusive indices of the longest positive run, or (0, -1)."""
        gaps = self.find_gaps(free_space_ranges)
        if not gaps:
            return 0, -1

        center = (len(free_space_ranges) - 1) / 2
        return max(
            gaps,
            key=lambda gap: (
                gap[1] - gap[0] + 1,
                -abs((gap[0] + gap[1]) / 2 - center),
            ),
        )

    def find_gaps(self, free_space_ranges):
        """Return inclusive index pairs for all positive runs."""
        free = np.asarray(free_space_ranges) > 0
        changes = np.diff(
            np.concatenate(([False], free, [False])).astype(int)
        )
        starts = np.flatnonzero(changes == 1)
        ends = np.flatnonzero(changes == -1)
        return [(int(start), int(end - 1))
                for start, end in zip(starts, ends)]

    @staticmethod
    def gap_width(start_i, end_i, ranges, angle_increment):
        """Estimate a gap's chord width from its median usable depth."""
        gap = np.asarray(ranges[start_i:end_i + 1], dtype=np.float64)
        if gap.size == 0:
            return 0.0
        depth = min(2.0, float(np.median(gap)))
        angle = min((end_i - start_i + 1) * angle_increment, np.pi)
        return float(2 * depth * np.sin(angle / 2))

    def find_drivable_gap(self, free_space_ranges, angle_increment):
        """Choose the physically widest gap that can fit the vehicle."""
        candidates = [
            gap for gap in self.find_gaps(free_space_ranges)
            if self.gap_width(*gap, free_space_ranges, angle_increment)
            >= self.min_gap_width
        ]
        if not candidates:
            return 0, -1
        center = (len(free_space_ranges) - 1) / 2
        return max(
            candidates,
            key=lambda gap: (
                gap[1] - gap[0] + 1,
                self.gap_width(*gap, free_space_ranges, angle_increment),
                -abs((gap[0] + gap[1]) / 2 - center),
            ),
        )

    def find_recovery_gap(self, free_space_ranges, angle_increment):
        """Choose the widest residual gap for a low-speed escape."""
        gaps = self.find_gaps(free_space_ranges)
        if not gaps:
            return 0, -1
        center = (len(free_space_ranges) - 1) / 2
        return max(
            gaps,
            key=lambda gap: (
                self.gap_width(
                    *gap, free_space_ranges, angle_increment
                ),
                gap[1] - gap[0] + 1,
                -abs((gap[0] + gap[1]) / 2 - center),
            ),
        )

    def find_best_point(self, start_i, end_i, ranges):
        """Aim at a broad, deep part of the selected gap."""
        gap = np.asarray(ranges[start_i:end_i + 1], dtype=np.float64)
        if gap.size == 0:
            return None
        window = min(21, gap.size)
        if window % 2 == 0:
            window -= 1
        smoothed = np.convolve(
            gap,
            np.ones(window) / window,
            mode='valid',
        )
        candidate_starts = np.flatnonzero(
            smoothed >= 0.98 * np.max(smoothed)
        )
        candidate_centers = candidate_starts + window // 2
        midpoint = (gap.size - 1) / 2
        center_distance = np.abs(candidate_centers - midpoint)
        central = np.flatnonzero(center_distance == np.min(center_distance))
        best_candidate = central[
            np.argmax(smoothed[candidate_starts[central]])
        ]
        best_local = candidate_centers[best_candidate]
        return start_i + int(best_local)

    @staticmethod
    def find_gap_center(start_i, end_i, ranges):
        """Return the center ray of a stable simple-mode gap."""
        if end_i < start_i or len(ranges) == 0:
            return None
        return (start_i + end_i) // 2

    def filter_steering(self, target, now_ns, complex_mode):
        """Smooth mode transitions and limit scan-to-scan steering changes."""
        if self.last_control_time_ns is None:
            self.last_control_time_ns = now_ns
            self.last_steering = float(target)
            return self.last_steering

        dt = max(0.0, (now_ns - self.last_control_time_ns) * 1e-9)
        self.last_control_time_ns = now_ns
        tau = (
            self.complex_steering_tau
            if complex_mode else self.simple_steering_tau
        )
        rate = (
            self.complex_steering_rate
            if complex_mode else self.simple_steering_rate
        )
        alpha = 1.0 if tau <= 0 else 1.0 - np.exp(-dt / tau)
        filtered_target = (
            self.last_steering + alpha * (target - self.last_steering)
        )
        max_change = rate * dt
        steering = self.last_steering + np.clip(
            filtered_target - self.last_steering,
            -max_change,
            max_change,
        )
        self.last_steering = float(np.clip(
            steering, -self.max_steering, self.max_steering
        ))
        return self.last_steering

    def apply_side_guard(self, steering, ranges, angles):
        """Stop turning toward a wall beside the car, as in lecture Tweak 3."""
        if steering > 0:
            side = ranges[angles > self.front_half_angle]
        elif steering < 0:
            side = ranges[angles < -self.front_half_angle]
        else:
            return steering, False
        valid_side = side[side > 0]
        blocked = (
            valid_side.size > 0
            and float(np.min(valid_side)) < self.side_clearance
        )
        return (0.0, True) if blocked else (steering, False)

    def calculate_speed(self, steering, clearance, recovering=False,
                        side_blocked=False, complex_mode=True):
        """Combine piecewise, braking, and lateral-acceleration speed limits."""
        if clearance <= self.stop_distance:
            return 0.0

        distance_speed = float(np.interp(
            clearance,
            [self.stop_distance, self.slow_clearance, self.fast_clearance],
            [self.min_motion_speed, self.slow_speed, self.max_speed],
        ))
        clearance_margin = clearance - self.stop_distance
        braking_speed = float(np.sqrt(
            2 * self.max_deceleration * clearance_margin
        ))
        tangent = abs(np.tan(steering))
        if tangent > 1e-6:
            cornering_speed = float(np.sqrt(
                self.max_lateral_accel * self.wheelbase / tangent
            ))
        else:
            cornering_speed = self.max_speed
        slowdown = (
            self.turn_slowdown
            if complex_mode else self.simple_turn_slowdown
        )
        turn_factor = (
            1.0
            - slowdown * abs(steering) / self.max_steering
        )
        if recovering:
            steering_speed = self.recovery_speed
        else:
            steering_speed = max(
                self.min_speed, self.max_speed * turn_factor
            )
        speed = min(
            self.max_speed,
            distance_speed,
            braking_speed,
            cornering_speed,
            steering_speed,
        )
        if side_blocked:
            speed = min(speed, self.slow_speed)
        return max(self.min_motion_speed, speed)

    def lidar_callback(self, data):
        """Process a LiDAR scan and publish a drive command."""
        if (not data.ranges or not np.isfinite(data.angle_min)
                or not np.isfinite(data.angle_increment)
                or data.angle_increment <= 0):
            self.publish_stop()
            return
        self.last_scan_time = self.get_clock().now()

        proc_ranges = self.preprocess_lidar(data.ranges)
        angles = (
            data.angle_min
            + np.arange(len(proc_ranges)) * data.angle_increment
        )
        proc_ranges[np.abs(angles) > self.front_half_angle] = 0.0
        if np.isfinite(data.range_min) and data.range_min > 0:
            proc_ranges[proc_ranges < data.range_min] = 0.0

        # Later lecture/hybrid additions are deliberately disabled here:
        # score, _ = self.environment_complexity(...)
        # self.update_control_mode(score, now.nanoseconds)
        # extended_ranges = self.extend_disparities(...)
        # steering, side_blocked = self.apply_side_guard(...)
        # steering = self.filter_steering(...)
        free_ranges = self.apply_safety_bubble(
            proc_ranges, data.angle_increment
        )
        fallback_start, fallback_end = self.find_max_gap(free_ranges)
        if fallback_end < fallback_start:
            self.publish_stop()
            return

        start_i, end_i = self.find_drivable_gap(
            free_ranges, data.angle_increment
        )
        recovering = end_i < start_i
        if recovering:
            start_i, end_i = fallback_start, fallback_end
            fallback_width = self.gap_width(
                start_i, end_i, free_ranges, data.angle_increment
            )
            if fallback_width < self.recovery_min_gap_width:
                self.publish_stop()
                return

        best_index = self.find_best_point(start_i, end_i, free_ranges)
        if best_index is None:
            self.publish_stop()
            return
        steering = float(np.clip(
            angles[best_index], -self.max_steering, self.max_steering
        ))
        now = self.get_clock().now()
        if self.steering_time_constant > 0:
            elapsed = max(
                0.0,
                (now - self.last_control_time).nanoseconds / 1e9,
            )
            alpha = 1.0 - np.exp(-elapsed / self.steering_time_constant)
            steering = float(
                self.last_steering + alpha * (steering - self.last_steering)
            )
        self.last_control_time = now
        self.last_steering = steering

        forward = proc_ranges[np.abs(angles) <= self.clearance_half_angle]
        if forward.size == 0:
            self.publish_stop()
            return
        valid_forward = forward[forward > 0]
        if valid_forward.size < max(1, len(forward) // 2):
            self.publish_stop()
            return
        forward_clearance = float(np.percentile(valid_forward, 10))
        if forward_clearance <= self.stop_distance:
            self.publish_drive(0.0, steering)
            return

        turn_factor = (
            1.0
            - self.turn_slowdown * abs(steering) / self.max_steering
        )
        preferred_speed = max(self.min_speed, self.max_speed * turn_factor)
        clearance_margin = max(
            0.0, forward_clearance - self.stop_distance
        )
        clearance_speed = min(
            self.clearance_speed_gain * clearance_margin,
            np.sqrt(2 * self.max_deceleration * clearance_margin),
        )
        if recovering:
            preferred_speed = self.recovery_speed
        speed = min(preferred_speed, clearance_speed)
        speed = max(self.min_motion_speed, speed)
        self.publish_drive(speed, steering)

    def check_scan_timeout(self):
        elapsed = (self.get_clock().now() - self.last_scan_time).nanoseconds
        if elapsed > 500_000_000:
            self.publish_stop()

    def publish_drive(self, speed, steering):
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.drive.speed = float(speed)
        drive.drive.steering_angle = float(steering)
        self.drive_pub.publish(drive)

    def publish_stop(self):
        self.publish_drive(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    reactive_node = ReactiveFollowGap()
    reactive_node.get_logger().info('Follow Gap Initialized')
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            reactive_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # A launch service may forward the same SIGINT more than once.
            pass


if __name__ == '__main__':
    main()
