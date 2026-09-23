"""
Launch the controller for the levine_obs obstacle course.

    ros2 launch gap_follow levine_obs_launch.py

Everything this map needs goes here: one node or several, Python or C++, and
the parameter values that suit this map (levine_blocked_launch.py is the
empty loop's). Start your own nodes only: the simulator is already running.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

# 'reactive_node.py' is scripts/reactive_node.py, 'reactive_node' is the C++
# src/reactive_node.cpp: name the one you wrote
EXECUTABLE = 'reactive_node.py'

# this map's values for the parameters your node declares
# e.g. {'max_speed': 6.0} or give it a full .yaml config file
PARAMETERS = {
    'bubble_radius': 0.18,
    'clearance_half_angle': 0.4363,
    'disparity_threshold': 0.50,
    'fast_clearance': 3.0,
    'max_speed': 4.0,
    'max_deceleration': 1.5,
    'max_lateral_accel': 3.0,
    'max_steering': 0.40,
    'min_speed': 0.40,
    'planning_range': 3.0,
    'recovery_min_gap_width': 0.05,
    'safety_margin': 0.05,
    'side_clearance': 0.12,
    'simple_planning_range': 6.0,
    'simple_turn_slowdown': 0.35,
    'slow_clearance': 0.75,
    'slow_speed': 0.60,
    'stop_distance': 0.20,
    'turn_slowdown': 0.90,
    'vehicle_half_width': 0.16,
    'wheelbase': 0.33,
}


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gap_follow',
            executable=EXECUTABLE,
            output='screen',
            parameters=[PARAMETERS],
        ),
    ])
