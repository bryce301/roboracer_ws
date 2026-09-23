from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            parameters=[{
                'v': 2.0,
                'd': 0.25,
            }],
        ),
        Node(
            package='lab1_pkg',
            executable='relay',
            name='relay',
        ),
    ])
