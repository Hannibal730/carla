"""Launch the placeholder Parking node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the CARLA simulation clock.'),
        Node(
            package='auto_parking',
            executable='parking',
            name='parking',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
