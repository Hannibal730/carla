"""Launch the Point Parking gap detector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('auto_parking')
    default_config = os.path.join(
        package_share, 'config', 'point_parking.yaml')
    if not os.path.isfile(default_config):
        package_root = os.path.dirname(
            os.path.dirname(os.path.realpath(__file__)))
        source_config = os.path.join(
            package_root, 'config', 'point_parking.yaml')
        if os.path.isfile(source_config):
            default_config = source_config

    use_sim_time = LaunchConfiguration('use_sim_time')
    point_parking_config = LaunchConfiguration('point_parking_config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the CARLA simulation clock.'),
        DeclareLaunchArgument(
            'point_parking_config', default_value=default_config,
            description='Point Parking gap-detection configuration.'),
        Node(
            package='auto_parking',
            executable='point_parking',
            name='point_parking',
            output='screen',
            parameters=[
                point_parking_config,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
