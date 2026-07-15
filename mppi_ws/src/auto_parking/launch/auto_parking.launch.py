"""Launch Zone Scan, Point Parking, and Parking nodes together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('auto_parking')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    zone_scan_launch = os.path.join(
        package_share, 'launch', 'zone_scan.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the CARLA simulation clock.'),
        DeclareLaunchArgument(
            'start_rviz', default_value='false',
            description='Start RViz with the Zone Scan configuration.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zone_scan_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_rviz': start_rviz,
            }.items(),
        ),
        Node(
            package='auto_parking',
            executable='point_parking',
            name='point_parking',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='auto_parking',
            executable='parking',
            name='parking',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
