"""Launch Zone Scan, Point Parking, and Parking nodes together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('auto_parking')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    zone_scan_launch = os.path.join(
        package_share, 'launch', 'zone_scan.launch.py')
    point_parking_launch = os.path.join(
        package_share, 'launch', 'point_parking.launch.py')
    parking_launch = os.path.join(
        package_share, 'launch', 'parking.launch.py')

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(point_parking_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(parking_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
