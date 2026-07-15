"""Launch the Zone Scan node and, optionally, a separate RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('auto_parking')
    rviz_config = os.path.join(
        package_share, 'rviz', 'zone_scan.rviz')
    default_zone_scan_config = os.path.join(
        package_share, 'config', 'zone_scan.yaml')
    default_gate_config = os.path.join(
        package_share, 'config', 'parking_mode_gates.yaml')
    package_root = os.path.dirname(
        os.path.dirname(os.path.realpath(__file__)))
    if not os.path.isfile(default_zone_scan_config):
        source_zone_scan_config = os.path.join(
            package_root, 'config', 'zone_scan.yaml')
        if os.path.isfile(source_zone_scan_config):
            default_zone_scan_config = source_zone_scan_config
    if not os.path.isfile(default_gate_config):
        source_gate_config = os.path.join(
            package_root, 'config', 'parking_mode_gates.yaml')
        if os.path.isfile(source_gate_config):
            default_gate_config = source_gate_config

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    zone_scan_config = LaunchConfiguration('zone_scan_config')
    gate_config = LaunchConfiguration('gate_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the CARLA simulation clock.'),
        DeclareLaunchArgument(
            'start_rviz', default_value='false',
            description='Start another RViz process using the package config.'),
        DeclareLaunchArgument(
            'zone_scan_config', default_value=default_zone_scan_config,
            description='LiDAR accumulation and ICP configuration.'),
        DeclareLaunchArgument(
            'gate_config', default_value=default_gate_config,
            description='Parking mode start/stop UTM gate configuration.'),
        Node(
            package='auto_parking',
            executable='zone_scan',
            name='zone_scan',
            output='screen',
            # Later parameter files override earlier ones. Gate coordinates
            # therefore have one authoritative source: gate_config.
            parameters=[
                zone_scan_config,
                gate_config,
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='zone_scan_rviz',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(start_rviz),
            output='screen',
        ),
    ])
