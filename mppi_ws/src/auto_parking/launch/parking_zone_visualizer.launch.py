"""Compatibility launch alias for the renamed Zone Scan node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    launch_file = os.path.join(
        get_package_share_directory('auto_parking'),
        'launch',
        'zone_scan.launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
    ])
