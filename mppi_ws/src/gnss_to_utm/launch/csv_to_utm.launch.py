import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gnss_to_utm_share = get_package_share_directory('gnss_to_utm')
    params_file = os.path.join(gnss_to_utm_share, 'config', 'csv_to_utm.yaml')

    csv_to_utm_node = Node(
        package='gnss_to_utm',
        executable='csv_to_utm',
        name='csv_to_utm',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    return LaunchDescription([csv_to_utm_node])
