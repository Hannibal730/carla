"""
Nav2 Controller Server Launch
==============================
controller_server (MPPI) + lifecycle_manager 을 함께 기동한다.
lifecycle_manager 가 autostart=True 로 controller_server 를 자동으로
configure → activate 전환하므로 별도의 lifecycle set 명령이 불필요하다.

사용법:
  source /opt/ros/humble/setup.bash
  source ~/carla/mppi/install/setup.bash
  ros2 launch dual_filter controller.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('dual_filter'),
        'config', 'nav2_carla_params.yaml',
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server'],
        }],
    )

    return LaunchDescription([
        controller_server,
        lifecycle_manager,
    ])
