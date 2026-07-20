"""
Nav2 Controller + Planner Server Launch
========================================
controller_server (MPPI) + planner_server (SmacPlannerHybrid) +
lifecycle_manager + mode_manager 를 함께 기동한다.

lifecycle_manager 가 autostart=True 로 두 서버를 자동으로
configure → activate 전환하므로 별도의 lifecycle set 명령이 불필요하다.

모드 전환:
  - RViz → /goal_pose, auto_parking/parking → /point_parking/nav_goal
    발행 → mode_manager 수신
    → CSV 추종 취소 → planner_server 경로 계산 → MPPI 주차 기동
  - Point Parking 완료 후 고정 E까지 직선 후진, 이후 CSV 복귀

사용법:
  source /opt/ros/humble/setup.bash
  source ~/carla/mppi_ws/install/setup.bash
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

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    # controller_server + planner_server 를 함께 lifecycle 관리
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server', 'planner_server'],
        }],
    )

    # CSV 경로 추종 ↔ 주차 모드 전환 노드
    follow_path_client = Node(
        package='dual_filter',
        executable='follow_path_client',
        name='follow_path_client',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        lifecycle_manager,
        follow_path_client,
    ])
