"""
Dual Filter Architecture Launch File — REP-105
=============================================
TF tree:  utm ──[global_ekf]──> odom ──[local_ekf]──> base_link

Topic remappings:
  CARLA                        →  Internal
  /carla/car/imu/data          →  /imu/data         (both EKFs)
  /carla/car/f9r/fix           →  /f9r/fix           (f9r_to_utm, azimuth_calc)
  /carla/car/f9p/fix           →  /f9p/fix           (f9p_to_utm)

Prerequisites:
  - ros2_sensor.py running with --python-ros2 flag  (publishes /carla/car/wheel_encoder/data)
  - sudo apt install ros-humble-robot-localization
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('dual_filter'), 'config', 'ekf_params.yaml')
    sim_time_param = {'use_sim_time': True}

    # ------------------------------------------------------------------
    # Node 2a: f9r NavSatFix → UTM PointStamped  (/f9r_utm)
    # ------------------------------------------------------------------
    f9r_to_utm = Node(
        package='gnss_to_utm',
        executable='f9r_to_utm',
        name='f9r_to_utm',
        output='screen',
        parameters=[sim_time_param],
        remappings=[('/f9r/fix', '/carla/car/f9r/fix')],
    )

    # ------------------------------------------------------------------
    # Node 2b: f9p NavSatFix → UTM PointStamped  (/f9p_utm)
    # ------------------------------------------------------------------
    f9p_to_utm = Node(
        package='gnss_to_utm',
        executable='f9p_to_utm',
        name='f9p_to_utm',
        output='screen',
        parameters=[sim_time_param],
        remappings=[('/f9p/fix', '/carla/car/f9p/fix')],
    )

    # ------------------------------------------------------------------
    # Node 2c: Dual GNSS → azimuth heading  (/azimuth_angle, degrees)
    #   Parameters override the hardcoded topic names in the node.
    # ------------------------------------------------------------------
    azimuth_calc = Node(
        package='gnss_to_utm',
        executable='azimuth_angle_calculator_node',
        name='azimuth_angle_calculator',
        output='screen',
        parameters=[sim_time_param, {
            # gnss1 = 기준점(차량 뒤쪽 = 후륜축), gnss2 = 벡터 끝점(차량 앞쪽)
            # 센서 위치 교체 후: f9p가 후륜축(x=0), f9r이 전방(x=1.4)
            'gnss1_topic': '/carla/car/f9p/fix',
            'gnss2_topic': '/carla/car/f9r/fix',
            'max_time_diff_sec': 0.5,
        }],
    )

    # ------------------------------------------------------------------
    # Node 2d: UTM + azimuth → /odometry/gnss  (bridge for global EKF)
    # ------------------------------------------------------------------
    gnss_to_odom = Node(
        package='dual_filter',
        executable='gnss_to_odom',
        name='gnss_to_odom',
        output='screen',
        parameters=[sim_time_param],
    )

    # ------------------------------------------------------------------
    # Node 1: Local EKF  →  /odometry/local  +  odom → base_link TF
    #   /imu/data is remapped from /carla/car/imu/data
    # ------------------------------------------------------------------
    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='local_ekf',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('/imu/data', '/carla/car/imu/data'),
            ('odometry/filtered', '/odometry/local'),
            ('/wheel_encoder/data', '/carla/car/wheel_encoder/data'),
        ],
    )

    # ------------------------------------------------------------------
    # Node 3: Global EKF  →  /odometry/global  +  utm → odom TF
    #   /imu/data is remapped from /carla/car/imu/data
    # ------------------------------------------------------------------
    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='global_ekf',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('/imu/data', '/carla/car/imu/data'),
            ('odometry/filtered', '/odometry/global'),
            ('/wheel_encoder/data', '/carla/car/wheel_encoder/data'),
        ],
    )

    # ------------------------------------------------------------------
    # Node 4a: Local EKF path  →  /path/local_ekf  (dead-reckoning, odom frame)
    # ------------------------------------------------------------------
    local_ekf_path = Node(
        package='dual_filter',
        executable='path_visualizer',
        name='local_ekf_path_publisher',
        output='screen',
        parameters=[sim_time_param, {
            'odom_topic': '/odometry/local',
            'path_topic': '/path/local_ekf',
            'frame_id':   'odom',
        }],
    )

    # ------------------------------------------------------------------
    # Node 4b: GNSS path  →  /path/gnss  (raw GNSS/dual-GNSS odometry, utm frame)
    # ------------------------------------------------------------------
    gnss_path = Node(
        package='dual_filter',
        executable='path_visualizer',
        name='gnss_path_publisher',
        output='screen',
        parameters=[sim_time_param, {
            'odom_topic': '/odometry/gnss',
            'path_topic': '/path/gnss',
            'frame_id':   'utm',
        }],
    )

    # ------------------------------------------------------------------
    # Node 4c: Global EKF path  →  /path/global_ekf  (fused odometry, utm frame)
    # ------------------------------------------------------------------
    global_ekf_path = Node(
        package='dual_filter',
        executable='path_visualizer',
        name='global_ekf_path_publisher',
        output='screen',
        parameters=[sim_time_param, {
            'odom_topic': '/odometry/global',
            'path_topic': '/path/global_ekf',
            'frame_id':   'utm',
        }],
    )

    return LaunchDescription([
        f9r_to_utm,
        f9p_to_utm,
        azimuth_calc,
        gnss_to_odom,
        local_ekf,
        global_ekf,
        local_ekf_path,
        gnss_path,
        global_ekf_path,
    ])
