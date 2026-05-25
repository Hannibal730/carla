"""
Dual Filter Architecture Launch File — REP-105
=============================================
TF tree:  map ──[global_ekf]──> odom ──[local_ekf]──> base_link

Topic remappings:
  CARLA                        →  Internal
  /carla/car/imu/data          →  /imu/data         (both EKFs)
  /carla/car/f9r/fix           →  /f9r/fix           (f9r_to_utm, azimuth_calc)
  /carla/car/f9p/fix           →  /f9p/fix           (f9p_to_utm)

Prerequisites:
  - ros2_sensor.py running with --python-ros2 flag  (publishes /wheel/odom)
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
            'gnss1_topic': '/carla/car/f9r/fix',
            'gnss2_topic': '/carla/car/f9p/fix',
            'max_time_diff_sec': 0.5,
        }],
    )

    # ------------------------------------------------------------------
    # Node 2d: UTM + azimuth → /odometry/gnss  (bridge for global EKF)
    # ------------------------------------------------------------------
    utm_to_odom = Node(
        package='dual_filter',
        executable='utm_to_odometry',
        name='utm_to_odometry',
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
        ],
    )

    # ------------------------------------------------------------------
    # Node 3: Global EKF  →  /odometry/global  +  map → odom TF
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
        ],
    )

    # ------------------------------------------------------------------
    # Node 4a: Odom path  →  /path/odom  (dead-reckoning, odom frame)
    # ------------------------------------------------------------------
    odom_path = Node(
        package='dual_filter',
        executable='odom_path_publisher',
        name='odom_path_publisher',
        output='screen',
        parameters=[sim_time_param, {
            'odom_topic': '/odometry/local',
            'path_topic': '/path/odom',
            'frame_id':   'odom',
        }],
    )

    # ------------------------------------------------------------------
    # Node 4b: GNSS path  →  /path/gnss  (raw GNSS/dual-GNSS odometry, map frame)
    # ------------------------------------------------------------------
    gnss_path = Node(
        package='dual_filter',
        executable='odom_path_publisher',
        name='gnss_path_publisher',
        output='screen',
        parameters=[sim_time_param, {
            'odom_topic': '/odometry/gnss',
            'path_topic': '/path/gnss',
            'frame_id':   'map',
        }],
    )

    # ------------------------------------------------------------------
    # Node 4c: Global EKF path  →  /path/global_ekf  (fused odometry, map frame)
    # ------------------------------------------------------------------
    global_ekf_path = Node(
        package='dual_filter',
        executable='odom_path_publisher',
        name='global_ekf_path_publisher',
        output='screen',
        parameters=[sim_time_param, {
            'odom_topic': '/odometry/global',
            'path_topic': '/path/global_ekf',
            'frame_id':   'map',
        }],
    )

    return LaunchDescription([
        f9r_to_utm,
        f9p_to_utm,
        azimuth_calc,
        utm_to_odom,
        local_ekf,
        global_ekf,
        odom_path,
        gnss_path,
        global_ekf_path,
    ])
