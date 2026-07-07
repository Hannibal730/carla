"""
/f9p_utm (PointStamped)  +  /azimuth_angle (Float64, degrees, geo N=0 CW+)
→  /odometry/gnss (Odometry, CARLA-aligned utm frame)

Conversion:
  yaw_enu = π/2 − bearing_deg × π/180
  yaw_ros = −yaw_enu           ← CARLA +Y=right → ROS +Y=left 보정
  x_ros   =  (easting  − datum_easting)
  y_ros   = −(northing − datum_northing)  ← 동일 보정

f9p GNSS 센서는 stack.json 에서 spawn_point x=0,y=0 으로 설정돼 있어
후륜축(= base_link 원점)에 정확히 위치한다. 별도 오프셋 보정 불필요.
(f9r은 전방 x=1.4 — heading 벡터 끝점으로만 사용)

datum: 최초 f9p 수신 시의 UTM 좌표를 래치. 이 점이 utm 프레임 (0,0)이 된다.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class GnssToOdom(Node):
    def __init__(self):
        super().__init__('gnss_to_odom')

        self._azimuth_deg: float | None = None  # geographic bearing, degrees, N=0 CW+
        self._datum_x: float | None = None      # first UTM easting  (utm 원점)
        self._datum_y: float | None = None      # first UTM northing (utm 원점)

        # 위치 기준 = 후륜축 센서 (센서 위치 교체 후 f9p가 후륜축 x=0)
        self.create_subscription(PointStamped, '/f9p_utm', self._utm_cb, 10)
        self.create_subscription(Float64, '/azimuth_angle', self._azimuth_cb, 10)
        self._pub = self.create_publisher(Odometry, '/odometry/gnss', 10)

        # transient_local: 늦게 시작하는 csv_to_utm 에게도 datum 재전송 보장
        _datum_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._datum_pub = self.create_publisher(PointStamped, '/utm_datum', _datum_qos)

        self.get_logger().info('gnss_to_odom node started.')

    def _azimuth_cb(self, msg: Float64) -> None:
        self._azimuth_deg = msg.data

    def _utm_cb(self, msg: PointStamped) -> None:
        if self._azimuth_deg is None:
            self.get_logger().warn(
                'Waiting for /azimuth_angle …', throttle_duration_sec=5.0)
            return

        # Latch first fix as datum so utm frame starts at (0, 0)
        if self._datum_x is None:
            self._datum_x = msg.point.x
            self._datum_y = msg.point.y
            self.get_logger().info(
                f'UTM datum set: easting={self._datum_x:.2f}, '
                f'northing={self._datum_y:.2f}')
            # /utm_datum 발행 (transient_local) → csv_to_utm 이 언제 시작해도 수신 보장
            datum_msg = PointStamped()
            datum_msg.header.stamp = msg.header.stamp
            datum_msg.header.frame_id = 'utm'
            datum_msg.point.x = self._datum_x
            datum_msg.point.y = self._datum_y
            self._datum_pub.publish(datum_msg)

        # Geographic bearing (N=0, CW+, deg) → ENU yaw → ROS yaw (부호 반전)
        # yaw_enu = π/2 - azimuth_rad
        # yaw_ros = -yaw_enu  (CARLA +Y=right → ROS +Y=left 미러링)
        yaw = -math.radians(90.0 - self._azimuth_deg)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # normalise to [-π, π]

        q_z = math.sin(yaw / 2.0)
        q_w = math.cos(yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'utm'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x =  (msg.point.x - self._datum_x)
        odom.pose.pose.position.y = -(msg.point.y - self._datum_y)   # CARLA +Y=right 보정
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w

        # Covariance (row-major 6×6): [x, y, z, roll, pitch, yaw]
        # RTK position ~0.01 m², dual-GNSS yaw ~0.001 rad²; unused → high value
        cov = [0.0] * 36
        cov[0]  = 0.01    # xx
        cov[7]  = 0.01    # yy
        cov[14] = 1e9     # zz  (unused)
        cov[21] = 1e9     # roll (unused)
        cov[28] = 1e9     # pitch (unused)
        cov[35] = 0.05    # yaw: dual-GNSS 1.4m baseline → ~13° σ. Tighter values
                          # cause EKF to over-trust GNSS heading during turns.
        odom.pose.covariance = cov

        self._pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GnssToOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
