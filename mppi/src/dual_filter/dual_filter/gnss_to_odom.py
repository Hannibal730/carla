"""
/f9r_utm (PointStamped)  +  /azimuth_angle (Float64, degrees, geo N=0 CW+)
→  /odometry/gnss (Odometry, CARLA-aligned ROS utm frame)

Conversion:  yaw_enu = π/2 − bearing_deg × π/180
CARLA uses +Y to the vehicle's right, while ROS uses +Y left. Mirror the
UTM northing axis and yaw so the GNSS path matches the local ROS odometry
and the CARLA simulator's apparent turn direction.

The first received UTM fix is stored as the datum origin so that the
utm frame starts at (0, 0) — matching the odom frame convention used by
the local EKF.  Without this subtraction, raw UTM coordinates (~300 000 m
easting / ~4 000 000 m northing) would place the utm origin hundreds of
kilometres from the local odom origin, making the global path appear
completely offset in RViz.
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
        self._datum_x: float | None = None      # first UTM easting  (utm origin)
        self._datum_y: float | None = None      # first UTM northing (utm origin)

        self.create_subscription(PointStamped, '/f9r_utm', self._utm_cb, 10)
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
                f'UTM datum set: easting={self._datum_x:.2f}, northing={self._datum_y:.2f}')
            # /utm_datum 발행 (transient_local) → csv_to_utm 이 언제 시작해도 수신 보장
            datum_msg = PointStamped()
            datum_msg.header.stamp = msg.header.stamp
            datum_msg.header.frame_id = 'utm'
            datum_msg.point.x = self._datum_x
            datum_msg.point.y = self._datum_y
            self._datum_pub.publish(datum_msg)

        # Geographic bearing (N=0, CW+, deg) → ENU yaw, then mirror Y for
        # CARLA's left-handed coordinate convention into ROS base_link (Y-left).
        yaw = -math.radians(90.0 - self._azimuth_deg)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # normalise to [-π, π]

        q_z = math.sin(yaw / 2.0)
        q_w = math.cos(yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'utm'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = msg.point.x - self._datum_x
        odom.pose.pose.position.y = -(msg.point.y - self._datum_y)
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
