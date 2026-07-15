"""Forward the selected parking goal to Nav2 when parking mode starts."""

import math

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool


class Parking(Node):
    """Convert the selected UTM goal to map and trigger Nav2 once per mode."""

    def __init__(self) -> None:
        super().__init__('parking')

        self.declare_parameter('parking_mode_topic', '/parkingMode')
        self.declare_parameter(
            'point_goal_topic', '/point_parking/goal_pose')
        self.declare_parameter(
            'point_goal_valid_topic', '/point_parking/goal_valid')
        self.declare_parameter('datum_topic', '/utm_datum')
        self.declare_parameter('output_goal_topic', '/goal_pose')
        self.declare_parameter('utm_frame_id', 'utm')
        self.declare_parameter('map_frame_id', 'map')

        parking_mode_topic = str(
            self.get_parameter('parking_mode_topic').value)
        point_goal_topic = str(
            self.get_parameter('point_goal_topic').value)
        point_goal_valid_topic = str(
            self.get_parameter('point_goal_valid_topic').value)
        datum_topic = str(self.get_parameter('datum_topic').value)
        output_goal_topic = str(
            self.get_parameter('output_goal_topic').value)
        self._utm_frame_id = str(
            self.get_parameter('utm_frame_id').value).lstrip('/')
        self._map_frame_id = str(
            self.get_parameter('map_frame_id').value).lstrip('/')

        self._parking_mode = False
        self._goal_valid = False
        self._point_goal = None
        self._datum = None
        self._sent_for_current_mode = False
        self._waiting_reason = None

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ModeManager subscribes to /goal_pose with reliable volatile QoS.
        self._goal_publisher = self.create_publisher(
            PoseStamped, output_goal_topic, 10)
        self.create_subscription(
            Bool, parking_mode_topic, self._on_parking_mode, latched_qos)
        self.create_subscription(
            Bool, point_goal_valid_topic, self._on_goal_valid, latched_qos)
        self.create_subscription(
            PoseStamped, point_goal_topic, self._on_point_goal, latched_qos)
        self.create_subscription(
            PointStamped, datum_topic, self._on_datum, latched_qos)

        self.get_logger().info(
            'Parking goal bridge started: '
            f'{parking_mode_topic}=true -> {point_goal_topic} -> '
            f'{output_goal_topic}.')

    def _on_parking_mode(self, message: Bool) -> None:
        enabled = bool(message.data)

        if not enabled:
            if self._parking_mode:
                self.get_logger().info(
                    '/parkingMode=false: 다음 주차 목표 전송을 재무장합니다.')
            self._parking_mode = False
            self._sent_for_current_mode = False
            self._waiting_reason = None
            return

        if not self._parking_mode:
            self.get_logger().info(
                '/parkingMode=true: Point Parking goal 전송을 준비합니다.')
        self._parking_mode = True
        self._try_publish_goal()

    def _on_goal_valid(self, message: Bool) -> None:
        self._goal_valid = bool(message.data)
        if not self._goal_valid:
            # Point Parking은 후보가 사라졌을 때 빈 Pose를 발행하지 않으므로
            # 이전 latched pose가 다음 parkingMode에서 재사용되지 않게 지운다.
            self._point_goal = None
            return
        self._try_publish_goal()

    def _on_point_goal(self, message: PoseStamped) -> None:
        self._point_goal = message
        self._try_publish_goal()

    def _on_datum(self, message: PointStamped) -> None:
        self._datum = (float(message.point.x), float(message.point.y))
        self._try_publish_goal()

    def _try_publish_goal(self) -> None:
        if not self._parking_mode or self._sent_for_current_mode:
            return
        if not self._goal_valid:
            self._log_wait_once(
                'valid', '유효한 /point_parking goal을 기다립니다.')
            return
        if self._point_goal is None:
            self._log_wait_once(
                'pose', '/point_parking/goal_pose를 기다립니다.')
            return

        map_goal = self._to_map_goal(self._point_goal)
        if map_goal is None:
            return

        self._goal_publisher.publish(map_goal)
        self._sent_for_current_mode = True
        self._waiting_reason = None
        self.get_logger().info(
            '[PARKING GOAL] /goal_pose 전송 완료: '
            f'x={map_goal.pose.position.x:.2f}, '
            f'y={map_goal.pose.position.y:.2f}, '
            f'frame={map_goal.header.frame_id}. '
            'planner_server 경로 계산 후 ParkingPath MPPI로 전달됩니다.')

    def _to_map_goal(self, goal: PoseStamped):
        source_frame = goal.header.frame_id.lstrip('/')
        result = PoseStamped()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = self._map_frame_id

        if source_frame == self._map_frame_id:
            result.pose = goal.pose
            return result

        if source_frame != self._utm_frame_id:
            self._log_wait_once(
                'frame',
                f'지원하지 않는 goal frame "{goal.header.frame_id}"입니다. '
                f'"{self._utm_frame_id}" 또는 "{self._map_frame_id}"가 필요합니다.',
                error=True,
            )
            return None

        if self._datum is None:
            self._log_wait_once('datum', '/utm_datum을 기다립니다.')
            return None

        datum_easting, datum_northing = self._datum
        result.pose.position.x = goal.pose.position.x - datum_easting
        result.pose.position.y = -(goal.pose.position.y - datum_northing)
        result.pose.position.z = goal.pose.position.z

        yaw_utm = self._yaw_from_quaternion(goal)
        yaw_map = -yaw_utm
        result.pose.orientation.z = math.sin(yaw_map * 0.5)
        result.pose.orientation.w = math.cos(yaw_map * 0.5)
        return result

    @staticmethod
    def _yaw_from_quaternion(goal: PoseStamped) -> float:
        orientation = goal.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z +
            orientation.x * orientation.y)
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y +
            orientation.z * orientation.z)
        return math.atan2(sin_yaw, cos_yaw)

    def _log_wait_once(
            self, reason: str, message: str, error: bool = False) -> None:
        if self._waiting_reason == reason:
            return
        self._waiting_reason = reason
        if error:
            self.get_logger().error(message)
        else:
            self.get_logger().warn(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Parking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
