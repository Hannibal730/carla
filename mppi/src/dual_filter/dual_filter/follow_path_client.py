"""
/csv_path (nav_msgs/Path, transient_local) → FollowPath action goal

csv_to_utm 노드가 발행하는 /csv_path 를 구독해 controller_server 의
FollowPath action 에 goal 로 전송한다.  action 이 수락되면 MPPI가
경로 추종을 시작하고, /cmd_vel 발행이 시작된다.

경로 트리밍:
  전송 전 로봇의 현재 위치(odom 프레임)와 가장 가까운 pose 를 찾아
  그 지점부터 잘라서 전송한다.  csv 파일이 로봇 스폰 위치보다 앞에서
  시작하거나, 이미 경로 중간에 있을 때 올바른 지점부터 추종 가능.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import FollowPath


class FollowPathClient(Node):
    def __init__(self) -> None:
        super().__init__('follow_path_client')

        self._client = ActionClient(self, FollowPath, 'follow_path')
        self._sent = False
        self._path: Path | None = None
        self._robot_x: float | None = None
        self._robot_y: float | None = None

        _path_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Path, '/csv_path', self._path_cb, _path_qos)

        # 로봇의 odom 위치 수신 (local_ekf 출력)
        self.create_subscription(Odometry, '/odometry/local', self._odom_cb, 10)

        self.get_logger().info('follow_path_client: /csv_path 와 /odometry/local 대기 중 ...')

    def _odom_cb(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._try_send()

    def _path_cb(self, path: Path) -> None:
        if len(path.poses) == 0:
            self.get_logger().warn('/csv_path 가 비어 있습니다 — 무시합니다.')
            return
        self._path = path
        self.get_logger().info(
            f'/csv_path 수신: {len(path.poses)} waypoints. '
            '로봇 위치 대기 중 ...'
        )
        self._try_send()

    def _try_send(self) -> None:
        if self._sent:
            return
        if self._path is None or self._robot_x is None:
            return

        # 로봇과 가장 가까운 pose 인덱스 탐색
        rx, ry = self._robot_x, self._robot_y
        closest_idx = min(
            range(len(self._path.poses)),
            key=lambda i: math.hypot(
                self._path.poses[i].pose.position.x - rx,
                self._path.poses[i].pose.position.y - ry,
            )
        )
        dist = math.hypot(
            self._path.poses[closest_idx].pose.position.x - rx,
            self._path.poses[closest_idx].pose.position.y - ry,
        )
        self.get_logger().info(
            f'가장 가까운 waypoint: index={closest_idx}, '
            f'거리={dist:.2f} m → 이 지점부터 경로 전송'
        )

        trimmed = Path()
        trimmed.header = self._path.header
        trimmed.poses = self._path.poses[closest_idx:]

        if len(trimmed.poses) == 0:
            self.get_logger().error('트리밍 후 경로가 비어 있습니다.')
            return

        self.get_logger().info(
            f'트리밍된 경로: {len(trimmed.poses)} waypoints. '
            'FollowPath action 서버 대기 중 ...'
        )

        while not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                'FollowPath action 서버 대기 중 ... '
                '(controller_server lifecycle: active 상태가 될 때까지 재시도)'
            )

        goal = FollowPath.Goal()
        goal.path = trimmed
        goal.controller_id = 'FollowPath'

        future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)
        self._sent = True

    def _goal_response_cb(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(
                'FollowPath goal 이 controller_server 에 의해 거부됐습니다. '
                'TF tree 와 costmap 초기화 상태를 확인하세요.'
            )
            self._sent = False
            return
        self.get_logger().info('FollowPath goal 수락됨. MPPI 경로 추종 시작.')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        pass

    def _result_cb(self, future) -> None:
        result = future.result()
        self.get_logger().info(
            f'FollowPath 완료. status={result.status}  '
            '(4=성공, 5=취소, 6=중단)'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowPathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
