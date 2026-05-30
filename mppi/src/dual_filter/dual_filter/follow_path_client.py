"""
/csv_path (nav_msgs/Path, transient_local) → FollowPath action goal

csv_to_utm 노드가 발행하는 /csv_path 를 구독해 controller_server 의
FollowPath action 에 goal 로 전송한다.  action 이 수락되면 MPPI가
경로 추종을 시작하고, /cmd_vel 발행이 시작된다.

QoS 주의:
  csv_to_utm 이 transient_local RELIABLE KeepLast(1) 로 발행하므로
  이 노드도 동일 QoS 로 구독해야 늦게 시작해도 경로를 수신할 수 있다.

경로 재전송:
  기본 동작은 경로를 한 번만 전송한다 (self._sent 플래그).
  경로를 다시 전송하려면 노드를 재시작하면 된다.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


class FollowPathClient(Node):
    def __init__(self) -> None:
        super().__init__('follow_path_client')

        self._client = ActionClient(self, FollowPath, 'follow_path')
        self._sent = False

        # csv_to_utm 과 동일 QoS: transient_local RELIABLE KeepLast(1)
        # 이 설정이 없으면 이 노드가 csv_to_utm 보다 늦게 시작했을 때
        # 경로 메시지를 영원히 수신하지 못한다.
        _path_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Path, '/csv_path', self._path_cb, _path_qos)
        self.get_logger().info('follow_path_client: /csv_path 대기 중 ...')

    def _path_cb(self, path: Path) -> None:
        if self._sent:
            return
        if len(path.poses) == 0:
            self.get_logger().warn('/csv_path 가 비어 있습니다 — 무시합니다.')
            return

        self.get_logger().info(
            f'/csv_path 수신: {len(path.poses)} waypoints. '
            'FollowPath action 서버 대기 중 ...'
        )

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'FollowPath action 서버에 연결하지 못했습니다. '
                'controller_server 가 실행 중인지 확인하세요.'
            )
            return

        goal = FollowPath.Goal()
        goal.path = path
        # nav2_carla_params.yaml 의 controller_plugins 이름과 반드시 일치해야 함
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
            self._sent = False  # 재시도 허용
            return
        self.get_logger().info('FollowPath goal 수락됨. MPPI 경로 추종 시작.')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        # feedback 에는 현재 추종 중인 pose 인덱스 등이 포함됨
        # 필요 시: self.get_logger().info(str(feedback_msg.feedback))
        pass

    def _result_cb(self, future) -> None:
        result = future.result()
        self.get_logger().info(
            f'FollowPath 완료. status={result.status}  '
            '(3=성공, 4=취소, 6=중단)'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowPathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
