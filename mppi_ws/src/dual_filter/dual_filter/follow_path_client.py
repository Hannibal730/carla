"""
모드 매니저 — CSV 경로 추종 ↔ 주차 모드 동적 전환
=====================================================

상태 머신:
  IDLE
    └─→ CSV_FOLLOWING  : /csv_path 수신 시 자동 진입
          └─→ PARKING  : RViz "2D Goal Pose" 클릭(/goal_pose) 시 전환
                └─→ CSV_FOLLOWING : 주차 완료 또는 /resume_path 수신 시 복귀

토픽:
  구독
    /csv_path       (nav_msgs/Path,              transient_local) — CSV 경로
    /odometry/local (nav_msgs/Odometry,          10)              — 현재 위치
    /goal_pose      (geometry_msgs/PoseStamped,  10)              — 주차 목표
                    RViz "2D Goal Pose" 툴이 이 토픽으로 발행

  발행
    /mode_status    (std_msgs/String,            10)              — 현재 모드 문자열

액션 클라이언트:
  controller_server/follow_path         (nav2_msgs/action/FollowPath)
  planner_server/compute_path_to_pose   (nav2_msgs/action/ComputePathToPose)
"""
import enum
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from nav2_msgs.action import FollowPath, ComputePathToPose


class Mode(enum.Enum):
    IDLE = 'IDLE'
    CSV_FOLLOWING = 'CSV_FOLLOWING'
    PARKING = 'PARKING'


class ModeManager(Node):
    def __init__(self) -> None:
        super().__init__('mode_manager')

        # ── 상태 ───────────────────────────────────────────────────────────
        self._mode = Mode.IDLE
        self._csv_path: Path | None = None
        self._robot_x: float | None = None
        self._robot_y: float | None = None
        self._follow_goal_handle = None   # 현재 FollowPath goal handle

        # ── Action clients ─────────────────────────────────────────────────
        self._follow_client = ActionClient(self, FollowPath, 'follow_path')
        self._planner_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')

        # ── 구독 ───────────────────────────────────────────────────────────
        _transient_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Path, '/csv_path', self._csv_path_cb,
                                 _transient_qos)
        self.create_subscription(Odometry, '/odometry/local',
                                 self._odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose',
                                 self._goal_pose_cb, 10)

        # ── 발행 ───────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/mode_status', 10)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            'ModeManager 시작. /csv_path 와 /odometry/local 대기 중 ...\n'
            '  RViz "2D Goal Pose" 클릭 → 주차 모드 전환\n'
            '  주차 완료 후 자동으로 CSV 경로 추종 복귀'
        )

    # ── 콜백 ───────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        if self._mode == Mode.IDLE:
            self._try_start_csv()

    def _csv_path_cb(self, path: Path) -> None:
        if len(path.poses) == 0:
            self.get_logger().warn('/csv_path 가 비어 있습니다 — 무시합니다.')
            return
        self._csv_path = path
        self.get_logger().info(
            f'/csv_path 수신: {len(path.poses)} waypoints.')
        if self._mode == Mode.IDLE:
            self._try_start_csv()

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        """RViz 2D Goal Pose 클릭 → 주차 모드 전환."""
        if self._mode == Mode.PARKING:
            self.get_logger().warn(
                '이미 주차 모드입니다. 현재 주차 취소 후 새 목표로 재시작합니다.')
        self.get_logger().info(
            f'주차 목표 수신: '
            f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, '
            f'frame={msg.header.frame_id}'
        )
        self._cancel_follow_then(lambda: self._start_parking(msg))

    # ── CSV 추종 ───────────────────────────────────────────────────────────

    def _try_start_csv(self) -> None:
        if self._csv_path is None or self._robot_x is None:
            return
        if self._mode not in (Mode.IDLE,):
            return
        self._start_csv()

    def _start_csv(self) -> None:
        trimmed = self._trim_csv_to_current_pos()
        if trimmed is None:
            return

        self.get_logger().info(
            f'[CSV] FollowPath 전송: {len(trimmed.poses)} waypoints')
        self._set_mode(Mode.CSV_FOLLOWING)

        self._send_follow_path(
            trimmed,
            on_accepted=lambda: self.get_logger().info('[CSV] goal 수락됨.'),
            on_result=self._on_csv_result,
        )

    def _on_csv_result(self, future) -> None:
        result = future.result()
        status = result.status
        error_code = getattr(result.result, 'error_code', 'N/A')
        self.get_logger().info(
            f'[CSV] FollowPath 완료. status={status}, error_code={error_code}')
        if self._mode == Mode.CSV_FOLLOWING:
            self._set_mode(Mode.IDLE)

    # ── 주차 ───────────────────────────────────────────────────────────────

    def _start_parking(self, goal_pose: PoseStamped) -> None:
        self._set_mode(Mode.PARKING)
        self.get_logger().info('[PARK] planner_server 에 경로 요청 중 ...')

        if not self._planner_client.server_is_ready():
            self.get_logger().error(
                '[PARK] planner_server 가 준비되지 않았습니다. '
                'controller.launch.py 에 planner_server 가 포함됐는지, '
                'lifecycle_manager 가 activate 상태인지 확인하세요.')
            self._set_mode(Mode.IDLE)
            return

        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        goal.planner_id = 'GridBased'
        goal.use_start = False          # 현재 로봇 위치를 시작점으로 사용

        future = self._planner_client.send_goal_async(goal)
        future.add_done_callback(self._on_plan_goal_response)

    def _on_plan_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('[PARK] planner_server 가 goal 을 거부했습니다.')
            self._resume_csv()
            return
        self.get_logger().info('[PARK] planner_server goal 수락. 경로 계산 중 ...')
        handle.get_result_async().add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future) -> None:
        result = future.result().result
        path: Path = result.path

        if len(path.poses) == 0:
            self.get_logger().error(
                '[PARK] 주차 경로 계산 실패 (빈 경로). '
                'global_costmap 과 planner_server 설정을 확인하세요.')
            self._resume_csv()
            return

        self.get_logger().info(
            f'[PARK] 주차 경로 수신: {len(path.poses)} poses. '
            'controller_server 에 전송 중 ...')

        self._send_follow_path(
            path,
            on_accepted=lambda: self.get_logger().info('[PARK] 주차 goal 수락됨.'),
            on_result=self._on_parking_result,
            controller_id='ParkingPath',   # 후진 전용 MPPI 플러그인
        )

    def _on_parking_result(self, future) -> None:
        result = future.result()
        status = result.status
        error_code = getattr(result.result, 'error_code', 'N/A')
        self.get_logger().info(
            f'[PARK] 주차 완료. status={status}, error_code={error_code}')
        self._resume_csv()

    # ── 공통 유틸리티 ──────────────────────────────────────────────────────

    def _send_follow_path(self, path: Path, on_accepted, on_result,
                          controller_id: str = 'FollowPath') -> None:
        if not self._follow_client.server_is_ready():
            self.get_logger().error(
                'controller_server/follow_path 가 준비되지 않았습니다. '
                'lifecycle_manager 가 activate 상태인지 확인하세요.')
            return

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = controller_id

        future = self._follow_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_follow_goal_response(f, on_accepted, on_result))

    def _on_follow_goal_response(self, future, on_accepted, on_result) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('FollowPath goal 거부됨.')
            if self._mode == Mode.PARKING:
                self._resume_csv()
            else:
                self._set_mode(Mode.IDLE)
            return
        self._follow_goal_handle = handle
        on_accepted()
        handle.get_result_async().add_done_callback(on_result)

    def _cancel_follow_then(self, callback) -> None:
        """현재 FollowPath goal 을 취소한 뒤 callback 실행."""
        handle = self._follow_goal_handle
        if handle is None:
            callback()
            return

        self.get_logger().info('현재 FollowPath goal 취소 중 ...')
        cancel_future = handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: callback())
        self._follow_goal_handle = None

    def _resume_csv(self) -> None:
        """주차 완료/실패 후 CSV 경로 추종으로 복귀."""
        self._set_mode(Mode.IDLE)
        if self._csv_path is None or self._robot_x is None:
            self.get_logger().warn('[RESUME] csv_path 또는 odometry 없음. IDLE 대기.')
            return
        self.get_logger().info('[RESUME] CSV 경로 추종으로 복귀.')
        self._start_csv()

    def _trim_csv_to_current_pos(self) -> Path | None:
        """현재 위치에서 가장 가까운 waypoint 이후로 경로 트리밍."""
        if self._csv_path is None or self._robot_x is None:
            return None

        rx, ry = self._robot_x, self._robot_y
        closest_idx = min(
            range(len(self._csv_path.poses)),
            key=lambda i: math.hypot(
                self._csv_path.poses[i].pose.position.x - rx,
                self._csv_path.poses[i].pose.position.y - ry,
            )
        )
        dist = math.hypot(
            self._csv_path.poses[closest_idx].pose.position.x - rx,
            self._csv_path.poses[closest_idx].pose.position.y - ry,
        )
        self.get_logger().info(
            f'[CSV] 가장 가까운 waypoint: index={closest_idx}, 거리={dist:.2f} m')

        trimmed = Path()
        trimmed.header = self._csv_path.header
        trimmed.poses = self._csv_path.poses[closest_idx:]

        if len(trimmed.poses) == 0:
            self.get_logger().error('[CSV] 트리밍 후 경로가 비어 있습니다.')
            return None
        return trimmed

    def _set_mode(self, mode: Mode) -> None:
        if self._mode != mode:
            self.get_logger().info(f'모드 전환: {self._mode.value} → {mode.value}')
        self._mode = mode

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._mode.value
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
