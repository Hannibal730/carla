"""
모드 매니저 — CSV 경로 추종 ↔ 주차 모드 동적 전환
=====================================================

상태 머신:
  IDLE
    └─→ CSV_FOLLOWING  : /csv_path 수신 시 자동 진입
          └─→ PARKING  : RViz 또는 /parking 자동 브리지 goal 수신
                ├─→ FINAL_REVERSE : Zone1 완료 후 E 목표까지 직선 후진
                ├─→ EXIT_STRAIGHT : Zone1의 1차 주차 goal까지 직선 전진
                ├─→ EXIT_FORWARD  : 저장된 진입 gate 좌표까지 전진 출차
                └─→ CSV_FOLLOWING : 출차 완료 후 복귀

토픽:
  구독
    /csv_path       (nav_msgs/Path,              transient_local) — CSV 경로
    /odometry/local (nav_msgs/Odometry,          10)              — 현재 위치
    /odometry/global (nav_msgs/Odometry,         10)              — 최종 후진 map 위치
    /goal_pose      (geometry_msgs/PoseStamped,  10)              — RViz 일반 주차 목표
    /point_parking/nav_goal (geometry_msgs/PoseStamped, 10)       — 최종 후진 연결 목표
    /parking_exit/goal_utm (geometry_msgs/PoseStamped, transient) — 저장된 gate 출차 목표
    /parking_exit/zone (std_msgs/String, transient_local)          — 출차 ParkingZone

  발행
    /mode_status    (std_msgs/String,            10)              — 현재 모드 문자열
    /parking_path/smac_reverse (nav_msgs/Path, transient_local)   — Smac 후진 경로
    /parking_path/final_reverse (nav_msgs/Path, transient_local)  — 최종 직선 후진
    /parking_path/exit_forward (nav_msgs/Path, transient_local)   — gate 전진 출차 경로
    /parking_path/exit_straight (nav_msgs/Path, transient_local)  — Zone1 직선 전진 경로

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

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from nav2_msgs.action import FollowPath, ComputePathToPose


class Mode(enum.Enum):
    IDLE = 'IDLE'
    CSV_FOLLOWING = 'CSV_FOLLOWING'
    PARKING = 'PARKING'
    FINAL_REVERSE = 'FINAL_REVERSE'
    EXIT_STRAIGHT = 'EXIT_STRAIGHT'
    EXIT_FORWARD = 'EXIT_FORWARD'


class ModeManager(Node):
    def __init__(self) -> None:
        super().__init__('mode_manager')

        self.declare_parameter('datum_topic', '/utm_datum')
        self.declare_parameter(
            'point_parking_goal_topic', '/point_parking/nav_goal')
        self.declare_parameter(
            'final_reverse_odom_topic', '/odometry/global')
        self.declare_parameter('final_reverse_easting', 417069.41)
        self.declare_parameter('final_reverse_path_step', 0.2)
        self.declare_parameter(
            'parking_exit_goal_topic', '/parking_exit/goal_utm')
        self.declare_parameter(
            'parking_exit_zone_topic', '/parking_exit/zone')
        self.declare_parameter('exit_straight_path_step', 0.2)
        self.declare_parameter('parking_path_visualization_enabled', True)
        self.declare_parameter(
            'parking_path_raw_topic', '/parking_path/smac_reverse')
        self.declare_parameter(
            'parking_path_final_topic', '/parking_path/final_reverse')
        self.declare_parameter(
            'parking_path_exit_topic', '/parking_path/exit_forward')
        self.declare_parameter(
            'parking_path_exit_straight_topic',
            '/parking_path/exit_straight')

        datum_topic = str(self.get_parameter('datum_topic').value)
        point_parking_goal_topic = str(
            self.get_parameter('point_parking_goal_topic').value)
        final_reverse_odom_topic = str(
            self.get_parameter('final_reverse_odom_topic').value)
        parking_exit_goal_topic = str(
            self.get_parameter('parking_exit_goal_topic').value)
        parking_exit_zone_topic = str(
            self.get_parameter('parking_exit_zone_topic').value)
        raw_path_topic = str(
            self.get_parameter('parking_path_raw_topic').value)
        final_path_topic = str(
            self.get_parameter('parking_path_final_topic').value)
        exit_path_topic = str(
            self.get_parameter('parking_path_exit_topic').value)
        exit_straight_path_topic = str(
            self.get_parameter(
                'parking_path_exit_straight_topic').value)
        self._final_reverse_easting = float(
            self.get_parameter('final_reverse_easting').value)
        self._final_reverse_path_step = float(
            self.get_parameter('final_reverse_path_step').value)
        self._exit_straight_path_step = float(
            self.get_parameter('exit_straight_path_step').value)
        if self._final_reverse_path_step <= 0.0:
            raise ValueError('final_reverse_path_step must be greater than zero')
        if self._exit_straight_path_step <= 0.0:
            raise ValueError(
                'exit_straight_path_step must be greater than zero')

        self._parking_path_visualization_enabled = bool(
            self.get_parameter(
                'parking_path_visualization_enabled').value)

        # ── 상태 ───────────────────────────────────────────────────────────
        self._mode = Mode.IDLE
        self._csv_path: Path | None = None
        self._robot_x: float | None = None
        self._robot_y: float | None = None
        self._robot_pose = None
        self._datum_easting: float | None = None
        self._datum_northing: float | None = None
        self._exit_goal_utm: PoseStamped | None = None
        self._exit_zone = ''
        self._first_parking_goal_map: PoseStamped | None = None
        self._final_reverse_requested = False
        self._follow_goal_handle = None   # 현재 FollowPath goal handle
        self._goal_seq = 0                # 주차 목표 시퀀스 (새 목표마다 증가)
                                          #   → 취소된 옛 목표의 늦은 콜백을 걸러냄

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
        self.create_subscription(Odometry, final_reverse_odom_topic,
                                 self._global_odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose',
                                 self._goal_pose_cb, 10)
        self.create_subscription(PoseStamped, point_parking_goal_topic,
                                 self._point_parking_goal_cb, 10)
        self.create_subscription(PointStamped, datum_topic,
                                 self._datum_cb, _transient_qos)
        self.create_subscription(
            PoseStamped, parking_exit_goal_topic,
            self._parking_exit_goal_cb, _transient_qos)
        self.create_subscription(
            String, parking_exit_zone_topic,
            self._parking_exit_zone_cb, _transient_qos)

        # ── 발행 ───────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/mode_status', 10)
        self._raw_parking_path_pub = self.create_publisher(
            Path, raw_path_topic, _transient_qos)
        self._final_parking_path_pub = self.create_publisher(
            Path, final_path_topic, _transient_qos)
        self._exit_parking_path_pub = self.create_publisher(
            Path, exit_path_topic, _transient_qos)
        self._exit_straight_path_pub = self.create_publisher(
            Path, exit_straight_path_topic, _transient_qos)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            'ModeManager 시작. /csv_path 와 /odometry/local 대기 중 ...\n'
            '  RViz 또는 /parking 자동 goal 수신 → 주차 모드 전환\n'
            '  주차 경로 시각화: '
            f'{"ON" if self._parking_path_visualization_enabled else "OFF"}\n'
            f'  Zone1 Point Parking 완료 후 현재 N을 유지하며 UTM E='
            f'{self._final_reverse_easting:.2f}까지 직선 후진\n'
            '  Zone2는 2차 후진 없이 저장된 Gate B 좌표로 전진 출차'
        )

    # ── 콜백 ───────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        if self._mode == Mode.IDLE:
            self._try_start_csv()

    def _global_odom_cb(self, msg: Odometry) -> None:
        self._robot_pose = msg.pose.pose

    def _datum_cb(self, msg: PointStamped) -> None:
        self._datum_easting = float(msg.point.x)
        self._datum_northing = float(msg.point.y)

    def _parking_exit_goal_cb(self, msg: PoseStamped) -> None:
        self._exit_goal_utm = msg
        self.get_logger().info(
            '[EXIT] gate 출차 목표 수신: '
            f'E={msg.pose.position.x:.2f}, N={msg.pose.position.y:.2f}.')

    def _parking_exit_zone_cb(self, msg: String) -> None:
        self._exit_zone = msg.data
        self.get_logger().info(f'[EXIT] 출차 구역 수신: {self._exit_zone}.')

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
        """RViz 일반 주차 goal을 실행한다."""
        self._handle_goal_pose(msg, final_reverse=False)

    def _point_parking_goal_cb(self, msg: PoseStamped) -> None:
        """Point Parking goal은 zone별 출차 시퀀스를 연결한다."""
        self._first_parking_goal_map = msg
        self._handle_goal_pose(msg, final_reverse=True)

    def _handle_goal_pose(
            self, msg: PoseStamped, final_reverse: bool) -> None:
        """RViz 또는 자동 Parking goal 수신 → 주차 모드 전환.

        새 목표가 올 때마다 시퀀스를 증가시켜, 진행 중이던 목표(및 그 취소로
        인해 뒤늦게 도착하는 콜백)를 무효화하고 곧바로 새 목표로 재타겟한다.
        """
        self._goal_seq += 1
        seq = self._goal_seq
        self._final_reverse_requested = final_reverse
        self._clear_parking_path_visualizations()

        if self._mode in (
                Mode.PARKING, Mode.FINAL_REVERSE,
                Mode.EXIT_STRAIGHT, Mode.EXIT_FORWARD):
            self.get_logger().warn(
                f'주차 진행 중 새 목표 수신 — 기존 목표 취소 후 새 목표로 재타겟합니다. '
                f'(seq={seq})')
        self.get_logger().info(
            f'주차 목표 수신: '
            f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, '
            f'frame={msg.header.frame_id}, '
            f'parking_exit_sequence={final_reverse} (seq={seq})'
        )
        self._cancel_follow_then(lambda: self._start_parking(msg, seq))

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

    def _is_stale(self, seq: int) -> bool:
        """더 새로운 목표가 도착했으면 True — 이 콜백 체인은 폐기한다."""
        if seq != self._goal_seq:
            self.get_logger().info(
                f'[PARK] 옛 목표(seq={seq}) 콜백 무시 — 최신 seq={self._goal_seq}')
            return True
        return False

    def _start_parking(self, goal_pose: PoseStamped, seq: int) -> None:
        if self._is_stale(seq):
            return
        self._set_mode(Mode.PARKING)
        self.get_logger().info(
            '[PARK] 후진 전용 경로를 planner_server에 요청 중 ...')

        if not self._planner_client.server_is_ready():
            self.get_logger().error(
                '[PARK] planner_server 가 준비되지 않았습니다. '
                'controller.launch.py 에 planner_server 가 포함됐는지, '
                'lifecycle_manager 가 activate 상태인지 확인하세요.')
            self._set_mode(Mode.IDLE)
            return

        if self._robot_pose is None:
            self.get_logger().error(
                '[PARK] /odometry/global이 없어 후진 경로의 '
                '현재 위치를 설정할 수 없습니다.')
            self._resume_csv()
            return

        stamp = self.get_clock().now().to_msg()
        reverse_start = PoseStamped()
        reverse_start.header.stamp = stamp
        reverse_start.header.frame_id = 'map'
        reverse_start.pose = goal_pose.pose

        reverse_goal = PoseStamped()
        reverse_goal.header.stamp = stamp
        reverse_goal.header.frame_id = 'map'
        reverse_goal.pose = self._robot_pose

        # DUBIN은 전진 경로만 생성한다. 주차 목표 → 현재 위치로
        # 계획한 뒤 경로 순서를 뒤집으면, 실제 차량은 같은 자세를
        # 유지하며 현재 위치 → 주차 목표로 전 구간 후진한다.
        goal = ComputePathToPose.Goal()
        goal.start = reverse_start
        goal.goal = reverse_goal
        goal.planner_id = 'GridBased'
        goal.use_start = True

        future = self._planner_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_plan_goal_response(f, seq))

    def _on_plan_goal_response(self, future, seq: int) -> None:
        if self._is_stale(seq):
            return
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('[PARK] planner_server 가 goal 을 거부했습니다.')
            self._resume_csv()
            return
        self.get_logger().info('[PARK] planner_server goal 수락. 경로 계산 중 ...')
        handle.get_result_async().add_done_callback(
            lambda f: self._on_plan_result(f, seq))

    def _on_plan_result(self, future, seq: int) -> None:
        if self._is_stale(seq):
            return
        result = future.result().result
        path: Path = result.path

        if len(path.poses) == 0:
            self.get_logger().error(
                '[PARK] 주차 경로 계산 실패 (빈 경로). '
                'global_costmap 과 planner_server 설정을 확인하세요.')
            self._resume_csv()
            return

        # planner 결과는 주차 목표 → 현재 위치 전진 경로이므로
        # pose 순서를 뒤집어 현재 위치 → 주차 목표 후진 경로로 만든다.
        path.poses = list(reversed(path.poses))
        self._publish_parking_path(self._raw_parking_path_pub, path)

        self.get_logger().info(
            f'[PARK] Smac 후진 전용 경로 수신: {len(path.poses)} poses. '
            'controller_server 에 전송 중 ...')

        self._send_follow_path(
            path,
            on_accepted=lambda: self.get_logger().info('[PARK] 주차 goal 수락됨.'),
            on_result=lambda f: self._on_parking_result(f, seq),
            controller_id='ParkingPath',          # 주차 전용 MPPI (후진 전용)
            goal_checker_id='first_parking_goal_checker',  # 1차 goal 조기 완료
        )

    def _on_parking_result(self, future, seq: int) -> None:
        # 취소된 옛 목표의 result(CANCELED)가 늦게 도착해 새 목표를 덮어쓰지 않도록
        # 최신 목표의 결과일 때만 CSV 로 복귀한다.
        if self._is_stale(seq):
            return
        self._follow_goal_handle = None
        result = future.result()
        status = result.status
        error_code = getattr(result.result, 'error_code', 'N/A')
        self.get_logger().info(
            f'[PARK] 주차 완료. status={status}, error_code={error_code}')
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._final_reverse_requested:
                if self._exit_zone == 'ParkingZone2':
                    self.get_logger().info(
                        '[PARK] ParkingZone2는 2차 후진 없이 '
                        'Gate B 출차를 시작합니다.')
                    if self._start_exit_forward(seq):
                        return
                elif self._exit_zone == 'ParkingZone1':
                    if self._start_final_reverse(seq):
                        return
                else:
                    self.get_logger().error(
                        f'[PARK] 알 수 없는 출차 구역 '
                        f'"{self._exit_zone}"입니다.')
        else:
            self.get_logger().error(
                '[PARK] 주차 goal이 성공하지 않았습니다.')
        self._resume_csv()

    def _start_final_reverse(self, seq: int) -> bool:
        """Zone1에서 현재 N과 자세를 유지해 고정 UTM E까지 후진한다."""
        if self._robot_pose is None or self._datum_easting is None:
            self.get_logger().error(
                '[FINAL REVERSE] /odometry/global 또는 /utm_datum이 없어 '
                '후진 목표를 만들 수 없습니다.')
            return False

        target_x = self._final_reverse_easting - self._datum_easting
        start_x = float(self._robot_pose.position.x)
        fixed_y = float(self._robot_pose.position.y)
        delta_x = target_x - start_x
        orientation = self._robot_pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z +
                   orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y +
                         orientation.z * orientation.z),
        )

        # 목표가 현재 차량 heading의 뒤쪽인 경우에만 진행해
        # MPPI가 직선 경로를 음의 vx로 추종하게 한다.
        if delta_x * math.cos(yaw) >= -0.05:
            self.get_logger().error(
                '[FINAL REVERSE] E 목표가 현재 차량의 뒤쪽이 '
                f'아닙니다: current_map_x={start_x:.2f}, '
                f'target_map_x={target_x:.2f}, yaw={yaw:.3f}.')
            return False

        segment_count = max(
            1, int(math.ceil(abs(delta_x) / self._final_reverse_path_step)))
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = 'map'

        for index in range(segment_count + 1):
            ratio = index / segment_count
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = start_x + delta_x * ratio
            pose.pose.position.y = fixed_y
            pose.pose.position.z = self._robot_pose.position.z
            pose.pose.orientation.x = orientation.x
            pose.pose.orientation.y = orientation.y
            pose.pose.orientation.z = orientation.z
            pose.pose.orientation.w = orientation.w
            path.poses.append(pose)

        self._publish_parking_path(self._final_parking_path_pub, path)

        self._set_mode(Mode.FINAL_REVERSE)
        self.get_logger().info(
            '[FINAL REVERSE] 직선 후진 경로 전송: '
            f'UTM E={self._final_reverse_easting:.2f}, '
            f'map y={fixed_y:.2f}(현재 N 고정), '
            f'{len(path.poses)} poses.')
        sent = self._send_follow_path(
            path,
            on_accepted=lambda: self.get_logger().info(
                '[FINAL REVERSE] 후진 goal 수락됨.'),
            on_result=lambda f: self._on_final_reverse_result(f, seq),
            controller_id='ParkingPath',
            goal_checker_id='parking_goal_checker',
        )
        return sent

    def _on_final_reverse_result(self, future, seq: int) -> None:
        if self._is_stale(seq):
            return
        self._follow_goal_handle = None
        result = future.result()
        status = result.status
        error_code = getattr(result.result, 'error_code', 'N/A')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                '[FINAL REVERSE] E='
                f'{self._final_reverse_easting:.2f} 후진 완료.')
            if self._start_exit_sequence(seq):
                return
        else:
            self.get_logger().error(
                '[FINAL REVERSE] 후진 실패. '
                f'status={status}, error_code={error_code}')
        self._resume_csv()

    def _start_exit_sequence(self, seq: int) -> bool:
        if self._exit_zone == 'ParkingZone1':
            return self._start_exit_straight(seq)
        if self._exit_zone == 'ParkingZone2':
            return self._start_exit_forward(seq)
        self.get_logger().error(
            f'[EXIT] 알 수 없는 출차 구역 "{self._exit_zone}"입니다.')
        return False

    def _start_exit_straight(self, seq: int) -> bool:
        """Drive Zone1 forward to its first goal before allowing steering."""
        if self._is_stale(seq):
            return False
        if self._robot_pose is None or self._first_parking_goal_map is None:
            self.get_logger().error(
                '[EXIT STRAIGHT] 현재 pose 또는 1차 주차 goal이 없습니다.')
            return False

        start_x = float(self._robot_pose.position.x)
        start_y = float(self._robot_pose.position.y)
        target_x = float(
            self._first_parking_goal_map.pose.position.x)
        target_y = float(
            self._first_parking_goal_map.pose.position.y)
        delta_x = target_x - start_x
        delta_y = target_y - start_y
        distance = math.hypot(delta_x, delta_y)
        orientation = self._robot_pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z +
                   orientation.x * orientation.y),
            1.0 - 2.0 * (
                orientation.y * orientation.y +
                orientation.z * orientation.z),
        )
        forward_distance = (
            delta_x * math.cos(yaw) + delta_y * math.sin(yaw))
        lateral_error = abs(
            -delta_x * math.sin(yaw) + delta_y * math.cos(yaw))
        if forward_distance <= 0.05:
            self.get_logger().error(
                '[EXIT STRAIGHT] 1차 주차 goal이 현재 차량 전방에 '
                f'없습니다: forward={forward_distance:.2f} m.')
            return False

        segment_count = max(
            1, int(math.ceil(
                distance / self._exit_straight_path_step)))
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = 'map'
        for index in range(segment_count + 1):
            ratio = index / segment_count
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = start_x + delta_x * ratio
            pose.pose.position.y = start_y + delta_y * ratio
            pose.pose.position.z = self._robot_pose.position.z
            pose.pose.orientation.x = orientation.x
            pose.pose.orientation.y = orientation.y
            pose.pose.orientation.z = orientation.z
            pose.pose.orientation.w = orientation.w
            path.poses.append(pose)

        self._publish_parking_path(self._exit_straight_path_pub, path)
        self._set_mode(Mode.EXIT_STRAIGHT)
        self.get_logger().info(
            '[EXIT STRAIGHT] 조향 전 1차 주차 goal까지 직선 전진: '
            f'distance={distance:.2f} m, lateral_error={lateral_error:.2f} m, '
            f'{len(path.poses)} poses.')
        sent = self._send_follow_path(
            path,
            on_accepted=lambda: self.get_logger().info(
                '[EXIT STRAIGHT] 전진 goal 수락됨.'),
            on_result=lambda f: self._on_exit_straight_result(f, seq),
            controller_id='FollowPath',
            goal_checker_id='first_parking_goal_checker',
        )
        if not sent:
            self._resume_csv()
        return sent

    def _on_exit_straight_result(self, future, seq: int) -> None:
        if self._is_stale(seq):
            return
        self._follow_goal_handle = None
        result = future.result()
        status = result.status
        error_code = getattr(result.result, 'error_code', 'N/A')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                '[EXIT STRAIGHT] 1차 주차 goal 도달. 이제 조향을 허용합니다.')
            if self._start_exit_forward(seq):
                return
        else:
            self.get_logger().error(
                '[EXIT STRAIGHT] 직선 전진 실패. '
                f'status={status}, error_code={error_code}')
        self._resume_csv()

    def _start_exit_forward(self, seq: int) -> bool:
        """Plan a forward DUBIN path from the parked pose to saved gate."""
        if self._is_stale(seq):
            return False
        if (self._robot_pose is None or
                self._datum_easting is None or
                self._datum_northing is None or
                self._exit_goal_utm is None):
            self.get_logger().error(
                '[EXIT] 현재 pose, UTM datum 또는 저장된 gate 목표가 없어 '
                '전진 출차 경로를 만들 수 없습니다.')
            return False
        if not self._planner_client.server_is_ready():
            self.get_logger().error(
                '[EXIT] planner_server가 준비되지 않았습니다.')
            return False

        stamp = self.get_clock().now().to_msg()
        start = PoseStamped()
        start.header.stamp = stamp
        start.header.frame_id = 'map'
        start.pose = self._robot_pose

        target = PoseStamped()
        target.header.stamp = stamp
        target.header.frame_id = 'map'
        target.pose.position.x = (
            self._exit_goal_utm.pose.position.x - self._datum_easting)
        target.pose.position.y = -(
            self._exit_goal_utm.pose.position.y - self._datum_northing)
        target.pose.position.z = self._robot_pose.position.z
        exit_orientation = self._exit_goal_utm.pose.orientation
        yaw_utm = math.atan2(
            2.0 * (exit_orientation.w * exit_orientation.z +
                   exit_orientation.x * exit_orientation.y),
            1.0 - 2.0 * (
                exit_orientation.y * exit_orientation.y +
                exit_orientation.z * exit_orientation.z),
        )
        yaw_map = -yaw_utm
        target.pose.orientation.z = math.sin(yaw_map * 0.5)
        target.pose.orientation.w = math.cos(yaw_map * 0.5)

        goal = ComputePathToPose.Goal()
        goal.start = start
        goal.goal = target
        goal.planner_id = 'GridBased'
        goal.use_start = True

        self._set_mode(Mode.EXIT_FORWARD)
        self.get_logger().info(
            '[EXIT] 저장된 gate 좌표까지 전진 경로 요청: '
            f'map x={target.pose.position.x:.2f}, '
            f'y={target.pose.position.y:.2f}, yaw={yaw_map:.3f}.')
        future = self._planner_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_exit_plan_goal_response(f, seq))
        return True

    def _on_exit_plan_goal_response(self, future, seq: int) -> None:
        if self._is_stale(seq):
            return
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(
                '[EXIT] planner_server가 출차 goal을 거부했습니다.')
            self._resume_csv()
            return
        self.get_logger().info(
            '[EXIT] planner_server goal 수락. 전진 경로 계산 중 ...')
        handle.get_result_async().add_done_callback(
            lambda f: self._on_exit_plan_result(f, seq))

    def _on_exit_plan_result(self, future, seq: int) -> None:
        if self._is_stale(seq):
            return
        result = future.result().result
        path: Path = result.path
        if len(path.poses) == 0:
            self.get_logger().error('[EXIT] 전진 출차 경로 계산 실패 (빈 경로).')
            self._resume_csv()
            return

        self._publish_parking_path(self._exit_parking_path_pub, path)
        self.get_logger().info(
            f'[EXIT] 전진 출차 경로 수신: {len(path.poses)} poses. '
            'controller_server에 전송 중 ...')
        sent = self._send_follow_path(
            path,
            on_accepted=lambda: self.get_logger().info(
                '[EXIT] 전진 goal 수락됨.'),
            on_result=lambda f: self._on_exit_forward_result(f, seq),
            controller_id='FollowPath',
            goal_checker_id='goal_checker',
        )
        if not sent:
            self._resume_csv()

    def _on_exit_forward_result(self, future, seq: int) -> None:
        if self._is_stale(seq):
            return
        self._follow_goal_handle = None
        result = future.result()
        status = result.status
        error_code = getattr(result.result, 'error_code', 'N/A')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('[EXIT] gate 좌표까지 전진 출차 완료.')
        else:
            self.get_logger().error(
                '[EXIT] 전진 출차 실패. '
                f'status={status}, error_code={error_code}')
        self._resume_csv()

    # ── 공통 유틸리티 ──────────────────────────────────────────────────────

    def _publish_parking_path(self, publisher, path: Path) -> None:
        if self._parking_path_visualization_enabled:
            publisher.publish(path)

    def _clear_parking_path_visualizations(self) -> None:
        if not self._parking_path_visualization_enabled:
            return
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self._raw_parking_path_pub.publish(empty_path)
        self._final_parking_path_pub.publish(empty_path)
        self._exit_straight_path_pub.publish(empty_path)
        self._exit_parking_path_pub.publish(empty_path)

    def _send_follow_path(self, path: Path, on_accepted, on_result,
                          controller_id: str = 'FollowPath',
                          goal_checker_id: str = 'goal_checker') -> bool:
        if not self._follow_client.server_is_ready():
            self.get_logger().error(
                'controller_server/follow_path 가 준비되지 않았습니다. '
                'lifecycle_manager 가 activate 상태인지 확인하세요.')
            return False

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = controller_id
        goal.goal_checker_id = goal_checker_id

        future = self._follow_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_follow_goal_response(f, on_accepted, on_result))
        return True

    def _on_follow_goal_response(self, future, on_accepted, on_result) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('FollowPath goal 거부됨.')
            if self._mode in (
                    Mode.PARKING, Mode.FINAL_REVERSE,
                    Mode.EXIT_STRAIGHT, Mode.EXIT_FORWARD):
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
        self._final_reverse_requested = False
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
