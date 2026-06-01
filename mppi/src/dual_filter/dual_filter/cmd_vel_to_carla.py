"""
/cmd_vel (geometry_msgs/Twist) → CARLA VehicleControl

  linear.x  → 목표 속도 → throttle / brake  (비례 제어)
  angular.z → 목표 yaw rate → 조향각        (자전거 모델 역변환)

자전거 모델:
  전진 운동학: wz = vx * tan(δ) / L
  역변환:      δ = atan2(wz * L, vx)

CARLA Python API 가 필요하므로 .venv 환경에서 실행한다.

실행 예시:
  cd ~/carla
  source .venv/bin/activate
  source /opt/ros/humble/setup.bash
  source mppi/install/setup.bash
  ros2 run dual_filter cmd_vel_to_carla \
    --ros-args -p use_sim_time:=true \
    -- --rolename car --wheelbase 1.47
"""
import argparse
import math
import time

import carla
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


_KP_SPEED = 0.8              # 속도 비례 게인: 오버슈트 발생 시 낮춘다


class CmdVelToCarla(Node):
    def __init__(self, vehicle: carla.Vehicle, wheelbase: float, max_steer_deg: float) -> None:
        super().__init__('cmd_vel_to_carla')
        self._vehicle = vehicle
        self._wheelbase = wheelbase
        self._max_steer_rad = math.radians(max_steer_deg)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.get_logger().info(
            f'cmd_vel_to_carla started | '
            f'vehicle={vehicle.type_id}  '
            f'wheelbase={wheelbase:.3f} m  '
            f'max_steer={max_steer_deg:.1f} deg'
        )

    def _cmd_vel_cb(self, msg: Twist) -> None:
        target_vx = msg.linear.x
        target_wz = msg.angular.z

        # 현재 전방 속도 (CARLA 월드 속도 → 차량 전방 방향 투영, 후진 시 음수)
        v = self._vehicle.get_velocity()
        yaw_rad = math.radians(self._vehicle.get_transform().rotation.yaw)
        current_vx = v.x * math.cos(yaw_rad) + v.y * math.sin(yaw_rad)

        # ── throttle / brake: 비례 제어 ──────────────────────────────
        err = target_vx - current_vx
        if err > 0.0:
            throttle = min(_KP_SPEED * err, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(-_KP_SPEED * err, 1.0)

        # ── 자전거 모델 역변환 ────────────────────────────────────────
        # Nav2/MPPI 표준 ROS 약속: wz > 0 = CCW = 좌회전
        # CARLA 약속:              steer > 0 = 우회전
        # 표준 자전거 모델 δ = atan2(wz*L, vx) 에서 δ > 0 = 좌회전 (표준 ROS),
        # 그러나 CARLA steer > 0 = 우회전이므로 부호를 반전해야 일치한다.
        # vx ≈ 0 이면 δ = 0 (정지 중 급선회 방지)
        if abs(target_vx) > 0.05:
            steer_rad = math.atan2(-target_wz * self._wheelbase, target_vx)
        else:
            steer_rad = 0.0

        # CARLA steer: [-1, 1] 정규화 (1.0 = 최대 조향각 방향)
        steer = max(-1.0, min(1.0, steer_rad / self._max_steer_rad))

        ctrl = carla.VehicleControl()
        ctrl.throttle = float(throttle)
        ctrl.brake = float(brake)
        ctrl.steer = float(steer)
        self._vehicle.apply_control(ctrl)
        self.get_logger().info(
            f'wz={target_wz:+.3f} steer={steer:+.3f} vx={target_vx:.2f}',
            throttle_duration_sec=0.5)


def _wait_for_vehicle(world: carla.World, role_name: str, timeout: float) -> carla.Vehicle:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for actor in world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == role_name:
                return actor
        time.sleep(0.5)
    raise RuntimeError(
        f"role_name='{role_name}' 차량을 {timeout:.0f} s 안에 찾지 못했습니다. "
        "CARLA 시뮬레이터에서 차량이 스폰되어 있는지 확인하세요."
    )


def main() -> None:
    parser = argparse.ArgumentParser(description='cmd_vel → CARLA VehicleControl 브리지')
    parser.add_argument('--host',      default='localhost',
                        help='CARLA 서버 호스트 (기본값: localhost)')
    parser.add_argument('--port',      type=int, default=2000,
                        help='CARLA 서버 포트 (기본값: 2000)')
    parser.add_argument('--rolename',  default='car',
                        help='차량 role_name (stack.json 의 id 필드, 기본값: car)')
    parser.add_argument('--wheelbase', type=float, default=1.47,
                        help='차량 축간거리 m (기본값: microlino 1.47 m)')
    parser.add_argument('--timeout',   type=float, default=30.0,
                        help='차량 탐색 제한 시간 s (기본값: 30 s)')
    # ROS 인수와 CARLA 인수가 섞이는 것을 방지: -- 이후를 ROS 인수로 취급
    args, ros_args = parser.parse_known_args()

    # CARLA 서버 연결
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    print(f"Waiting for vehicle role_name='{args.rolename}' (timeout={args.timeout:.0f} s) ...")
    vehicle = _wait_for_vehicle(world, args.rolename, args.timeout)

    # 차량 물리 정보: 전륜 최대 조향각 자동 조회
    phys = vehicle.get_physics_control()
    max_steer_deg = max(w.max_steer_angle for w in phys.wheels[:2])  # 전륜 2개(FL, FR)
    print(f"  type     : {vehicle.type_id}")
    print(f"  actor_id : {vehicle.id}")
    print(f"  wheelbase: {args.wheelbase:.3f} m  (--wheelbase 로 변경 가능)")
    print(f"  max_steer: {max_steer_deg:.1f} deg (physics_control 에서 읽음)")

    rclpy.init(args=ros_args)
    node = CmdVelToCarla(vehicle, args.wheelbase, max_steer_deg)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
