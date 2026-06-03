"""
mppi_speed_calc — MPPI 속도 파라미터 계산기

원하는 최대 속도(vx_max, m/s)를 입력하면
nav2_carla_params.yaml 에 반영해야 할 파라미터를 자동 계산한다.

모듈로 import:
    from dual_filter.mppi_speed_calc import calculate, print_report, to_yaml_snippet
    params = calculate(7.0)
    print_report(params)
    print(to_yaml_snippet(params))

CLI 실행 (단일 속도):
    python -m dual_filter.mppi_speed_calc 7.0

CLI 실행 (여러 속도 비교표):
    python -m dual_filter.mppi_speed_calc 3 5 7 10 13
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass

# ── 차량 / 시스템 고정 상수 ───────────────────────────────────────────────────
#   실차 계측 후 아래 값을 교체할 것.
_MIN_TURNING_R   = 3.3    # m    microlino 최소 회전반경
_CONTROLLER_FREQ = 10.0   # Hz   controller_server 주파수 (model_dt 와 연동)
_MODEL_DT        = 1.0 / _CONTROLLER_FREQ   # s   = 0.1 s
_TIME_STEPS      = 40     # MPPI 예측 단계 수 (time_steps)
_LIDAR_RANGE     = 20.0   # m    lidar_2d 하드웨어 최대 감지 거리


# ── 파라미터 데이터 클래스 ────────────────────────────────────────────────────

@dataclass
class ControllerParams:
    vx_max:              float  # m/s    최대 전진 속도
    vx_min:              float  # m/s    최대 후진 속도
    wz_max:              float  # rad/s  최대 yaw 속도
    ax_max:              float  # m/s²   최대 전방 가속도
    ax_min:              float  # m/s²   최대 감속도 (음수)
    az_max:              float  # rad/s² 최대 yaw 가속도
    vx_std:              float  # m/s    속도 탐색 노이즈 σ
    wz_std:              float  # rad/s  조향 탐색 노이즈 σ
    temperature:         float  # MPPI softmax λ
    batch_size:          int    # 샘플 궤적 수
    prediction_distance: float  # m      예측 수평선 거리 (참고용, 변경 불필요)


@dataclass
class CostmapParams:
    width:              int    # m
    height:             int    # m
    raytrace_max_range: float  # m  (센서 하드웨어 한계로 고정)
    obstacle_max_range: float  # m


@dataclass
class CriticParams:
    path_follow_offset: int  # PathFollowCritic.offset_from_furthest
    path_align_offset:  int  # PathAlignCritic.offset_from_furthest
    path_angle_offset:  int  # PathAngleCritic.offset_from_furthest


@dataclass
class MPPISpeedParams:
    target_speed_ms: float
    controller:      ControllerParams
    costmap:         CostmapParams
    critics:         CriticParams


# ── 내부 헬퍼 ────────────────────────────────────────────────────────────────

def _ceil_to(value: float, unit: int) -> int:
    """value 를 unit 의 배수로 올림."""
    return math.ceil(value / unit) * unit


# ── 계산 공식 ─────────────────────────────────────────────────────────────────

def calculate(
    vx_max: float,
    *,
    vx_min: float = -0.5,
    min_turning_r: float = _MIN_TURNING_R,
    lidar_range: float = _LIDAR_RANGE,
) -> MPPISpeedParams:
    """
    vx_max 기준으로 MPPI 파라미터 전체를 계산해 반환한다.

    Parameters
    ----------
    vx_max        : 목표 최대 속도 (m/s, 양수)
    vx_min        : 최대 후진 속도 (m/s, 음수). 기본값 -0.5.
    min_turning_r : 최소 회전반경 (m). 실측 후 교체 필요. 기본값 3.3.
    lidar_range   : LiDAR 하드웨어 최대 감지 거리 (m). 기본값 20.0.

    Returns
    -------
    MPPISpeedParams
    """
    if vx_max <= 0:
        raise ValueError(f"vx_max 는 양수여야 합니다: {vx_max}")

    # 기준 속도(5 m/s) 대비 증가분 — 선형 스케일 공식의 기준점
    delta = max(0.0, vx_max - 5.0)

    # ── 예측 거리 ─────────────────────────────────────────────────────────────
    # 직진 최대 이동 거리 = time_steps × model_dt × vx_max
    prediction_dist = _TIME_STEPS * _MODEL_DT * vx_max

    # ── 운동학 제약 ───────────────────────────────────────────────────────────
    wz_max = vx_max / min_turning_r   # 최대 yaw 속도: v / r
    az_max = wz_max                   # yaw 가속도 상한 ≈ wz_max (현 설정 기준)

    # ── 종방향 가속 / 감속 ───────────────────────────────────────────────────
    # 5 m/s 기준 ax_max=2.0. 증가분의 30%씩 선형 증가. 상한 5.0 m/s².
    ax_max = min(2.0 + delta * 0.30, 5.0)
    # 감속도(절댓값): 5 m/s 기준 3.0. 증가분의 20%씩 증가. 상한 6.0 m/s².
    ax_min = -min(3.0 + delta * 0.20, 6.0)

    # ── 탐색 노이즈 ───────────────────────────────────────────────────────────
    # vx_std: 목표 속도의 6%. 최솟값 0.3 m/s.
    vx_std = max(0.3, vx_max * 0.06)
    # wz_std: 조향 노이즈는 속도와 무관하게 고정.
    wz_std = 0.3

    # ── MPPI temperature ─────────────────────────────────────────────────────
    # 빠를수록 최저 비용 궤적에 집중(낮은 값 = 공격적).
    # 5 m/s → 0.30, 1 m/s 증가마다 0.01 감소. 하한 0.15.
    temperature = max(0.15, 0.30 - delta * 0.01)

    # ── batch_size ────────────────────────────────────────────────────────────
    # 빠를수록 탐색 다양성 확보. 5 m/s→1000, 1 m/s 증가마다 +100. 상한 2000.
    batch_size = min(2000, int(1000 + delta * 100))

    # ── costmap 크기 ──────────────────────────────────────────────────────────
    # 예측 거리(= 4 × vx_max)를 10 m 단위로 올림. 최솟값 20 m.
    # 근거: 5 m/s → pred=20m → costmap 20m 가 현재 동작 기준.
    costmap_size = max(20, _ceil_to(int(prediction_dist), 10))

    # ── LiDAR 관측 범위 ───────────────────────────────────────────────────────
    # 센서 하드웨어(lidar_range)에 종속되므로 속도와 무관하게 고정.
    # raytrace: 하드웨어 범위 - 2 m 여유.
    # obstacle: raytrace - 3 m (포인트 노이즈 마진).
    raytrace_max = round(lidar_range - 2.0, 1)
    obstacle_max = round(lidar_range * 0.75, 1)

    # ── Critics offset ────────────────────────────────────────────────────────
    # PathFollowCritic: 빠를수록 더 먼 waypoint 추종. 5~15 클리핑.
    path_follow_offset = int(min(15, max(5, round(vx_max))))
    # PathAlignCritic: 고속(10 m/s~)에서 더 먼 경로 방향 정렬 선행. 20~30 클리핑.
    path_align_offset = int(min(30, max(20, round(vx_max * 2.0))))
    # PathAngleCritic: 빠를수록 선행 각도 정렬 강화. 4~8 클리핑.
    path_angle_offset = int(min(8, max(4, round(vx_max * 0.6))))

    return MPPISpeedParams(
        target_speed_ms=round(vx_max, 2),
        controller=ControllerParams(
            vx_max=round(vx_max, 2),
            vx_min=vx_min,
            wz_max=round(wz_max, 2),
            ax_max=round(ax_max, 1),
            ax_min=round(ax_min, 1),
            az_max=round(az_max, 2),
            vx_std=round(vx_std, 2),
            wz_std=wz_std,
            temperature=round(temperature, 2),
            batch_size=batch_size,
            prediction_distance=round(prediction_dist, 1),
        ),
        costmap=CostmapParams(
            width=costmap_size,
            height=costmap_size,
            raytrace_max_range=raytrace_max,
            obstacle_max_range=obstacle_max,
        ),
        critics=CriticParams(
            path_follow_offset=path_follow_offset,
            path_align_offset=path_align_offset,
            path_angle_offset=path_angle_offset,
        ),
    )


# ── 출력 유틸리티 ─────────────────────────────────────────────────────────────

def print_report(params: MPPISpeedParams) -> None:
    """계산 결과를 출력한다. 필수 항목만 표시."""
    c  = params.controller
    m  = params.costmap
    cr = params.critics

    w = 34
    sep = "=" * 52

    print(sep)
    print(f"  MPPI 파라미터 — {c.vx_max} m/s  ({c.vx_max * 3.6:.1f} km/h)")
    print(sep)

    print("\n[필수 변경]")
    print(f"  {'vx_max':<{w}}: {c.vx_max} m/s")
    print(f"  {'wz_max':<{w}}: {c.wz_max} rad/s  (= {c.vx_max} / {_MIN_TURNING_R})")
    print(f"  {'ax_max':<{w}}: {c.ax_max} m/s²")
    print(f"  {'costmap width / height':<{w}}: {m.width} m")
    print(f"  {'PathFollowCritic.offset_from_furthest':<{w}}: {cr.path_follow_offset}")

    print(sep)


def compare(speeds: list[float]) -> None:
    """여러 속도에 대한 필수 파라미터를 비교표로 출력한다."""
    results = [calculate(v) for v in speeds]

    col     = 10
    label_w = 28
    sep     = "-" * (label_w + col * len(results))

    def row(label: str, values) -> str:
        return f"{label:<{label_w}}" + "".join(f"{v:>{col}}" for v in values)

    print("\n" + "=" * (label_w + col * len(results)))
    print(" MPPI 필수 파라미터 비교표")
    print("=" * (label_w + col * len(results)))
    print(f"{'속도':<{label_w}}" + "".join(
        f"{r.target_speed_ms:>{col}.1f}" for r in results
    ))
    print(f"{'(m/s)':<{label_w}}" + "".join(
        f"{'('+str(round(r.target_speed_ms*3.6))+'km/h)':>{col}}" for r in results
    ))
    print(sep)

    c_list  = [r.controller for r in results]
    m_list  = [r.costmap    for r in results]
    cr_list = [r.critics    for r in results]

    print(row("vx_max (m/s)",       [f"{c.vx_max:.1f}"  for c in c_list]))
    print(row("wz_max (rad/s)",     [f"{c.wz_max:.2f}"  for c in c_list]))
    print(row("ax_max (m/s²)",      [f"{c.ax_max:.1f}"  for c in c_list]))
    print(row("costmap 크기 (m)",   [str(m.width)        for m in m_list]))
    print(row("PathFollow offset",  [str(cr.path_follow_offset) for cr in cr_list]))
    print("=" * (label_w + col * len(results)))


def to_yaml_snippet(params: MPPISpeedParams) -> str:
    """nav2_carla_params.yaml 에 바로 붙여넣을 수 있는 YAML 스니펫을 반환한다."""
    c  = params.controller
    m  = params.costmap
    cr = params.critics

    return f"""\
# vx_max = {c.vx_max} m/s ({c.vx_max * 3.6:.1f} km/h)
      vx_max:  {c.vx_max}
      wz_max:  {c.wz_max}         # = {c.vx_max} / min_turning_r
      ax_max:  {c.ax_max}

      PathFollowCritic:
        offset_from_furthest: {cr.path_follow_offset}

# local_costmap
      width:  {m.width}
      height: {m.height}
"""


# ── CLI 진입점 ────────────────────────────────────────────────────────────────

def main() -> None:
    args = sys.argv[1:]

    if not args:
        # 인자 없으면 기본 비교표 출력
        compare([3.0, 5.0, 7.0, 10.0, 13.0])
        return

    try:
        speeds = [float(a) for a in args]
    except ValueError:
        print(f"오류: 속도(m/s)를 숫자로 입력하세요.  예) {sys.argv[0]} 7.0",
              file=sys.stderr)
        sys.exit(1)

    if len(speeds) == 1:
        result = calculate(speeds[0])
        print_report(result)
        print("\n── YAML 스니펫 ──────────────────────────────────────────────────")
        print(to_yaml_snippet(result))
    else:
        compare(speeds)


if __name__ == "__main__":
    main()
