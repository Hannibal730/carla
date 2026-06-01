#!/usr/bin/env python3
"""
CSV 경로 스플라인 보간 + 등간격 재샘플링

용도: GNSS로 녹화한 sparse CSV (예: 1~3 m 간격)를 cubic spline으로 보간하여
     지정된 간격(기본 0.1 m = 10 cm)의 dense CSV로 저장.
     저장된 파일을 csv_to_utm 노드에 그대로 사용할 수 있다.

입력 CSV 형식:
  X(E/m),Y(N/m)           ← 헤더 행 (있어도 없어도 무관)
  easting_1,northing_1
  easting_2,northing_2
  ...

출력 CSV: 동일 형식, 등간격으로 재샘플링됨

사용 예:
  # .venv 불필요 — 시스템 Python3 + scipy 로 실행 가능
  python3 interpolate_path.py input.csv output_10cm.csv
  python3 interpolate_path.py input.csv output_50cm.csv --interval 0.5
  python3 interpolate_path.py input.csv output.csv --interval 0.1 --plot

의존성: numpy, scipy
  pip install numpy scipy           # 필요 시
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline


# ─────────────────────────────────────────────────────────────────────────────

def load_csv(path: str) -> np.ndarray:
    """UTM CSV → (N, 2) ndarray [easting, northing]"""
    pts = []
    with open(path, newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                pts.append((float(row[0]), float(row[1])))
            except ValueError:
                continue  # 헤더 행 또는 파싱 불가 행 스킵
    if len(pts) < 2:
        raise ValueError(f"CSV 파일에서 유효한 점이 2개 미만입니다: {path}")
    return np.array(pts)


def remove_duplicates(pts: np.ndarray, tol: float = 1e-6) -> np.ndarray:
    """인접한 중복점 제거 (스플라인 파라미터화에서 0 간격 방지)"""
    mask = np.ones(len(pts), dtype=bool)
    for i in range(1, len(pts)):
        if np.hypot(pts[i, 0] - pts[i-1, 0], pts[i, 1] - pts[i-1, 1]) < tol:
            mask[i] = False
    return pts[mask]


def arc_length_param(pts: np.ndarray) -> np.ndarray:
    """각 점의 누적 호 길이 파라미터 계산"""
    diffs = np.diff(pts, axis=0)
    seg_lens = np.hypot(diffs[:, 0], diffs[:, 1])
    s = np.concatenate([[0.0], np.cumsum(seg_lens)])
    return s


def interpolate_path(pts: np.ndarray, interval: float) -> np.ndarray:
    """
    cubic spline 보간 + 등간격 재샘플링

    Parameters
    ----------
    pts      : (N, 2) 원본 UTM 점 배열 [easting, northing]
    interval : 재샘플링 간격 (m)

    Returns
    -------
    (M, 2) 재샘플링된 UTM 점 배열
    """
    pts = remove_duplicates(pts)
    s = arc_length_param(pts)
    total_len = s[-1]

    if total_len < interval:
        raise ValueError(
            f"경로 총 길이({total_len:.3f} m)가 재샘플링 간격({interval} m)보다 짧습니다."
        )

    # cubic spline: 호 길이 파라미터 s → (easting, northing)
    cs_e = CubicSpline(s, pts[:, 0])  # easting  spline
    cs_n = CubicSpline(s, pts[:, 1])  # northing spline

    # 등간격 샘플 위치
    s_new = np.arange(0.0, total_len, interval)
    # 마지막 점 포함 (total_len 정확히 포함되도록)
    if s_new[-1] < total_len - 1e-9:
        s_new = np.append(s_new, total_len)

    e_new = cs_e(s_new)
    n_new = cs_n(s_new)

    return np.column_stack([e_new, n_new])


def save_csv(path: str, pts: np.ndarray) -> None:
    """재샘플링된 점 배열을 CSV로 저장"""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['X(E/m)', 'Y(N/m)'])
        for e, n in pts:
            writer.writerow([f"{e:.15f}", f"{n:.15f}"])


def plot_paths(original: np.ndarray, interpolated: np.ndarray) -> None:
    """원본과 보간 결과 시각화 (matplotlib 필요)"""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib 미설치 — 시각화 생략. pip install matplotlib")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    ax1.plot(original[:, 0], original[:, 1], 'b.-', ms=4, label='원본')
    ax1.plot(interpolated[:, 0], interpolated[:, 1], 'r-', lw=0.8, label='보간')
    ax1.set_aspect('equal')
    ax1.legend()
    ax1.set_title('경로 비교 (전체)')
    ax1.set_xlabel('Easting (m)')
    ax1.set_ylabel('Northing (m)')

    # 첫 50 m 구간 확대
    max_s = 50.0
    s = arc_length_param(original)
    idx = np.searchsorted(s, max_s)
    sub_orig = original[:idx+1]
    s_i = arc_length_param(interpolated)
    idx_i = np.searchsorted(s_i, max_s)
    sub_interp = interpolated[:idx_i+1]

    ax2.plot(sub_orig[:, 0], sub_orig[:, 1], 'b.-', ms=6, label='원본')
    ax2.plot(sub_interp[:, 0], sub_interp[:, 1], 'r-', lw=1.0, alpha=0.7, label='보간')
    ax2.set_aspect('equal')
    ax2.legend()
    ax2.set_title(f'처음 {max_s:.0f} m 구간 확대')
    ax2.set_xlabel('Easting (m)')

    plt.tight_layout()
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description='CSV 경로 cubic spline 보간 + 등간격 재샘플링',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('input',    help='입력 CSV 파일 경로')
    parser.add_argument('output',   help='출력 CSV 파일 경로')
    parser.add_argument('--interval', type=float, default=0.1,
                        help='재샘플링 간격 m (기본값: 0.1 = 10 cm)')
    parser.add_argument('--plot', action='store_true',
                        help='보간 결과 시각화 (matplotlib 필요)')
    args = parser.parse_args()

    if args.interval <= 0:
        print("--interval 은 양수여야 합니다.", file=sys.stderr)
        sys.exit(1)

    print(f"입력: {args.input}")
    original = load_csv(args.input)
    total_len = arc_length_param(original)[-1]
    print(f"  점 수    : {len(original)}")
    print(f"  총 길이  : {total_len:.3f} m")
    print(f"  평균 간격: {total_len / (len(original)-1):.3f} m")
    print(f"\n재샘플링 간격: {args.interval} m")

    interpolated = interpolate_path(original, args.interval)
    print(f"  보간 후 점 수: {len(interpolated)}")

    # 출력 디렉토리 생성
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    save_csv(args.output, interpolated)
    print(f"\n저장 완료: {args.output}")

    if args.plot:
        plot_paths(original, interpolated)


if __name__ == '__main__':
    main()
