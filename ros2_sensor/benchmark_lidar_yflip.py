#!/usr/bin/env python3
"""Benchmark: Python for-loop vs numpy vectorization for LiDAR Y-flip."""

import time
import numpy as np

# Simulate realistic point cloud sizes
SCENARIOS = [
    ("lidar_2d  (150k pts/s @ 20Hz)", 7_500),
    ("lidar 3D  (600k pts/s @ 20Hz)", 30_000),
]
ITERATIONS = 500


def make_fake_scan(n_points):
    """Create fake raw lidar data: n_points × 16 bytes."""
    return bytes(np.random.randint(0, 256, n_points * 16, dtype=np.uint8))


def method_python_loop(raw_data):
    raw = bytearray(raw_data)
    for i in range(7, len(raw), 16):
        raw[i] ^= 0x80
    return bytes(raw)


def method_numpy(raw_data):
    raw = np.frombuffer(raw_data, dtype=np.uint8).copy()
    raw[7::16] ^= 0x80
    return raw.tobytes()


def benchmark(label, n_points):
    raw_data = make_fake_scan(n_points)
    data_kb = len(raw_data) / 1024

    # Warmup
    for _ in range(10):
        method_python_loop(raw_data)
        method_numpy(raw_data)

    # Python loop
    t0 = time.perf_counter()
    for _ in range(ITERATIONS):
        method_python_loop(raw_data)
    loop_us = (time.perf_counter() - t0) / ITERATIONS * 1e6

    # numpy
    t0 = time.perf_counter()
    for _ in range(ITERATIONS):
        method_numpy(raw_data)
    numpy_us = (time.perf_counter() - t0) / ITERATIONS * 1e6

    speedup = loop_us / numpy_us

    print(f"\n{'='*55}")
    print(f"  {label}")
    print(f"  Points: {n_points:,}   Data: {data_kb:.1f} KB")
    print(f"{'='*55}")
    print(f"  Python loop : {loop_us:>8.1f} µs / scan")
    print(f"  numpy       : {numpy_us:>8.1f} µs / scan")
    print(f"  Speedup     : {speedup:>8.1f}x")
    print(f"  CPU saved @ 20Hz:")
    print(f"    loop  → {loop_us * 20 / 1e6 * 100:>5.2f}% of 1s")
    print(f"    numpy → {numpy_us * 20 / 1e6 * 100:>5.2f}% of 1s")

    # Correctness check
    assert method_python_loop(raw_data) == method_numpy(raw_data), "MISMATCH!"
    print(f"  Correctness : OK (outputs identical)")


if __name__ == "__main__":
    print(f"\nLiDAR Y-flip benchmark  ({ITERATIONS} iterations each)")
    for label, n_pts in SCENARIOS:
        benchmark(label, n_pts)
    print()
