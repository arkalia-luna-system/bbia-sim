#!/usr/bin/env python3
"""Benchmarks Vision: FPS 10s et budget CPU/RAM 10s (simulation).

Skips si vision désactivée. Mesure réalisée sans modèles lourds.
"""

import os
import statistics
import time

import pytest

from bbia_sim.bbia_vision import BBIAVision


@pytest.mark.unit
@pytest.mark.fast
def test_vision_fps_10s_simulated() -> None:
    if os.environ.get("BBIA_DISABLE_VISION", "0") == "1":
        pytest.skip("Vision désactivée par BBIA_DISABLE_VISION=1")

    vision = BBIAVision(robot_api=None)

    duration_s = 10.0
    frames = 0
    latencies_ms: list[float] = []

    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duration_s:
        t_start = time.perf_counter()
        _ = vision.scan_environment()
        t_end = time.perf_counter()
        frames += 1
        latencies_ms.append((t_end - t_start) * 1000.0)

    elapsed = time.perf_counter() - t0
    fps = frames / elapsed if elapsed > 0 else 0.0

    # Cible CPU: ≥ 10 FPS en simulation
    assert fps >= 10.0, f"FPS trop bas: {fps:.2f}"

    # Latence pipeline simulé rapide
    p50 = statistics.median(latencies_ms)
    latencies_ms.sort()
    p95 = latencies_ms[int(0.95 * len(latencies_ms)) - 1] if latencies_ms else 0.0
    assert p50 < 10.0 and p95 < 25.0, f"Latence élevée: p50={p50:.2f}ms p95={p95:.2f}ms"


@pytest.mark.unit
@pytest.mark.fast
def test_vision_budget_cpu_ram_10s() -> None:
    if os.environ.get("BBIA_DISABLE_VISION", "0") == "1":
        pytest.skip("Vision désactivée par BBIA_DISABLE_VISION=1")

    import tracemalloc

    vision = BBIAVision(robot_api=None)

    duration_s = 10.0
    tracemalloc.start()
    t0_cpu = time.process_time()
    t0 = time.perf_counter()

    while time.perf_counter() - t0 < duration_s:
        _ = vision.scan_environment()
        time.sleep(0.01)

    current, peak = tracemalloc.get_traced_memory()
    t1_cpu = time.process_time()
    tracemalloc.stop()

    cpu_time_s = t1_cpu - t0_cpu

    # Budgets larges CI/macOS
    assert peak < 200 * 1024 * 1024, f"Peak RAM trop élevé: {peak}B"
    assert cpu_time_s < 3.0, f"Temps CPU trop élevé: {cpu_time_s:.2f}s/10s"
