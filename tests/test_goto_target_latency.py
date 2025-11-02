#!/usr/bin/env python3
"""
Benchmark latence goto_target() en simulation (p50/p95) avec petite matrice pose.
"""

import statistics
import time

# removed deprecated typing.List import
import numpy as np
import numpy.typing as npt
import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


def _small_pose() -> npt.NDArray[np.float64]:
    pose = np.eye(4, dtype=np.float64)
    # petite rotation pour éviter le no-op
    angle = 0.05
    c, s = np.cos(angle), np.sin(angle)
    pose[0, 0] = c
    pose[0, 1] = -s
    pose[1, 0] = s
    pose[1, 1] = c
    return pose


@pytest.mark.unit
@pytest.mark.fast
def test_goto_target_latency_simulation() -> None:
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    head = _small_pose()
    iterations = 50
    latencies_ms: list[float] = []

    try:
        for _ in range(iterations):
            t0 = time.perf_counter()
            backend.goto_target(head=head, duration=0.1, method="minjerk", body_yaw=0.0)
            t1 = time.perf_counter()
            latencies_ms.append((t1 - t0) * 1000.0)

        p50 = statistics.median(latencies_ms)
        p95 = float(np.percentile(latencies_ms, 95))

        # Budget: l’appel wrapper doit être rapide en simulation (< 20 ms)
        assert p50 < 20.0, f"p50 trop élevée: {p50:.2f} ms"
        assert p95 < 40.0, f"p95 trop élevée: {p95:.2f} ms"
    finally:
        backend.disconnect()
