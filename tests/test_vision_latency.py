#!/usr/bin/env python3
"""Benchmark léger de latence pipeline Vision (simulation ou caméra SDK si dispo).

Skips si vision désactivée. Si YOLO/MediaPipe absents, mesure pipeline simulé.
"""

import os
import statistics
import time

import numpy as np
import pytest

from bbia_sim.bbia_vision import BBIAVision


@pytest.mark.unit
@pytest.mark.fast
def test_vision_pipeline_latency_simulated() -> None:
    if os.environ.get("BBIA_DISABLE_VISION", "0") == "1":
        pytest.skip("Vision désactivée par BBIA_DISABLE_VISION=1")

    vision = BBIAVision(robot_api=None)

    latencies_ms: list[float] = []
    # OPTIMISATION RAM: Réduire itérations (50 → 20, suffisant pour p50/p95)
    iterations = 20

    for _ in range(iterations):
        t0 = time.perf_counter()
        res = vision.scan_environment()
        assert isinstance(res, dict)
        t1 = time.perf_counter()
        latencies_ms.append((t1 - t0) * 1000.0)

    p50 = statistics.median(latencies_ms)
    p95 = float(np.percentile(latencies_ms, 95))

    # Pipeline simulé - seuils ajustés pour simulation réaliste selon matériel
    # CI peut être plus lent, tolérance augmentée
    is_ci = os.environ.get("CI", "false").lower() == "true"
    max_p50 = 200.0 if is_ci else 100.0
    max_p95 = 400.0 if is_ci else 200.0
    assert p50 < max_p50, f"p50 trop élevée: {p50:.2f} ms (max: {max_p50})"
    assert p95 < max_p95, f"p95 trop élevée: {p95:.2f} ms (max: {max_p95})"
