#!/usr/bin/env python3
"""
Test latence set_joint_pos / get_joint_pos sur RobotAPI (N=1e3 appels).
Mesure p50/p95 pour validation performance interface abstraite.
"""

import statistics
import time

import numpy as np
import pytest

from bbia_sim.robot_factory import RobotFactory


@pytest.mark.unit
@pytest.mark.slow
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (500 itérations)
def test_robot_api_set_joint_pos_latency_1e3() -> None:
    """Test latence set_joint_pos RobotAPI sur 500 appels (optimisé)."""
    robot = RobotFactory.create_backend("reachy_mini")
    if not robot or not robot.connect():
        pytest.skip("Backend non disponible")

    assert robot is not None  # Type narrowing pour mypy
    # OPTIMISATION: Réduire 500 → 300 itérations (suffisant pour statistiques p50/p95, 1.7x plus rapide)
    iterations = 300
    joint = "yaw_body"
    latencies_ms: list[float] = []

    try:
        for i in range(iterations):
            # Position variée
            angle = 0.1 * np.sin(i * 0.01)
            t0 = time.perf_counter()
            robot.set_joint_pos(joint, angle)
            t1 = time.perf_counter()
            latencies_ms.append((t1 - t0) * 1000.0)

        p50 = statistics.median(latencies_ms)
        p95 = float(np.percentile(latencies_ms, 95))

        # Budget: Interface abstraite doit avoir overhead minimal (< 10ms p95)
        assert p50 < 5.0, f"p50 trop élevée: {p50:.2f} ms"
        assert p95 < 10.0, f"p95 trop élevée: {p95:.2f} ms"
    finally:
        robot.disconnect()


@pytest.mark.unit
@pytest.mark.slow
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (500 itérations)
def test_robot_api_get_joint_pos_latency_1e3() -> None:
    """Test latence get_joint_pos RobotAPI sur 500 appels (optimisé)."""
    robot = RobotFactory.create_backend("reachy_mini")
    if not robot or not robot.connect():
        pytest.skip("Backend non disponible")

    assert robot is not None  # Type narrowing pour mypy
    # OPTIMISATION: Réduire 500 → 300 itérations (suffisant pour statistiques p50/p95, 1.7x plus rapide)
    iterations = 300
    joint = "yaw_body"
    latencies_ms: list[float] = []

    try:
        for _ in range(iterations):
            t0 = time.perf_counter()
            pos = robot.get_joint_pos(joint)
            t1 = time.perf_counter()
            assert pos is not None
            latencies_ms.append((t1 - t0) * 1000.0)

        p50 = statistics.median(latencies_ms)
        p95 = float(np.percentile(latencies_ms, 95))

        # Budget: Interface abstraite doit avoir overhead minimal (< 10ms p95)
        assert p50 < 5.0, f"p50 trop élevée: {p50:.2f} ms"
        assert p95 < 10.0, f"p95 trop élevée: {p95:.2f} ms"
    finally:
        robot.disconnect()
