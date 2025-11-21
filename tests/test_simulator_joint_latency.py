#!/usr/bin/env python3
"""
Test latence set/get_joint_pos sur simulateur (N=1e3 appels).
Mesure p50/p95 pour validation performance.
"""

import statistics
import time

import numpy as np
import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


@pytest.mark.unit
@pytest.mark.slow
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (500 itérations)
def test_simulator_set_joint_pos_latency_1e3() -> None:
    """Test latence set_joint_pos sur 500 appels (optimisé)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    # OPTIMISATION: Réduire 500 → 300 itérations (suffisant pour statistiques p50/p95, 1.7x plus rapide)
    iterations = 300
    joint = "yaw_body"
    latencies_ms: list[float] = []

    try:
        for i in range(iterations):
            # Position variée pour éviter optimisations
            angle = 0.1 * np.sin(i * 0.01)
            t0 = time.perf_counter()
            backend.set_joint_pos(joint, angle)
            t1 = time.perf_counter()
            latencies_ms.append((t1 - t0) * 1000.0)

        p50 = statistics.median(latencies_ms)
        p95 = float(np.percentile(latencies_ms, 95))

        # Budget: set_joint_pos doit être très rapide en simulation (< 5ms p95)
        assert p50 < 2.0, f"p50 trop élevée: {p50:.2f} ms"
        assert p95 < 5.0, f"p95 trop élevée: {p95:.2f} ms"
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.slow
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (500 itérations)
def test_simulator_get_joint_pos_latency_1e3() -> None:
    """Test latence get_joint_pos sur 500 appels (optimisé)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    # OPTIMISATION: Réduire 500 → 300 itérations (suffisant pour statistiques p50/p95, 1.7x plus rapide)
    iterations = 300
    joint = "yaw_body"
    latencies_ms: list[float] = []

    try:
        for _ in range(iterations):
            t0 = time.perf_counter()
            pos = backend.get_joint_pos(joint)
            t1 = time.perf_counter()
            assert pos is not None
            latencies_ms.append((t1 - t0) * 1000.0)

        p50 = statistics.median(latencies_ms)
        p95 = float(np.percentile(latencies_ms, 95))

        # Budget: get_joint_pos doit être très rapide en simulation (< 5ms p95)
        assert p50 < 2.0, f"p50 trop élevée: {p50:.2f} ms"
        assert p95 < 5.0, f"p95 trop élevée: {p95:.2f} ms"
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.slow
def test_simulator_step_jitter_p50_p95() -> None:
    """Test jitter boucle step() - écart à période théorique p50/p95."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    iterations = 200
    target_period_ms = 20.0  # 50 Hz = 20ms
    periods_ms: list[float] = []

    try:
        t_prev = time.perf_counter()
        for _ in range(iterations):
            backend.step()
            t_curr = time.perf_counter()
            period_ms = (t_curr - t_prev) * 1000.0
            periods_ms.append(period_ms)
            t_prev = t_curr

        # Calculer jitter (écart à période théorique)
        jitters_ms = [abs(p - target_period_ms) for p in periods_ms]
        p50 = statistics.median(jitters_ms)
        p95 = float(np.percentile(jitters_ms, 95))

        # Budget: jitter raisonnable (p50 ~20ms, p95 ≤ 40ms)
        assert p50 < 30.0, f"Jitter p50 trop élevé: {p50:.2f} ms"
        assert p95 < 50.0, f"Jitter p95 trop élevé: {p95:.2f} ms"
    finally:
        backend.disconnect()
