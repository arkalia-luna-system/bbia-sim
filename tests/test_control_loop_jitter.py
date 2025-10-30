#!/usr/bin/env python3
"""
Benchmark jitter boucle contrôle (simulation). Si SDK réel indisponible, on simule une
boucle 50 Hz et vérifie p50/p95 de l'intervalle.
"""

import statistics
import time

import numpy as np
import pytest


@pytest.mark.unit
@pytest.mark.fast
def test_control_loop_jitter_simulated_50hz() -> None:
    """Mesure du jitter d'une boucle 50 Hz simulée (20 ms cible)."""
    target_period_s = 0.020
    samples = 200
    timestamps: list[float] = []

    t_prev = time.perf_counter()
    for _ in range(samples):
        # Travail minimal
        time.sleep(target_period_s)
        t_now = time.perf_counter()
        timestamps.append(t_now - t_prev)
        t_prev = t_now

    # Environnements CI peuvent être bruyants : tolérances larges
    intervals_ms = [dt * 1000.0 for dt in timestamps]
    p50 = statistics.median(intervals_ms)
    p95 = float(np.percentile(intervals_ms, 95))

    # Cible ~20 ms +/- tolérences
    assert 15.0 <= p50 <= 30.0, f"p50 hors tolérance: {p50:.2f} ms"
    assert p95 <= 40.0, f"p95 trop élevée: {p95:.2f} ms"
