#!/usr/bin/env python3
"""
Benchmark de latence emergency_stop (simulation).

Mesure p50/p95 sur N itérations en mode simulation.
"""

import statistics
import time

import numpy as np
import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


@pytest.mark.unit
@pytest.mark.fast
def test_emergency_stop_latency_simulation() -> None:
    """Mesure latence emergency_stop (p50/p95) en simulation.

    Le test reste rapide (N=30) et non-e2e.
    """
    latencies_ms: list[float] = []
    iterations = 30

    for _ in range(iterations):
        backend = ReachyMiniBackend(use_sim=True)
        assert backend.connect() is True
        t0 = time.perf_counter()
        res = backend.emergency_stop()
        t1 = time.perf_counter()
        assert res is False  # en simulation l'arrêt renvoie False (pas de robot)
        assert backend.is_connected is False
        latencies_ms.append((t1 - t0) * 1000.0)

    p50 = statistics.median(latencies_ms)
    p95 = float(np.percentile(latencies_ms, 95))

    # Budget réaliste en simulation (arrêt local immédiat)
    assert p50 < 10.0, f"p50 trop élevée: {p50:.2f} ms"
    assert p95 < 20.0, f"p95 trop élevée: {p95:.2f} ms"
