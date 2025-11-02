#!/usr/bin/env python3
"""
Benchmark léger CPU/RAM (simulation) sur 10 s.
Sans dépendances externes: utilise tracemalloc et process_time comme proxys.
"""

import time
import tracemalloc

import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


@pytest.mark.unit
@pytest.mark.fast
def test_runtime_budget_simulation_10s() -> None:
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        duration_s = 10.0

        # Démarrer suivi mémoire
        tracemalloc.start()
        t0_wall = time.perf_counter()
        t0_cpu = time.process_time()

        # Boucle légère: step() toutes 50 ms (~20 Hz) pendant 10 s
        while time.perf_counter() - t0_wall < duration_s:
            backend.step()
            time.sleep(0.05)

        current, peak = tracemalloc.get_traced_memory()
        t1_cpu = time.process_time()
        tracemalloc.stop()

        cpu_time_s = t1_cpu - t0_cpu

        # Budgets très larges pour CI
        # - pic mémoire < 64 MB
        # - temps CPU < 2.5 s sur 10 s mur (assoupli pour CI avec overhead)
        assert peak < 64 * 1024 * 1024, f"Peak RAM trop élevé: {peak} B"
        assert cpu_time_s < 2.5, f"Temps CPU trop élevé: {cpu_time_s:.2f}s/10s"
    finally:
        backend.disconnect()
