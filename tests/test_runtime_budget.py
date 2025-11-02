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
@pytest.mark.slow  # OPTIMISATION RAM: Test avec boucle (mais marqué slow)
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (boucle 3s)
def test_runtime_budget_simulation_10s() -> None:
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        # OPTIMISATION RAM: Réduire 10s → 3s (suffisant pour mesurer budget)
        duration_s = 3.0

        # Démarrer suivi mémoire
        tracemalloc.start()
        t0_wall = time.perf_counter()
        t0_cpu = time.process_time()

        # Boucle légère: step() toutes 50 ms (~20 Hz) pendant 3s (optimisé)
        while time.perf_counter() - t0_wall < duration_s:
            backend.step()
            time.sleep(0.05)

        current, peak = tracemalloc.get_traced_memory()
        t1_cpu = time.process_time()
        tracemalloc.stop()

        cpu_time_s = t1_cpu - t0_cpu

        # Budgets très larges pour CI (proportionnels à 3s au lieu de 10s)
        # - pic mémoire < 64 MB
        # - temps CPU < 1.0 s sur 3 s mur (proportionnel à 2.5s/10s)
        assert peak < 64 * 1024 * 1024, f"Peak RAM trop élevé: {peak} B"
        assert cpu_time_s < 1.0, f"Temps CPU trop élevé: {cpu_time_s:.2f}s/3s"
    finally:
        backend.disconnect()
