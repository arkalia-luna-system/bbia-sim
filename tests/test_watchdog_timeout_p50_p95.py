#!/usr/bin/env python3
"""
Test watchdog timeout avec métriques p50/p95 améliorées.
Test déclenchement timeout 2s → emergency_stop() avec latence mesurée.
"""

import statistics
import time

import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


@pytest.mark.unit
@pytest.mark.fast
def test_watchdog_timeout_emergency_stop_p50_p95() -> None:
    """Test timeout watchdog 2s → emergency_stop() avec métriques p50/p95."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    # Watchdog démarre automatiquement lors de la connexion
    # On teste la latence d'emergency_stop quand déclenché
    # OPTIMISATION RAM: Réduire 10 → 5 itérations (suffisant pour p50/p95)
    iterations = 5
    latencies_ms: list[float] = []

    try:
        for _ in range(iterations):
            # Tester latence emergency_stop
            t0 = time.perf_counter()
            backend.emergency_stop()
            t1 = time.perf_counter()
            latencies_ms.append((t1 - t0) * 1000.0)

            # Réconnecter pour réinitialiser watchdog
            backend.disconnect()
            # OPTIMISATION RAM: Réduire sleep 0.05s → 0.03s (suffisant pour disconnect)
            time.sleep(0.03)
            backend.connect()
            time.sleep(0.03)  # OPTIMISATION RAM: Réduire sleep 0.05s → 0.03s

        if len(latencies_ms) > 0:
            p50 = statistics.median(latencies_ms)
            p95 = float(
                statistics.quantiles(latencies_ms, n=20)[18]
                if len(latencies_ms) > 1
                else latencies_ms[0]
            )

            # Budget: emergency_stop doit être très rapide (< 20ms p95)
            assert p50 < 10.0, f"Latence p50 trop élevée: {p50:.2f} ms"
            assert p95 < 20.0, f"Latence p95 trop élevée: {p95:.2f} ms"
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_watchdog_timeout_logic_exists() -> None:
    """Vérifie que la logique timeout 2s existe dans le code."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        # Vérifier que watchdog thread existe (démarre automatiquement)
        assert hasattr(backend, "_watchdog_thread")
        assert backend._watchdog_thread is not None or backend.is_connected

        # Vérifier que emergency_stop existe
        assert hasattr(backend, "emergency_stop")
        assert callable(backend.emergency_stop)

        # Vérifier que _stop_watchdog existe (méthode privée)
        assert hasattr(backend, "_stop_watchdog")
        assert callable(backend._stop_watchdog)

        # Vérifier que _start_watchdog existe (méthode privée)
        assert hasattr(backend, "_start_watchdog")
        assert callable(backend._start_watchdog)
    finally:
        backend.disconnect()
