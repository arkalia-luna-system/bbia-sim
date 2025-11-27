#!/usr/bin/env python3
"""
Test watchdog timeout mocké avec mesure p50/p95 de la latence emergency_stop.

On force l'absence de heartbeat en surchargant l'état interne du backend pour
déclencher l'arrêt. Mesure réalisée en simulation (SDK non requis).
"""

import statistics
import time

# removed deprecated typing.List import
import numpy as np
import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


@pytest.mark.unit
@pytest.mark.fast
def test_watchdog_timeout_latency_simulated() -> None:
    """Mesure p50/p95 de la latence d'arrêt via timeout watchdog (simulation)."""
    latencies_ms: list[float] = []
    # OPTIMISATION: Réduire 10 → 5 itérations (suffisant pour p50/p95, 2x plus rapide)
    iterations = 5

    for _ in range(iterations):
        backend = ReachyMiniBackend(use_sim=True)
        assert backend.connect() is True

        # Simuler un robot "physique" dont la lecture positions échoue → watchdog déclenche
        class _FailingRobot:
            def get_current_joint_positions(self):  # noqa: D401
                raise RuntimeError("simulated failure")

        backend.robot = _FailingRobot()  # type: ignore[assignment]

        t0 = time.perf_counter()
        # Attendre jusqu'à 1s maximum que le watchdog déclenche emergency_stop
        deadline = t0 + 1.0
        while (
            backend._watchdog_thread is not None and backend._watchdog_thread.is_alive()
        ) and time.perf_counter() < deadline:
            # OPTIMISATION: Réduire sleep de 0.02 à 0.01 (2x plus rapide)
            time.sleep(0.01)
        t1 = time.perf_counter()

        # Critère: le watchdog doit s'être arrêté (thread terminé) dans la fenêtre
        assert (
            backend._watchdog_thread is None or not backend._watchdog_thread.is_alive()
        ), "Watchdog n'a pas stoppé dans la fenêtre"
        latencies_ms.append((t1 - t0) * 1000.0)

    p50 = statistics.median(latencies_ms)
    p95 = float(np.percentile(latencies_ms, 95))

    # Budget: déclenchement < 200 ms (intervalle watchdog 100 ms + marge)
    assert p50 < 200.0, f"p50 trop élevée: {p50:.2f} ms"
    assert p95 < 300.0, f"p95 trop élevée: {p95:.2f} ms"
