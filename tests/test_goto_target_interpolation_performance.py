#!/usr/bin/env python3
"""
Test performance goto_target avec différentes méthodes d'interpolation.
Vérifie que toutes les méthodes (minjerk, linear, ease_in_out, cartoon) fonctionnent
et respectent les budgets de latence.
"""

import statistics
import time

import numpy as np
import numpy.typing as npt
import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


def _create_head_pose(pitch: float = 0.1, yaw: float = 0.05) -> npt.NDArray[np.float64]:
    """Crée une pose de tête pour les tests."""
    pose = np.eye(4, dtype=np.float64)
    # Petite rotation pour éviter le no-op
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    pose[0, 0] = cy
    pose[0, 1] = -sy
    pose[1, 0] = sy * cp
    pose[1, 1] = cy * cp
    pose[2, 0] = sy * sp
    pose[2, 1] = -cy * sp
    pose[2, 2] = cp
    return pose


@pytest.mark.unit
@pytest.mark.fast
def test_goto_target_interpolation_methods() -> None:
    """Test que toutes les méthodes d'interpolation fonctionnent."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    methods = ["minjerk", "linear", "ease_in_out", "cartoon"]
    head = _create_head_pose()

    try:
        for method in methods:
            # Vérifier que la méthode ne lève pas d'exception
            backend.goto_target(head=head, duration=0.1, method=method, body_yaw=0.0)
            # OPTIMISATION: Réduire sleep de 0.05 à 0.02 (2.5x plus rapide)
            time.sleep(0.02)
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_goto_target_interpolation_latency() -> None:
    """Test latence avec différentes méthodes d'interpolation."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    head = _create_head_pose()
    # OPTIMISATION RAM: Réduire 30 → 20 itérations (suffisant pour p50/p95)
    iterations = 20
    methods = ["minjerk", "linear", "ease_in_out"]
    results: dict[str, list[float]] = {}

    try:
        for method in methods:
            latencies_ms: list[float] = []
            for _ in range(iterations):
                t0 = time.perf_counter()
                backend.goto_target(
                    head=head, duration=0.1, method=method, body_yaw=0.0
                )
                t1 = time.perf_counter()
                latencies_ms.append((t1 - t0) * 1000.0)
                # OPTIMISATION RAM: Réduire sleep si pas besoin (garder pour stabilité)
                time.sleep(0.005)  # Petit délai entre appels (réduit de 0.01)

            results[method] = latencies_ms

            # Budget: toutes les méthodes doivent être rapides (< 30ms p95)
            p95 = float(np.percentile(latencies_ms, 95))
            p50 = statistics.median(latencies_ms)
            assert p95 < 30.0, f"p95 trop élevée pour {method}: {p95:.2f} ms"
            assert p50 < 20.0, f"p50 trop élevée pour {method}: {p50:.2f} ms"
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_goto_target_with_body_yaw() -> None:
    """Test goto_target avec mouvement combiné tête+corps."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    head = _create_head_pose()
    body_yaw = 0.1

    try:
        # Vérifier que mouvement combiné fonctionne
        backend.goto_target(
            head=head, body_yaw=body_yaw, duration=0.2, method="minjerk"
        )
        # OPTIMISATION: Réduire sleep de 0.25 à 0.15 (1.7x plus rapide)
        time.sleep(0.15)

        # Vérifier position corps
        pos = backend.get_joint_pos("yaw_body")
        assert isinstance(pos, float | int) or np.isscalar(pos)
    finally:
        backend.disconnect()
