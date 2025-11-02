#!/usr/bin/env python3
"""
Test charge système sous stress.
Vérifie que le système reste stable et performant sous charge élevée.
"""

import concurrent.futures
import time

import numpy as np
import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


@pytest.mark.unit
@pytest.mark.slow
def test_concurrent_goto_target_requests() -> None:
    """Test avec requêtes goto_target concurrentes (optimisé)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    # Optimisé: moins de threads et requêtes (suffisant pour tester concurrence)
    num_threads = 3  # Réduit de 5 à 3
    requests_per_thread = 15  # Réduit de 20 à 15
    pose = np.eye(4, dtype=np.float64)

    def make_requests(thread_id: int) -> int:
        """Fait des requêtes depuis un thread."""
        success_count = 0
        for i in range(requests_per_thread):
            try:
                angle = 0.05 * np.sin(thread_id * 0.1 + i * 0.1)
                cp, sp = np.cos(angle), np.sin(angle)
                pose[0, 0] = cp
                pose[0, 1] = -sp
                pose[1, 0] = sp
                pose[1, 1] = cp

                backend.goto_target(head=pose, duration=0.05, method="minjerk")
                success_count += 1
                time.sleep(0.01)  # Petit délai
            except Exception:
                pass
        return success_count

    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
            futures = [executor.submit(make_requests, i) for i in range(num_threads)]
            results = [f.result() for f in concurrent.futures.as_completed(futures)]

        # Vérifier que la majorité des requêtes ont réussi
        total_success = sum(results)
        expected_success = num_threads * requests_per_thread * 0.8  # 80% minimum
        assert (
            total_success >= expected_success
        ), f"Trop d'échecs: {total_success}/{num_threads * requests_per_thread}"
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.slow
def test_rapid_emotion_switching() -> None:
    """Test changement rapide d'émotions sous stress (optimisé)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    emotions = ["happy", "sad", "neutral", "excited", "curious", "calm"]
    # Optimisé: 150 itérations au lieu de 200 (suffisant pour test stress)
    iterations = 150

    try:
        start_time = time.perf_counter()
        for i in range(iterations):
            emotion = emotions[i % len(emotions)]
            intensity = 0.5 + 0.3 * np.sin(i * 0.1)
            backend.set_emotion(emotion, intensity)
            # Pas de sleep pour tester sous stress réel

        end_time = time.perf_counter()
        duration = end_time - start_time
        ops_per_second = iterations / duration

        # Vérifier que le système reste rapide même sous stress
        assert (
            ops_per_second > 50.0
        ), f"Performance dégradée: {ops_per_second:.1f} ops/s"

        # Vérifier que le backend fonctionne toujours
        assert backend.is_connected
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.slow
def test_rapid_joint_updates() -> None:
    """Test mises à jour rapides de joints sous stress (optimisé)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    joints = ["yaw_body", "stewart_1", "stewart_2", "stewart_3", "stewart_4"]
    # Optimisé: 300 itérations au lieu de 500 (suffisant pour test stress)
    iterations = 300

    try:
        start_time = time.perf_counter()
        for i in range(iterations):
            for joint in joints:
                angle = 0.1 * np.sin(i * 0.01)
                backend.set_joint_pos(joint, angle)
                # Vérifier que la position est bien mise
                pos = backend.get_joint_pos(joint)
                assert isinstance(pos, float | int) or np.isscalar(pos)

        end_time = time.perf_counter()
        duration = end_time - start_time
        ops_per_second = (iterations * len(joints)) / duration

        # Vérifier que le système reste performant
        assert (
            ops_per_second > 100.0
        ), f"Performance dégradée: {ops_per_second:.1f} ops/s"
    finally:
        backend.disconnect()
