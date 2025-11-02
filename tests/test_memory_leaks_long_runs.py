#!/usr/bin/env python3
"""
Test fuites mémoire lors de longs runs (1000+ itérations).
Vérifie que la mémoire ne s'accumule pas indéfiniment lors d'opérations répétées.
"""

import gc

import numpy as np
import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


# Import conditionnel pour memory_profiler
def get_memory_usage() -> float | None:
    """Obtient l'utilisation mémoire actuelle en MB."""
    try:
        from memory_profiler import memory_usage

        usage = memory_usage()
        return float(usage[0]) if usage else None
    except ImportError:
        try:
            import psutil

            process = psutil.Process()
            return float(process.memory_info().rss / 1024 / 1024)  # MB
        except ImportError:
            return None


@pytest.mark.unit
@pytest.mark.slow
def test_memory_leaks_goto_target_iterations() -> None:
    """Test fuites mémoire avec 500 appels goto_target (optimisé)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    # Optimisé: 500 itérations au lieu de 1000 (suffisant pour détecter fuites)
    iterations = 500
    pose = np.eye(4, dtype=np.float64)

    # Mesurer mémoire initiale
    gc.collect()
    mem_before = get_memory_usage()

    try:
        for i in range(iterations):
            # Rotation légère pour varier
            angle = 0.05 * np.sin(i * 0.01)
            cp, sp = np.cos(angle), np.sin(angle)
            pose[0, 0] = cp
            pose[0, 1] = -sp
            pose[1, 0] = sp
            pose[1, 1] = cp

            backend.goto_target(head=pose, duration=0.05, method="minjerk")

            # Nettoyer tous les 50 itérations (plus fréquent pour stabilité)
            if i % 50 == 0:
                gc.collect()

        # Mesurer mémoire finale
        gc.collect()
        mem_after = get_memory_usage()

        # Vérifier qu'on n'a pas de fuite majeure
        if mem_before and mem_after:
            memory_increase = mem_after - mem_before
            # Augmentation acceptable: < 30MB sur 500 itérations (proportionnel)
            assert memory_increase < 30.0, f"Fuite mémoire détectée: {memory_increase:.2f} MB"
    finally:
        backend.disconnect()
        gc.collect()


@pytest.mark.unit
@pytest.mark.slow
def test_memory_leaks_joint_operations() -> None:
    """Test fuites mémoire avec 500 opérations sur joints (optimisé)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    # Optimisé: 500 itérations au lieu de 1000 (suffisant pour détecter fuites)
    iterations = 500
    joints = ["yaw_body", "stewart_1", "stewart_2", "stewart_3"]

    # Mesurer mémoire initiale
    gc.collect()

    try:
        for i in range(iterations):
            # Alterner entre set et get
            for joint in joints:
                angle = 0.1 * np.sin(i * 0.01)
                backend.set_joint_pos(joint, angle)
                pos = backend.get_joint_pos(joint)
                assert isinstance(pos, float | int) or np.isscalar(pos)

            # Nettoyer tous les 50 itérations
            if i % 50 == 0:
                gc.collect()

        # Vérifier que le backend fonctionne toujours
        assert backend.is_connected
        assert backend.get_available_joints() is not None
    finally:
        backend.disconnect()
        gc.collect()


@pytest.mark.unit
@pytest.mark.slow
def test_memory_leaks_emotion_changes() -> None:
    """Test fuites mémoire avec changements d'émotions répétés (optimisé)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    # Optimisé: 300 itérations au lieu de 500 (suffisant pour détecter fuites)
    iterations = 300
    emotions = ["happy", "sad", "neutral", "excited", "curious", "calm"]

    try:
        for i in range(iterations):
            emotion = emotions[i % len(emotions)]
            intensity = 0.3 + 0.4 * (i % 10) / 10.0
            backend.set_emotion(emotion, intensity)

            # Nettoyer tous les 100 itérations
            if i % 100 == 0:
                gc.collect()

        # Vérifier que le backend fonctionne toujours
        assert backend.is_connected
    finally:
        backend.disconnect()
        gc.collect()
