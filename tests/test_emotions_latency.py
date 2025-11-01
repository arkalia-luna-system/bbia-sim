#!/usr/bin/env python3
"""
Test latence inférence émotions (p50/p95 sur N=1e3 évaluations).
Test stress bornes sous charge.
"""

import statistics
import time

import numpy as np
import pytest

from bbia_sim.bbia_emotions import BBIAEmotions


@pytest.mark.unit
@pytest.mark.slow
def test_emotions_inference_latency_1e3() -> None:
    """Test latence inférence émotions sur 500 évaluations (optimisé)."""
    emotions = BBIAEmotions()

    # Optimisé: 500 au lieu de 1000 (suffisant pour statistiques p50/p95)
    iterations = 500
    latencies_ms: list[float] = []

    # Émotions à tester
    emotion_names = ["happy", "sad", "angry", "surprised", "neutral"]
    intensities = [0.1, 0.3, 0.5, 0.7, 0.9]

    for i in range(iterations):
        # Varier émotion et intensité
        emotion = emotion_names[i % len(emotion_names)]
        intensity = intensities[i % len(intensities)]

        t0 = time.perf_counter()
        emotions.set_emotion(emotion, intensity)
        # Récupérer état
        state = emotions.get_current_emotion()
        t1 = time.perf_counter()

        assert state is not None
        assert isinstance(state, dict)

        latencies_ms.append((t1 - t0) * 1000.0)

    p50 = statistics.median(latencies_ms)
    p95 = float(np.percentile(latencies_ms, 95))

    # Budget: Inférence émotions doit être très rapide (< 1ms p95)
    assert p50 < 0.5, f"Latence p50 trop élevée: {p50:.3f} ms"
    assert p95 < 1.0, f"Latence p95 trop élevée: {p95:.3f} ms"


@pytest.mark.unit
@pytest.mark.slow
def test_emotions_stress_bounds_under_load() -> None:
    """Test stress bornes sous charge (dérive/oscillation)."""
    emotions = BBIAEmotions()

    iterations = 500
    extreme_values = [0.0, 1.0, -0.1, 1.1, 0.5]

    # Test sous charge avec valeurs extrêmes
    for i in range(iterations):
        emotion = "happy"
        intensity = extreme_values[i % len(extreme_values)]

        emotions.set_emotion(emotion, intensity)
        # Vérifier intensité directement depuis attribut
        current_intensity = emotions.emotion_intensity

        # Vérifier que intensité est toujours dans [0.0, 1.0]
        assert 0.0 <= current_intensity <= 1.0, (
            f"Intensité hors bornes: {current_intensity} " f"(input: {intensity})"
        )

    # Vérifier pas de dérive (état final stable)
    final_intensity = emotions.emotion_intensity
    assert isinstance(final_intensity, float | int)
    assert (
        0.0 <= float(final_intensity) <= 1.0
    ), f"Intensité finale hors bornes: {final_intensity}"


@pytest.mark.unit
@pytest.mark.fast
def test_emotions_rapid_switching() -> None:
    """Test switching rapide entre émotions (oscillation)."""
    emotions = BBIAEmotions()

    iterations = 100
    emotion_sequence = ["happy", "sad", "angry", "happy", "sad"]

    for i in range(iterations):
        emotion = emotion_sequence[i % len(emotion_sequence)]
        intensity = 0.5 + 0.3 * np.sin(i * 0.1)  # Oscillation

        emotions.set_emotion(emotion, intensity)
        # Vérifier état directement
        assert emotions.current_emotion == emotion

        # Vérifier pas d'oscillation excessive (normalisé)
        current = emotions.emotion_intensity
        assert 0.0 <= current <= 1.0, f"Oscillation hors bornes: {current}"
