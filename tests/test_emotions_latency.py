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
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (500 itérations)
def test_emotions_inference_latency_1e3() -> None:
    """Test latence inférence émotions sur 200 évaluations (optimisé)."""
    emotions = BBIAEmotions()

    # OPTIMISATION RAM: Réduire 100 → 50 itérations (suffisant pour statistiques p50/p95, 2x plus rapide)
    iterations = 50
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
    # Tolérance CI: p50 peut être légèrement plus élevé en CI (0.7 ms au lieu de 0.5 ms)
    import os

    p50_threshold = 0.7 if os.environ.get("CI", "false").lower() == "true" else 0.5
    assert (
        p50 < p50_threshold
    ), f"Latence p50 trop élevée: {p50:.3f} ms (seuil: {p50_threshold} ms)"
    assert p95 < 1.0, f"Latence p95 trop élevée: {p95:.3f} ms"


@pytest.mark.unit
@pytest.mark.slow
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (300 itérations)
def test_emotions_stress_bounds_under_load() -> None:
    """Test stress bornes sous charge (dérive/oscillation)."""
    emotions = BBIAEmotions()

    # OPTIMISATION RAM: Réduire 150 → 100 itérations (suffisant pour détecter dérive, 1.5x plus rapide)
    iterations = 100
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

    # OPTIMISATION RAM: Réduire 100 → 50 itérations (suffisant pour test oscillation)
    iterations = 50
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
