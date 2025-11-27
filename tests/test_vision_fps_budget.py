#!/usr/bin/env python3
"""Benchmarks Vision: FPS 10s et budget CPU/RAM 10s (simulation).

Skips si vision désactivée. Mesure réalisée sans modèles lourds.
"""

import gc
import os
import statistics
import time

import pytest

from bbia_sim.bbia_vision import BBIAVision


@pytest.mark.unit
@pytest.mark.slow  # OPTIMISATION RAM: Test avec boucle (mais optimisé)
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (boucle 3s + peut charger modèles YOLO)
def test_vision_fps_10s_simulated() -> None:
    # Skip en CI si trop lent (tolérance déjà ajustée mais peut encore timeout)
    is_ci = os.environ.get("CI", "false").lower() == "true"
    if is_ci and os.environ.get("BBIA_SKIP_SLOW_TESTS", "0") == "1":
        pytest.skip("Tests lents désactivés en CI (BBIA_SKIP_SLOW_TESTS=1)")
    if os.environ.get("BBIA_DISABLE_VISION", "0") == "1":
        pytest.skip("Vision désactivée par BBIA_DISABLE_VISION=1")

    vision = BBIAVision(robot_api=None)

    try:
        # OPTIMISATION RAM: Réduire 3s → 2s (suffisant pour mesurer FPS, 1.5x plus rapide)
        duration_s = 2.0
        frames = 0
        latencies_ms: list[float] = []
        max_iterations = (
            100  # OPTIMISATION: Limiter itérations pour éviter boucle infinie
        )

        t0 = time.perf_counter()
        iteration = 0
        while time.perf_counter() - t0 < duration_s and iteration < max_iterations:
            iteration += 1
            t_start = time.perf_counter()
            _ = vision.scan_environment()
            t_end = time.perf_counter()
            frames += 1
            latencies_ms.append((t_end - t_start) * 1000.0)

        elapsed = time.perf_counter() - t0
        fps = frames / elapsed if elapsed > 0 else 0.0

        # Cible CPU: ≥ 10 FPS en simulation (tolérance CI si plus lent)
        is_ci = os.environ.get("CI", "false").lower() == "true"
        min_fps = 5.0 if is_ci else 10.0  # Tolérance CI réduite
        assert fps >= min_fps, f"FPS trop bas: {fps:.2f} (min: {min_fps})"

        # Latence pipeline simulé - seuils ajustés pour simulation réaliste
        p50 = statistics.median(latencies_ms)
        latencies_ms.sort()
        p95 = latencies_ms[int(0.95 * len(latencies_ms)) - 1] if latencies_ms else 0.0
        # Seuils ajustés: simulation peut avoir latence plus élevée selon matériel
        # CI peut être plus lent, tolérance augmentée
        max_p50 = 200.0 if is_ci else 100.0
        max_p95 = 400.0 if is_ci else 200.0
        assert (
            p50 < max_p50 and p95 < max_p95
        ), f"Latence élevée: p50={p50:.2f}ms (max: {max_p50}) p95={p95:.2f}ms (max: {max_p95})"
    finally:
        # OPTIMISATION RAM: Nettoyer modèles vision après le test
        try:
            # Décharger détecteurs YOLO si chargés
            if hasattr(vision, "yolo_detector") and vision.yolo_detector:
                vision.yolo_detector.model = None
                vision.yolo_detector.is_loaded = False
            # Décharger détecteurs MediaPipe si chargés
            if hasattr(vision, "face_detector") and vision.face_detector:
                vision.face_detector.face_detection = None
        except (AttributeError, TypeError):
            pass
        # Vider cache YOLO
        try:
            import bbia_sim.vision_yolo as vision_yolo_module

            with vision_yolo_module._yolo_cache_lock:
                vision_yolo_module._yolo_model_cache.clear()
        except (AttributeError, ImportError):
            pass
        gc.collect()


@pytest.mark.unit
@pytest.mark.slow  # OPTIMISATION RAM: Test avec boucle (mais optimisé)
@pytest.mark.heavy  # OPTIMISATION RAM: Test lourd (boucle 3s + peut charger modèles YOLO)
def test_vision_budget_cpu_ram_10s() -> None:
    # Skip en CI si trop lent (tolérance déjà ajustée mais peut encore timeout)
    is_ci = os.environ.get("CI", "false").lower() == "true"
    if is_ci and os.environ.get("BBIA_SKIP_SLOW_TESTS", "0") == "1":
        pytest.skip("Tests lents désactivés en CI (BBIA_SKIP_SLOW_TESTS=1)")
    if os.environ.get("BBIA_DISABLE_VISION", "0") == "1":
        pytest.skip("Vision désactivée par BBIA_DISABLE_VISION=1")

    import tracemalloc

    vision = BBIAVision(robot_api=None)

    try:
        # OPTIMISATION RAM: Réduire 3s → 2s (suffisant pour mesurer budget, 1.5x plus rapide)
        duration_s = 2.0
        tracemalloc.start()
        t0_cpu = time.process_time()
        t0 = time.perf_counter()
        max_iterations = (
            100  # OPTIMISATION: Limiter itérations pour éviter boucle infinie
        )

        iteration = 0
        while time.perf_counter() - t0 < duration_s and iteration < max_iterations:
            _ = vision.scan_environment()
            # OPTIMISATION: Réduire sleep de 0.01 à 0.005 (2x plus rapide)
            time.sleep(0.005)
            iteration += 1

        current, peak = tracemalloc.get_traced_memory()
        t1_cpu = time.process_time()
        tracemalloc.stop()

        cpu_time_s = t1_cpu - t0_cpu

        # Budgets larges CI/macOS - seuils ajustés pour simulation réaliste
        assert peak < 200 * 1024 * 1024, f"Peak RAM trop élevé: {peak}B"
        # Seuil CPU ajusté: simulation peut utiliser plus de CPU selon matériel
        # OPTIMISATION RAM: Proportionnel à 3s au lieu de 5s
        assert cpu_time_s < 3.0, f"Temps CPU trop élevé: {cpu_time_s:.2f}s/3s"
    finally:
        # OPTIMISATION RAM: Nettoyer modèles vision après le test
        try:
            # Décharger détecteurs YOLO si chargés
            if hasattr(vision, "yolo_detector") and vision.yolo_detector:
                vision.yolo_detector.model = None
                vision.yolo_detector.is_loaded = False
            # Décharger détecteurs MediaPipe si chargés
            if hasattr(vision, "face_detector") and vision.face_detector:
                vision.face_detector.face_detection = None
        except (AttributeError, TypeError):
            pass
        # Vider cache YOLO
        try:
            import bbia_sim.vision_yolo as vision_yolo_module

            with vision_yolo_module._yolo_cache_lock:
                vision_yolo_module._yolo_model_cache.clear()
        except (AttributeError, ImportError):
            pass
        gc.collect()
