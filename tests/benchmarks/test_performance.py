#!/usr/bin/env python3
"""
Tests de performance consolidés pour BBIA-SIM.

Mesure la latence actuelle pour :
- Vision (scan_environment)
- Audio (STT/transcription)
- Mouvements (goto_target)

Objectifs de performance :
- Vision : <50ms (actuel ~100ms)
- Audio : <100ms (actuel ~200ms)
- Mouvements : <10ms (actuel ~20ms)

Ce fichier sert de baseline pour mesurer les améliorations après optimisations.
"""

import os
import statistics
import time

import numpy as np
import numpy.typing as npt
import pytest

# Imports conditionnels pour éviter erreurs si modules absents
try:
    from bbia_sim.bbia_vision import BBIAVision
except ImportError:
    BBIAVision = None  # type: ignore[assignment,misc]

try:
    from bbia_sim.bbia_audio import BBIAAudio  # type: ignore[attr-defined]
except ImportError:
    BBIAAudio = None  # type: ignore[assignment,misc,attr-defined]

try:
    from bbia_sim.bbia_voice import transcribe_audio
except ImportError:
    transcribe_audio = None  # type: ignore[assignment,misc]

try:
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend
except ImportError:
    ReachyMiniBackend = None  # type: ignore[assignment,misc]

try:
    from reachy_mini.utils import create_head_pose  # type: ignore[import-untyped]
except ImportError:
    create_head_pose = None  # type: ignore[assignment,misc]


def _small_pose() -> npt.NDArray[np.float64]:
    """Crée une petite pose de test pour éviter les no-ops.

    Returns:
        Matrice de transformation 4x4 avec petite rotation
    """
    pose = np.eye(4, dtype=np.float64)
    angle = 0.05
    c, s = np.cos(angle), np.sin(angle)
    pose[0, 0] = c
    pose[0, 1] = -s
    pose[1, 0] = s
    pose[1, 1] = c
    return pose


class TestVisionLatency:
    """Tests de latence pour le module vision."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_vision_scan_environment_latency(self) -> None:
        """Mesure la latence actuelle de scan_environment (baseline).

        Objectif : <50ms (actuel ~100ms)
        """
        if os.environ.get("BBIA_DISABLE_VISION", "0") == "1":
            pytest.skip("Vision désactivée par BBIA_DISABLE_VISION=1")

        if BBIAVision is None:
            pytest.skip("BBIAVision non disponible")

        vision = BBIAVision(robot_api=None)
        latencies_ms: list[float] = []
        iterations = 20  # Suffisant pour p50/p95

        # Warmup
        for _ in range(3):
            vision.scan_environment()

        # Mesures
        for _ in range(iterations):
            t0 = time.perf_counter()
            result = vision.scan_environment()
            t1 = time.perf_counter()
            assert isinstance(result, dict)
            latencies_ms.append((t1 - t0) * 1000.0)

        p50 = statistics.median(latencies_ms)
        p95 = float(np.percentile(latencies_ms, 95))
        p99 = float(np.percentile(latencies_ms, 99))
        mean = statistics.mean(latencies_ms)
        min_latency = min(latencies_ms)
        max_latency = max(latencies_ms)

        # Afficher résultats
        print("\n📊 Vision Latency Results:")
        print(f"  Mean: {mean:.2f} ms")
        print(f"  p50:  {p50:.2f} ms")
        print(f"  p95:  {p95:.2f} ms")
        print(f"  p99:  {p99:.2f} ms")
        print(f"  Min:  {min_latency:.2f} ms")
        print(f"  Max:  {max_latency:.2f} ms")

        # Seuils ajustés selon environnement
        is_ci = os.environ.get("CI", "false").lower() == "true"
        max_p50 = 200.0 if is_ci else 100.0
        max_p95 = 400.0 if is_ci else 200.0

        assert p50 < max_p50, f"p50 trop élevée: {p50:.2f} ms (max: {max_p50})"
        assert p95 < max_p95, f"p95 trop élevée: {p95:.2f} ms (max: {max_p95})"

    @pytest.mark.unit
    @pytest.mark.slow
    def test_vision_fps_benchmark(self) -> None:
        """Mesure le FPS du pipeline vision sur 3 secondes.

        Objectif : ≥10 FPS (≥30 FPS idéal)
        """
        if os.environ.get("BBIA_DISABLE_VISION", "0") == "1":
            pytest.skip("Vision désactivée par BBIA_DISABLE_VISION=1")

        if BBIAVision is None:
            pytest.skip("BBIAVision non disponible")

        vision = BBIAVision(robot_api=None)
        duration_s = 3.0
        frames = 0
        latencies_ms: list[float] = []

        t0 = time.perf_counter()
        max_iterations = (
            100  # OPTIMISATION: Limiter itérations pour éviter boucle infinie
        )
        iteration = 0
        while time.perf_counter() - t0 < duration_s and iteration < max_iterations:
            t_start = time.perf_counter()
            _ = vision.scan_environment()
            t_end = time.perf_counter()
            frames += 1
            latencies_ms.append((t_end - t_start) * 1000.0)
            iteration += 1

        elapsed = time.perf_counter() - t0
        fps = frames / elapsed if elapsed > 0 else 0.0

        p50 = statistics.median(latencies_ms)

        print("\n📊 Vision FPS Results:")
        print(f"  FPS:  {fps:.2f}")
        print(f"  Frames: {frames} en {elapsed:.2f}s")
        print(f"  p50 Latency: {p50:.2f} ms")

        is_ci = os.environ.get("CI", "false").lower() == "true"
        min_fps = 5.0 if is_ci else 10.0
        assert fps >= min_fps, f"FPS trop bas: {fps:.2f} (min: {min_fps})"


class TestAudioLatency:
    """Tests de latence pour le module audio."""

    @pytest.mark.unit
    def test_audio_stt_latency(self) -> None:
        """Mesure la latence actuelle de transcription audio (baseline).

        Objectif : <100ms (actuel ~200ms)
        """
        if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
            pytest.skip("Audio désactivé par BBIA_DISABLE_AUDIO=1")

        if transcribe_audio is None:
            pytest.skip("transcribe_audio non disponible")

        # Générer un signal audio de test (0.5s @ 16kHz)
        sample_rate = 16000
        duration_s = 0.5
        tone_hz = 1000
        t = np.arange(int(sample_rate * duration_s)) / sample_rate
        audio_data = (0.25 * np.sin(2 * np.pi * tone_hz * t)).astype(np.float32)

        latencies_ms: list[float] = []
        iterations = 10  # Moins d'itérations car STT peut être lent

        # Warmup
        try:
            transcribe_audio(audio_data, sample_rate)
        except Exception:
            pytest.skip("transcribe_audio non fonctionnel (dépendances manquantes)")

        # Mesures
        for _ in range(iterations):
            t0 = time.perf_counter()
            try:
                _ = transcribe_audio(audio_data, sample_rate)
                t1 = time.perf_counter()
                latencies_ms.append((t1 - t0) * 1000.0)
            except Exception as e:
                pytest.skip(f"transcribe_audio erreur: {e}")

        if not latencies_ms:
            pytest.skip("Aucune mesure de latence réussie")

        p50 = statistics.median(latencies_ms)
        p95 = float(np.percentile(latencies_ms, 95))
        mean = statistics.mean(latencies_ms)
        min_latency = min(latencies_ms)
        max_latency = max(latencies_ms)

        print("\n📊 Audio STT Latency Results:")
        print(f"  Mean: {mean:.2f} ms")
        print(f"  p50:  {p50:.2f} ms")
        print(f"  p95:  {p95:.2f} ms")
        print(f"  Min:  {min_latency:.2f} ms")
        print(f"  Max:  {max_latency:.2f} ms")

        # Seuils larges pour STT (peut être lent selon modèle)
        is_ci = os.environ.get("CI", "false").lower() == "true"
        max_p50 = 2000.0 if is_ci else 500.0
        max_p95 = 5000.0 if is_ci else 1000.0

        assert p50 < max_p50, f"p50 trop élevée: {p50:.2f} ms (max: {max_p50})"
        assert p95 < max_p95, f"p95 trop élevée: {p95:.2f} ms (max: {max_p95})"

    @pytest.mark.unit
    def test_audio_record_latency(self) -> None:
        """Mesure la latence d'enregistrement audio (overhead SDK).

        Objectif : <50ms overhead
        """
        if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
            pytest.skip("Audio désactivé par BBIA_DISABLE_AUDIO=1")

        if BBIAAudio is None:
            pytest.skip("BBIAAudio non disponible")

        audio = BBIAAudio(robot_api=None)
        latencies_ms: list[float] = []
        iterations = 10
        duration_s = 0.5

        # Warmup
        try:
            audio.record_audio(duration_s)
        except Exception:
            pytest.skip("record_audio non fonctionnel (hardware indisponible)")

        # Mesures
        for _ in range(iterations):
            t0 = time.perf_counter()
            try:
                _ = audio.record_audio(duration_s)
                t1 = time.perf_counter()
                # Latence = temps total - durée audio attendue
                latency_ms = ((t1 - t0) - duration_s) * 1000.0
                latencies_ms.append(latency_ms)
            except Exception as e:
                pytest.skip(f"record_audio erreur: {e}")

        if not latencies_ms:
            pytest.skip("Aucune mesure de latence réussie")

        p50 = statistics.median(latencies_ms)
        p95 = float(np.percentile(latencies_ms, 95))
        mean = statistics.mean(latencies_ms)

        print("\n📊 Audio Record Latency Results (overhead):")
        print(f"  Mean: {mean:.2f} ms")
        print(f"  p50:  {p50:.2f} ms")
        print(f"  p95:  {p95:.2f} ms")

        # Overhead acceptable < 100ms
        is_ci = os.environ.get("CI", "false").lower() == "true"
        max_p50 = 200.0 if is_ci else 100.0
        max_p95 = 500.0 if is_ci else 200.0

        assert p50 < max_p50, f"p50 overhead trop élevé: {p50:.2f} ms (max: {max_p50})"
        assert p95 < max_p95, f"p95 overhead trop élevé: {p95:.2f} ms (max: {max_p95})"


class TestMovementLatency:
    """Tests de latence pour les mouvements robot."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_goto_target_latency(self) -> None:
        """Mesure la latence actuelle de goto_target (baseline).

        Objectif : <10ms (actuel ~20ms)
        """
        if ReachyMiniBackend is None:
            pytest.skip("ReachyMiniBackend non disponible")

        backend = ReachyMiniBackend(use_sim=True)
        assert backend.connect() is True

        # Créer pose de test
        if create_head_pose is not None:
            head = create_head_pose(pitch=0.05, yaw=0.05, degrees=False)
        else:
            head = _small_pose()

        latencies_ms: list[float] = []
        iterations = 50

        try:
            # Warmup
            for _ in range(3):
                backend.goto_target(
                    head=head, duration=0.1, method="minjerk", body_yaw=0.0
                )

            # Mesures
            for _ in range(iterations):
                t0 = time.perf_counter()
                backend.goto_target(
                    head=head, duration=0.1, method="minjerk", body_yaw=0.0
                )
                t1 = time.perf_counter()
                latencies_ms.append((t1 - t0) * 1000.0)

            p50 = statistics.median(latencies_ms)
            p95 = float(np.percentile(latencies_ms, 95))
            p99 = float(np.percentile(latencies_ms, 99))
            mean = statistics.mean(latencies_ms)
            min_latency = min(latencies_ms)
            max_latency = max(latencies_ms)

            print("\n📊 Movement Latency Results (goto_target):")
            print(f"  Mean: {mean:.2f} ms")
            print(f"  p50:  {p50:.2f} ms")
            print(f"  p95:  {p95:.2f} ms")
            print(f"  p99:  {p99:.2f} ms")
            print(f"  Min:  {min_latency:.2f} ms")
            print(f"  Max:  {max_latency:.2f} ms")

            # Budget: appel wrapper doit être rapide en simulation (< 20 ms)
            assert p50 < 20.0, f"p50 trop élevée: {p50:.2f} ms"
            assert p95 < 40.0, f"p95 trop élevée: {p95:.2f} ms"

        finally:
            backend.disconnect()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_look_at_world_latency(self) -> None:
        """Mesure la latence de look_at_world.

        Objectif : <15ms
        """
        if ReachyMiniBackend is None:
            pytest.skip("ReachyMiniBackend non disponible")

        backend = ReachyMiniBackend(use_sim=True)
        assert backend.connect() is True

        latencies_ms: list[float] = []
        iterations = 30

        try:
            # Warmup
            for _ in range(3):
                backend.look_at_world(0.2, 0.0, 0.3, duration=0.1)

            # Mesures
            for _ in range(iterations):
                t0 = time.perf_counter()
                backend.look_at_world(0.2, 0.0, 0.3, duration=0.1)
                t1 = time.perf_counter()
                latencies_ms.append((t1 - t0) * 1000.0)

            p50 = statistics.median(latencies_ms)
            p95 = float(np.percentile(latencies_ms, 95))
            mean = statistics.mean(latencies_ms)

            print("\n📊 Look At World Latency Results:")
            print(f"  Mean: {mean:.2f} ms")
            print(f"  p50:  {p50:.2f} ms")
            print(f"  p95:  {p95:.2f} ms")

            # Budget: < 15ms pour look_at_world
            assert p50 < 15.0, f"p50 trop élevée: {p50:.2f} ms"
            assert p95 < 30.0, f"p95 trop élevée: {p95:.2f} ms"

        finally:
            backend.disconnect()


class TestPerformanceBaseline:
    """Tests consolidés pour établir la baseline de performance."""

    @pytest.mark.unit
    @pytest.mark.slow
    def test_full_pipeline_latency(self) -> None:
        """Mesure la latence d'un pipeline complet (vision + mouvement).

        Objectif : <150ms end-to-end
        """
        if os.environ.get("BBIA_DISABLE_VISION", "0") == "1":
            pytest.skip("Vision désactivée par BBIA_DISABLE_VISION=1")

        if BBIAVision is None or ReachyMiniBackend is None:
            pytest.skip("Modules requis non disponibles")

        vision = BBIAVision(robot_api=None)
        backend = ReachyMiniBackend(use_sim=True)
        assert backend.connect() is True

        latencies_ms: list[float] = []
        iterations = 10

        try:
            # Warmup
            for _ in range(2):
                result = vision.scan_environment()
                if result.get("objects") and len(result["objects"]) > 0:
                    # Simuler mouvement vers premier objet
                    if create_head_pose is not None:
                        head = create_head_pose(pitch=0.1, yaw=0.1, degrees=False)
                        backend.goto_target(head=head, duration=0.2)

            # Mesures
            for _ in range(iterations):
                t0 = time.perf_counter()
                result = vision.scan_environment()
                if result.get("objects") and len(result["objects"]) > 0:
                    if create_head_pose is not None:
                        head = create_head_pose(pitch=0.1, yaw=0.1, degrees=False)
                        backend.goto_target(head=head, duration=0.2)
                t1 = time.perf_counter()
                latencies_ms.append((t1 - t0) * 1000.0)

            if not latencies_ms:
                pytest.skip("Aucune mesure réussie")

            p50 = statistics.median(latencies_ms)
            p95 = float(np.percentile(latencies_ms, 95))
            mean = statistics.mean(latencies_ms)

            print("\n📊 Full Pipeline Latency Results (vision + movement):")
            print(f"  Mean: {mean:.2f} ms")
            print(f"  p50:  {p50:.2f} ms")
            print(f"  p95:  {p95:.2f} ms")

            # Budget end-to-end : < 150ms
            is_ci = os.environ.get("CI", "false").lower() == "true"
            max_p50 = 300.0 if is_ci else 150.0
            max_p95 = 500.0 if is_ci else 250.0

            assert (
                p50 < max_p50
            ), f"p50 pipeline trop élevée: {p50:.2f} ms (max: {max_p50})"
            assert (
                p95 < max_p95
            ), f"p95 pipeline trop élevée: {p95:.2f} ms (max: {max_p95})"

        finally:
            backend.disconnect()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
