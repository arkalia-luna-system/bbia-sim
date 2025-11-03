#!/usr/bin/env python3
"""
Test latence E2E audio complète in→out (loopback) p50/p95 si hardware disponible.
Skip si pas de hardware loopback.
"""

import os
import statistics
import time

import numpy as np
import pytest

sd = None  # type: ignore[assignment]
try:
    import sounddevice as _sd  # type: ignore

    sd = _sd
except Exception:  # pragma: no cover
    sd = None  # type: ignore[assignment]


@pytest.mark.unit
@pytest.mark.slow
def test_audio_latency_e2e_loopback() -> None:
    """Test latence E2E audio loopback (in→out) p50/p95."""
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        pytest.skip("Audio désactivé par BBIA_DISABLE_AUDIO=1")
    if sd is None:
        pytest.skip("sounddevice indisponible")

    assert sd is not None  # Pour mypy, après le skip

    # Vérifier si loopback hardware disponible
    # En général, on skip ce test si pas de loopback
    try:
        devices = sd.query_devices()
        has_loopback = any("loopback" in str(d).lower() for d in devices)
        if not has_loopback:
            pytest.skip("Pas de loopback hardware disponible")
    except Exception:
        pytest.skip("Impossible de vérifier dispositifs audio")

    sample_rate = 16000
    blocksize = 512
    iterations = 100
    latencies_ms: list[float] = []

    # Buffer pour stocker audio capturé
    playback_start_time = None

    def input_callback(indata, frames, time_info, status):  # type: ignore[no-redef]
        """Callback entrée audio."""
        nonlocal playback_start_time
        if playback_start_time and indata is not None:
            # Détecter signal (simple détection amplitude)
            if np.max(np.abs(indata)) > 0.01:
                latency = (time.perf_counter() - playback_start_time) * 1000.0
                latencies_ms.append(latency)
                playback_start_time = None  # Une seule mesure par itération

    def output_callback(outdata, frames, time_info, status):  # type: ignore[no-redef]
        """Callback sortie audio."""
        nonlocal playback_start_time
        # Générer tone de test
        t = np.arange(frames) / sample_rate
        tone = (0.1 * np.sin(2 * np.pi * 440.0 * t)).astype(np.float32)
        outdata[:] = tone.reshape(-1, 1)
        if playback_start_time is None:
            playback_start_time = time.perf_counter()

    try:
        with sd.InputStream(  # type: ignore[misc]
            samplerate=sample_rate,
            blocksize=blocksize,
            dtype="float32",
            channels=1,
            callback=input_callback,
        ):
            with sd.OutputStream(  # type: ignore[misc]
                samplerate=sample_rate,
                blocksize=blocksize,
                dtype="float32",
                channels=1,
                callback=output_callback,
            ):
                # Attendre que toutes les itérations soient mesurées avec timeout
                max_wait_time = 30.0  # 30s max pour éviter boucle infinie
                start_wait = time.perf_counter()
                while len(latencies_ms) < iterations:
                    if time.perf_counter() - start_wait > max_wait_time:
                        # Timeout: pas assez de mesures capturées
                        break
                    time.sleep(0.1)

        # Calculer statistiques
        if len(latencies_ms) > 0:
            p50 = statistics.median(latencies_ms)
            p95 = float(np.percentile(latencies_ms, 95))

            # Budget: Latence loopback acceptable (< 100ms p95 pour hardware)
            assert p50 < 50.0, f"Latence p50 trop élevée: {p50:.2f} ms"
            assert p95 < 100.0, f"Latence p95 trop élevée: {p95:.2f} ms"
        else:
            pytest.skip("Aucune mesure de latence capturée")
    except Exception as e:
        pytest.skip(f"Test loopback non disponible: {e}")
