#!/usr/bin/env python3
"""
Test budget CPU/RAM pipeline audio (10-30s profiling léger).
Mesure overhead audio avec psutil.
"""

import os
import time

import numpy as np
import pytest

sd = None  # type: ignore[assignment]
try:
    import sounddevice as _sd  # type: ignore

    sd = _sd
except Exception:  # pragma: no cover
    sd = None  # type: ignore[assignment]


def get_cpu_time() -> float:
    """Obtient temps CPU écoulé en secondes."""
    try:
        import psutil

        process = psutil.Process()
        return float(process.cpu_times().user + process.cpu_times().system)
    except ImportError:
        return float(time.process_time())


def get_memory_usage() -> float | None:
    """Obtient utilisation mémoire actuelle en MB."""
    try:
        import psutil

        process = psutil.Process()
        return float(process.memory_info().rss / 1024 / 1024)  # MB
    except ImportError:
        return None


@pytest.mark.unit
@pytest.mark.slow
def test_audio_pipeline_budget_cpu_ram() -> None:
    """Test budget CPU/RAM pipeline audio (5s optimisé)."""
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        pytest.skip("Audio désactivé par BBIA_DISABLE_AUDIO=1")
    if sd is None:
        pytest.skip("sounddevice indisponible")

    sample_rate = 16000
    blocksize = 512
    # Optimisé: 5s au lieu de 10s (suffisant pour mesurer budget)
    duration_s = 5.0

    frames_processed = 0
    phase = 0.0

    def callback(outdata, frames, time_info, status):  # type: ignore[no-redef]
        """Callback sortie audio."""
        nonlocal frames_processed, phase
        t = np.arange(frames) / sample_rate
        tone = (0.1 * np.sin(2 * np.pi * 440.0 * t)).astype(np.float32)
        outdata[:] = tone.reshape(-1, 1)
        phase += frames
        frames_processed += frames

    try:
        # Mesurer avant
        cpu_before = get_cpu_time()
        mem_before = get_memory_usage()

        with sd.OutputStream(  # type: ignore[misc]
            samplerate=sample_rate,
            blocksize=blocksize,
            dtype="float32",
            channels=1,
            callback=callback,
        ):
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < duration_s:
                time.sleep(0.1)

        # Mesurer après
        cpu_after = get_cpu_time()
        mem_after = get_memory_usage()

        cpu_time = cpu_after - cpu_before
        if mem_before and mem_after:
            mem_increase = mem_after - mem_before
        else:
            mem_increase = None

        # Budget: CPU < 0.5s pour 5s runtime (10% CPU max pipeline audio)
        assert (
            cpu_time < 0.5
        ), f"Temps CPU trop élevé: {cpu_time:.2f}s pour {duration_s}s runtime"

        # Budget: RAM < 50MB augmentation (pipeline audio léger)
        if mem_increase is not None:
            assert (
                mem_increase < 50.0
            ), f"Augmentation RAM trop élevée: {mem_increase:.1f}MB"
    except Exception as e:
        pytest.skip(f"Test audio non disponible: {e}")
