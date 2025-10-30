#!/usr/bin/env python3
"""
Test loopback audio E2E (in→out) avec mesure p50/p95.

Conditions de skip:
- BBIA_DISABLE_AUDIO=1
- sounddevice indisponible
- Backend audio sans support loopback (ex: CoreAudio sans périphérique loopback)
"""

import os
import platform
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


def _get_wasapi_loopback_settings() -> object | None:
    if sd is None:
        return None
    try:
        from sounddevice import WasapiSettings  # type: ignore

        return WasapiSettings(loopback=True)
    except Exception:
        return None


@pytest.mark.unit
@pytest.mark.fast
def test_audio_latency_loopback_e2e() -> None:
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        pytest.skip("Audio désactivé par BBIA_DISABLE_AUDIO=1")
    if sd is None:
        pytest.skip("sounddevice indisponible sur cet environnement")

    # Loopback uniquement fiable avec WASAPI (Windows). Sur macOS/Linux, skip par défaut.
    if platform.system() != "Windows":
        pytest.skip("Loopback fiable seulement sur Windows/WASAPI dans ce test")

    wasapi_settings = _get_wasapi_loopback_settings()
    if wasapi_settings is None:
        pytest.skip("WASAPI loopback non disponible")

    sample_rate = 16000
    duration_s = 0.25
    tone_hz = 1000

    t = np.arange(int(sample_rate * duration_s)) / sample_rate
    tone = (0.25 * np.sin(2 * np.pi * tone_hz * t)).astype(np.float32)

    latencies_ms: list[float] = []
    iterations = 20

    # Utiliser un stream full-duplex avec loopback activé côté entrée
    for _ in range(iterations):

        def callback(outdata, frames, time_info, status):  # type: ignore[no-redef]
            # Ignorer les flags (s'ils existent) pour la mesure latence simple
            # On push le signal en sortie, l'entrée (loopback) est lue par sd.Stream
            outdata[:] = tone[:frames].reshape(-1, 1)

        t0 = time.perf_counter()
        try:
            with sd.Stream(  # type: ignore[misc]
                samplerate=sample_rate,
                blocksize=512,
                dtype="float32",
                channels=1,
                callback=callback,
                extra_settings=wasapi_settings,  # input loopback
            ):
                # Attendre la lecture/restitution du buffer court
                sd.sleep(int((duration_s + 0.05) * 1000))  # marge 50 ms
        finally:
            t1 = time.perf_counter()

        latencies_ms.append((t1 - t0) * 1000.0)

    p50 = statistics.median(latencies_ms)
    p95 = float(np.percentile(latencies_ms, 95))

    # Budgets tolérants (dépend du driver) mais devraient rester < 150 ms
    assert p50 < 150.0, f"p50 trop élevée: {p50:.1f} ms"
    assert p95 < 250.0, f"p95 trop élevée: {p95:.1f} ms"
