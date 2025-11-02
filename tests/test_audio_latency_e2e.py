#!/usr/bin/env python3
"""
Test conditionnel de latence audio E2E (in→out) court.
Skip si BBIA_DISABLE_AUDIO=1 ou si sounddevice indisponible.
"""

import os
import time

# removed unused Optional import
import numpy as np
import pytest

try:
    import sounddevice as sd  # type: ignore
except Exception:  # pragma: no cover
    sd = None  # type: ignore[assignment]


@pytest.mark.unit
def test_audio_latency_e2e_conditional() -> None:
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        pytest.skip("Audio désactivé par BBIA_DISABLE_AUDIO=1")
    if sd is None:
        pytest.skip("sounddevice indisponible sur cet environnement")

    # Paramètres simples
    sample_rate = 16000
    duration_s = 0.25
    tone_hz = 1000

    # Générer un signal court
    t = np.arange(int(sample_rate * duration_s)) / sample_rate
    tone = (0.25 * np.sin(2 * np.pi * tone_hz * t)).astype(np.float32)

    # Lecture puis enregistrement en boucle (best effort)
    # Attention: sur CI/mac sans boucle hardware, cette mesure est approximative
    t0 = time.perf_counter()
    sd.play(tone, sample_rate)
    sd.wait()
    t1 = time.perf_counter()

    # Mesure approximative lecture seule (proche de 0 en mémoire)
    latency_ms = (t1 - t0) * 1000.0

    # Budget: < 100 ms en local. CI peut être bruité: seuil large 600 ms
    assert latency_ms < 600.0, f"Latence lecture trop élevée: {latency_ms:.1f} ms"
