#!/usr/bin/env python3
"""
Test stabilité buffers audio (underruns/overruns) pendant ~10 s.

Conditions de skip:
- BBIA_DISABLE_AUDIO=1
- sounddevice indisponible
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


@pytest.mark.unit
def test_audio_buffer_stability_10s() -> None:
    if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
        pytest.skip("Audio désactivé par BBIA_DISABLE_AUDIO=1")
    if sd is None:
        pytest.skip("sounddevice indisponible sur cet environnement")

    sample_rate = 16000
    blocksize = 512
    duration_s = 10.0

    phase = 0.0
    underruns = 0
    overruns = 0

    def callback(outdata, frames, time_info, status):  # type: ignore[no-redef]
        nonlocal phase, underruns, overruns
        if status.output_underflow:
            underruns += 1
        if status.input_overflow:
            overruns += 1

        t = (np.arange(frames) + phase) / sample_rate
        tone = (0.1 * np.sin(2 * np.pi * 440.0 * t)).astype(np.float32)
        outdata[:] = tone.reshape(-1, 1)
        phase += frames

    with sd.OutputStream(  # type: ignore[misc]
        samplerate=sample_rate,
        blocksize=blocksize,
        dtype="float32",
        channels=1,
        callback=callback,
    ):
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < duration_s:
            sd.sleep(50)

    # Objectif: aucun underrun/overrun dans un environnement stable
    # CI peut être bruyant; tolérance faible
    assert underruns == 0, f"Underruns détectés: {underruns}"
    assert overruns == 0, f"Overruns détectés: {overruns}"


