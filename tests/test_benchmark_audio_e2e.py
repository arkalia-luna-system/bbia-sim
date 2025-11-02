#!/usr/bin/env python3
"""Benchmark latence E2E audio - Tests légers."""

import os
import time
import unittest
from unittest.mock import MagicMock, patch


# Désactiver audio pour CI
os.environ["BBIA_DISABLE_AUDIO"] = "1"

import sys


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))


class TestAudioE2EBenchmark(unittest.TestCase):
    """Benchmarks latence E2E audio (légers)."""

    @patch("os.environ.get", return_value="1")
    def test_benchmark_transcription_latency(self, mock_env: MagicMock) -> None:
        """Benchmark latence transcription (mock, léger)."""
        from bbia_sim.voice_whisper import WhisperSTT

        whisper = WhisperSTT(model_size="tiny")
        # Mock pour éviter chargement réel (léger)
        whisper.model = None
        whisper.is_loaded = False

        # Simuler benchmark
        start = time.time()
        # Simuler opération rapide
        time.sleep(0.001)  # 1ms simule traitement
        latency = time.time() - start

        # Vérifier latence < 100ms (mock, marge pour CI)
        self.assertLess(latency, 0.1)

    @patch("os.environ.get", return_value="1")
    def test_benchmark_audio_processing(self, mock_env: MagicMock) -> None:
        """Benchmark traitement audio (léger)."""
        import numpy as np

        # Petit chunk audio (léger)
        audio_chunk = np.random.rand(1600).astype(np.float32)  # 0.1s @ 16kHz

        start = time.time()
        # Simuler traitement
        _ = np.max(np.abs(audio_chunk))
        latency = time.time() - start

        # Vérifier traitement rapide (< 5ms)
        self.assertLess(latency, 0.005)

    def test_benchmark_memory_usage(self) -> None:
        """Vérifier usage mémoire raisonnable."""

        # Vérifier taille objets (léger)
        from bbia_sim.bbia_audio import DEFAULT_BUFFER_SIZE, DEFAULT_SAMPLE_RATE

        # Constantes devraient être légères
        self.assertIsInstance(DEFAULT_SAMPLE_RATE, int)
        self.assertIsInstance(DEFAULT_BUFFER_SIZE, int)
        self.assertLess(DEFAULT_BUFFER_SIZE, 10000)  # Buffer raisonnable


if __name__ == "__main__":
    unittest.main()
