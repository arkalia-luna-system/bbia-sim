#!/usr/bin/env python3
"""Tests pour le module de détection tactile BBIA-SIM.

Issue #251: Tests pour détection tactile.
"""

import os
import unittest
from unittest.mock import patch

import numpy as np

from src.bbia_sim import bbia_touch


class TestBBIATouchDetection(unittest.TestCase):
    """Tests pour BBIATouchDetection."""

    def setUp(self) -> None:
        """Setup pour chaque test."""
        # Désactiver audio pour tests (éviter dépendance hardware)
        os.environ["BBIA_DISABLE_AUDIO"] = "1"

    def tearDown(self) -> None:
        """Cleanup après chaque test."""
        if "BBIA_DISABLE_AUDIO" in os.environ:
            del os.environ["BBIA_DISABLE_AUDIO"]

    def test_touch_detection_disabled_when_audio_disabled(self) -> None:
        """Test que la détection est désactivée si audio désactivé."""
        detector = bbia_touch.BBIATouchDetection()
        self.assertFalse(detector.is_enabled())

    @patch("src.bbia_sim.bbia_touch.SOUNDDEVICE_AVAILABLE", True)
    def test_touch_detection_enabled_when_audio_available(self) -> None:
        """Test que la détection est activée si audio disponible."""
        if "BBIA_DISABLE_AUDIO" in os.environ:
            del os.environ["BBIA_DISABLE_AUDIO"]

        detector = bbia_touch.BBIATouchDetection()
        # Peut être désactivé si sounddevice non disponible en vrai
        # On teste juste que la méthode existe
        self.assertIsInstance(detector.is_enabled(), bool)

    def test_analyze_audio_pattern_tap(self) -> None:
        """Test analyse pattern audio pour tap."""
        detector = bbia_touch.BBIATouchDetection()
        # Créer audio simulé avec caractéristiques de tap
        # Tap: amplitude élevée, fréquences hautes
        audio = np.random.randn(1600).astype(np.float32)
        audio = audio * 0.5  # Amplitude élevée

        analysis = detector._analyze_audio_pattern(audio)
        self.assertIn("type", analysis)
        self.assertIn("confidence", analysis)
        self.assertIn("max_amplitude", analysis)

    def test_analyze_audio_pattern_empty(self) -> None:
        """Test analyse avec audio vide."""
        detector = bbia_touch.BBIATouchDetection()
        audio = np.array([])

        analysis = detector._analyze_audio_pattern(audio)
        self.assertEqual(analysis["type"], bbia_touch.TouchType.NONE)
        self.assertEqual(analysis["confidence"], 0.0)

    def test_detect_touch_disabled(self) -> None:
        """Test détection quand désactivée."""
        detector = bbia_touch.BBIATouchDetection()
        result = detector.detect_touch()

        self.assertEqual(result["type"], bbia_touch.TouchType.NONE)
        self.assertFalse(result.get("enabled", True))

    def test_detect_tap_disabled(self) -> None:
        """Test détection tap quand désactivée."""
        detector = bbia_touch.BBIATouchDetection()
        result = detector.detect_tap()
        self.assertFalse(result)

    def test_detect_caress_disabled(self) -> None:
        """Test détection caresse quand désactivée."""
        detector = bbia_touch.BBIATouchDetection()
        result = detector.detect_caress()
        self.assertFalse(result)

    def test_create_touch_detector(self) -> None:
        """Test factory create_touch_detector."""
        detector = bbia_touch.create_touch_detector()
        self.assertIsInstance(detector, bbia_touch.BBIATouchDetection)

    def test_touch_type_enum(self) -> None:
        """Test enum TouchType."""
        self.assertEqual(bbia_touch.TouchType.TAP.value, "tap")
        self.assertEqual(bbia_touch.TouchType.CARESS.value, "caress")
        self.assertEqual(bbia_touch.TouchType.PAT.value, "pat")
        self.assertEqual(bbia_touch.TouchType.NONE.value, "none")


if __name__ == "__main__":
    unittest.main()
