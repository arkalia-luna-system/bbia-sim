#!/usr/bin/env python3
"""Tests pour les nouveaux exemples de modules avancés.

Tests unitaires pour vérifier que les exemples de modules avancés fonctionnent.
"""

import sys
from pathlib import Path
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestDemoModulesAdvanced:
    """Tests pour les exemples de modules avancés."""

    @patch("examples.demo_emotion_recognition.BBIAEmotionRecognition")
    def test_demo_emotion_recognition_import(self, mock_emotion_recognition):
        """Test import demo_emotion_recognition."""
        import examples.demo_emotion_recognition as demo_emotion_recognition

        assert hasattr(demo_emotion_recognition, "main")

    @patch("examples.demo_integration.MuJoCoBackend")
    @patch("examples.demo_integration.SimulationService")
    def test_demo_integration_import(self, mock_service, mock_backend):
        """Test import demo_integration."""
        import examples.demo_integration as demo_integration

        assert hasattr(demo_integration, "main")

    @patch("examples.demo_voice_advanced.MuJoCoBackend")
    def test_demo_voice_advanced_import(self, mock_backend):
        """Test import demo_voice_advanced."""
        import examples.demo_voice_advanced as demo_voice_advanced

        assert hasattr(demo_voice_advanced, "main")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
