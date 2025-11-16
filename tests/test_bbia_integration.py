#!/usr/bin/env python3
"""
Tests unitaires pour bbia_integration.py
Tests d'intégration BBIA ↔ Robot Reachy Mini
"""

from unittest.mock import patch

import pytest

# OPTIMISATION COVERAGE: Import au niveau module pour que coverage détecte le module
try:
    from bbia_sim.bbia_integration import BBIAIntegration

    BBIA_INTEGRATION_AVAILABLE = True
except ImportError:
    BBIA_INTEGRATION_AVAILABLE = False
    BBIAIntegration = None  # type: ignore[assignment,misc]


class TestBBIAIntegration:
    """Tests pour BBIAIntegration."""

    @pytest.mark.skipif(
        not BBIA_INTEGRATION_AVAILABLE,
        reason="Module bbia_integration non disponible",
    )
    def test_bbia_integration_class_exists(self):
        """Test que la classe BBIAIntegration existe."""
        assert BBIAIntegration is not None

    @pytest.mark.skipif(
        not BBIA_INTEGRATION_AVAILABLE,
        reason="Module bbia_integration non disponible",
    )
    def test_integration_initialization(self):
        """Test initialisation BBIAIntegration."""
        with patch("bbia_sim.bbia_integration.BBIAEmotions"):
            with patch("bbia_sim.bbia_integration.BBIAVision"):
                with patch("bbia_sim.bbia_integration.BBIABehaviorManager"):
                    with patch("bbia_sim.bbia_integration.SimulationService"):
                        integration = BBIAIntegration()

                        assert integration.is_active is False
                        assert integration.current_emotion == "neutral"
                        assert hasattr(integration, "emotion_mappings")
                        assert hasattr(integration, "reaction_config")

    @pytest.mark.skipif(
        not BBIA_INTEGRATION_AVAILABLE,
        reason="Module bbia_integration non disponible",
    )
    def test_emotion_mappings_structure(self):
        """Test structure mappings émotions."""
        with patch("bbia_sim.bbia_integration.BBIAEmotions"):
            with patch("bbia_sim.bbia_integration.BBIAVision"):
                with patch("bbia_sim.bbia_integration.BBIABehaviorManager"):
                    with patch("bbia_sim.bbia_integration.SimulationService"):
                        integration = BBIAIntegration()

                        assert isinstance(integration.emotion_mappings, dict)
                        assert "neutral" in integration.emotion_mappings
                        assert "happy" in integration.emotion_mappings
                        assert "sad" in integration.emotion_mappings

    @pytest.mark.skipif(
        not BBIA_INTEGRATION_AVAILABLE,
        reason="Module bbia_integration non disponible",
    )
    def test_reaction_config_structure(self):
        """Test structure reaction_config."""
        with patch("bbia_sim.bbia_integration.BBIAEmotions"):
            with patch("bbia_sim.bbia_integration.BBIAVision"):
                with patch("bbia_sim.bbia_integration.BBIABehaviorManager"):
                    with patch("bbia_sim.bbia_integration.SimulationService"):
                        integration = BBIAIntegration()

                        assert isinstance(integration.reaction_config, dict)
                        assert "face_detection" in integration.reaction_config
                        assert "object_detection" in integration.reaction_config
                        assert "sound_detection" in integration.reaction_config

    @pytest.mark.skipif(
        not BBIA_INTEGRATION_AVAILABLE,
        reason="Module bbia_integration non disponible",
    )
    def test_audio_functions_structure(self):
        """Test structure audio_functions."""
        with patch("bbia_sim.bbia_integration.BBIAEmotions"):
            with patch("bbia_sim.bbia_integration.BBIAVision"):
                with patch("bbia_sim.bbia_integration.BBIABehaviorManager"):
                    with patch("bbia_sim.bbia_integration.SimulationService"):
                        integration = BBIAIntegration()

                        assert isinstance(integration.audio_functions, dict)
                        assert "enregistrer" in integration.audio_functions
                        assert "lire" in integration.audio_functions
                        assert "detecter" in integration.audio_functions

    @pytest.mark.skipif(
        not BBIA_INTEGRATION_AVAILABLE,
        reason="Module bbia_integration non disponible",
    )
    def test_voice_functions_structure(self):
        """Test structure voice_functions."""
        with patch("bbia_sim.bbia_integration.BBIAEmotions"):
            with patch("bbia_sim.bbia_integration.BBIAVision"):
                with patch("bbia_sim.bbia_integration.BBIABehaviorManager"):
                    with patch("bbia_sim.bbia_integration.SimulationService"):
                        integration = BBIAIntegration()

                        assert isinstance(integration.voice_functions, dict)
                        assert "dire" in integration.voice_functions
                        assert "reconnaitre" in integration.voice_functions


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
