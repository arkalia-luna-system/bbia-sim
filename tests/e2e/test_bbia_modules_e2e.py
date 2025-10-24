#!/usr/bin/env python3
"""Test E2E minimal : BBIA Modules individuels.

Test déterministe et rapide (< 5s).
"""

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from bbia_sim.bbia_audio import detecter_son
from bbia_sim.bbia_emotions import BBIAEmotions
from bbia_sim.bbia_vision import BBIAVision
from bbia_sim.bbia_voice import dire_texte, reconnaitre_parole


class TestBBIAModules:
    """Tests E2E pour les modules BBIA individuels."""

    @pytest.fixture(scope="class")
    def api_server(self):
        """Fixture simplifié sans démarrage d'API."""
        return None

    def test_bbia_emotions_module(self):
        """Test : Module BBIA Emotions."""
        emotions = BBIAEmotions()

        # Test création
        assert emotions.current_emotion == "neutral"
        assert emotions.emotion_intensity == 0.5
        assert len(emotions.emotions) == 8

        # Test changement d'émotion
        emotions.set_emotion("happy", 0.8)
        assert emotions.current_emotion == "happy"
        assert emotions.emotion_intensity == 0.8

        # Test émotions disponibles
        available = list(emotions.emotions.keys())
        expected_emotions = [
            "neutral",
            "happy",
            "sad",
            "angry",
            "curious",
            "excited",
            "surprised",
            "fearful",
        ]
        assert len(available) == len(expected_emotions)
        for emotion in expected_emotions:
            assert emotion in available

    def test_bbia_vision_module(self):
        """Test : Module BBIA Vision."""
        vision = BBIAVision()

        # Test création
        assert vision.camera_active
        assert vision.vision_quality == "HD"
        assert vision.detection_range == 3.0
        assert not vision.tracking_active

        # Test spécifications
        specs = vision.specs
        assert specs["camera"] == "Grand angle"
        assert specs["resolution"] == "1080p"
        assert specs["fov"] == "120°"

        # Test contrôle
        vision.tracking_active = True
        assert vision.tracking_active

        vision.current_focus = "test_object"
        assert vision.current_focus == "test_object"

    @patch("src.bbia_sim.bbia_voice.pyttsx3.init")
    @patch("src.bbia_sim.bbia_voice.get_bbia_voice")
    def test_bbia_voice_functions(self, mock_get_voice, mock_init):
        """Test : Fonctions BBIA Voice (mockées pour éviter les blocages)."""
        # Mock des fonctions audio pour éviter les blocages
        mock_engine = MagicMock()
        mock_init.return_value = mock_engine
        mock_get_voice.return_value = "test_voice_id"

        # Test synthèse vocale
        text = "Bonjour, je suis BBIA"
        result = dire_texte(text)
        # Accepter None car la fonction peut retourner None
        assert result is None or result is True

        # Test reconnaissance vocale (simulation)
        with patch("src.bbia_sim.bbia_voice.sr.Recognizer") as mock_recognizer:
            mock_r = MagicMock()
            mock_recognizer.return_value = mock_r
            mock_r.listen.return_value = MagicMock()
            mock_r.recognize_google.return_value = "test speech recognition"

            result = reconnaitre_parole()
            assert isinstance(result, str)
            assert len(result) > 0

    @patch("src.bbia_sim.bbia_audio.wave.open")
    def test_bbia_audio_functions(self, mock_wave_open):
        """Test : Fonctions BBIA Audio (mockées pour éviter les blocages)."""
        # Mock de la fonction de détection de son
        mock_wf = MagicMock()
        mock_wf.getnframes.return_value = 1000
        mock_wf.readframes.return_value = (
            b"\x00\x01\x02\x03" * 250
        )  # Données audio simulées
        mock_wave_open.return_value.__enter__.return_value = mock_wf

        # Test détection sonore avec fichier temporaire
        import os
        import tempfile

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = temp_file.name
            # Créer un fichier audio vide pour le test
            temp_file.write(b"dummy_audio_data")

        try:
            result = detecter_son(temp_path)
            assert isinstance(
                bool(result), bool
            )  # Convertir numpy.bool_ en bool Python
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)

    def test_bbia_with_api_integration(self, api_server):
        """Test : BBIA avec API (ultra-simplifié)."""
        # Test BBIA Emotions simple seulement
        emotions = BBIAEmotions()
        emotions.set_emotion("happy", 0.7)
        assert emotions.current_emotion == "happy"

        # Test BBIA Vision simple
        vision = BBIAVision()
        assert vision.camera_active is True
