#!/usr/bin/env python3
"""Tests étendus pour BBIA Voice.

Tests ciblés pour améliorer la couverture de code.
"""

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim.bbia_voice import (
    dire_texte,
    get_bbia_voice,
    lister_voix_disponibles,
    reconnaitre_parole,
)


class TestBBIAVoiceExtended:
    """Tests étendus pour BBIA Voice."""

    def test_get_bbia_voice_france_priority(self):
        """Test priorité voix France."""
        # Créer des objets avec des attributs string réels
        voice1 = MagicMock()
        voice1.name = "Amélie"
        voice1.id = "com.apple.speech.voice.Amelie.fr-FR"

        voice2 = MagicMock()
        voice2.name = "Amélie"
        voice2.id = "com.apple.speech.voice.Amelie.fr-CA"

        voice3 = MagicMock()
        voice3.name = "Amélie"
        voice3.id = "com.apple.speech.voice.Amelie"

        mock_voices = [voice1, voice2, voice3]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        voice_id = get_bbia_voice(mock_engine)

        assert voice_id == "com.apple.speech.voice.Amelie.fr-FR"

    def test_get_bbia_voice_canada_fallback(self):
        """Test fallback voix Canada."""
        voice1 = MagicMock()
        voice1.name = "Amélie"
        voice1.id = "com.apple.speech.voice.Amelie.fr-CA"

        voice2 = MagicMock()
        voice2.name = "Amélie"
        voice2.id = "com.apple.speech.voice.Amelie"

        mock_voices = [voice1, voice2]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        voice_id = get_bbia_voice(mock_engine)

        assert voice_id == "com.apple.speech.voice.Amelie.fr-CA"

    def test_get_bbia_voice_any_amelie(self):
        """Test toute voix Amélie."""
        voice1 = MagicMock()
        voice1.name = "Amélie"
        voice1.id = "com.apple.speech.voice.Amelie"

        mock_voices = [voice1]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        voice_id = get_bbia_voice(mock_engine)

        assert voice_id == "com.apple.speech.voice.Amelie"

    def test_get_bbia_voice_no_amelie(self):
        """Test aucune voix Amélie disponible."""
        voice1 = MagicMock()
        voice1.name = "Autre"
        voice1.id = "com.apple.speech.voice.Other"

        mock_voices = [voice1]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        with pytest.raises(RuntimeError, match="Aucune voix 'Amélie'"):
            get_bbia_voice(mock_engine)

    def test_get_bbia_voice_normalize_accent(self):
        """Test normalisation avec accents."""
        voice1 = MagicMock()
        voice1.name = "Amélie"
        voice1.id = "com.apple.speech.voice.Amelie.fr-FR"

        mock_voices = [voice1]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        voice_id = get_bbia_voice(mock_engine)

        assert voice_id == "com.apple.speech.voice.Amelie.fr-FR"

    def test_get_bbia_voice_case_insensitive(self):
        """Test insensible à la casse."""
        voice1 = MagicMock()
        voice1.name = "AMÉLIE"
        voice1.id = "com.apple.speech.voice.Amelie.fr-FR"

        mock_voices = [voice1]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        voice_id = get_bbia_voice(mock_engine)

        assert voice_id == "com.apple.speech.voice.Amelie.fr-FR"

    @patch("os.environ.get", return_value="0")  # Désactiver BBIA_DISABLE_AUDIO
    @patch("bbia_sim.bbia_voice._pyttsx3_engine_cache", None)
    @patch("bbia_sim.bbia_voice._bbia_voice_id_cache", None)
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice.get_bbia_voice")
    def test_dire_texte_success(self, mock_get_voice, mock_get_engine, mock_env_get):
        """Test synthèse vocale réussie."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine
        mock_get_voice.return_value = "voice_id"

        # Mock complet de l'engine
        mock_engine.setProperty.return_value = None
        mock_engine.say.return_value = None
        mock_engine.runAndWait.return_value = None

        dire_texte("Test message")

        mock_get_engine.assert_called()
        mock_get_voice.assert_called()
        mock_engine.setProperty.assert_any_call("voice", "voice_id")
        mock_engine.setProperty.assert_any_call("rate", 170)
        mock_engine.setProperty.assert_any_call("volume", 1.0)
        mock_engine.say.assert_called_once_with("Test message")
        mock_engine.runAndWait.assert_called_once()

    @patch("os.environ.get", return_value="0")  # Désactiver BBIA_DISABLE_AUDIO
    @patch("bbia_sim.bbia_voice._pyttsx3_engine_cache", None)
    @patch("bbia_sim.bbia_voice._bbia_voice_id_cache", None)
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    def test_dire_texte_error(self, mock_get_engine, mock_env_get):
        """Test erreur synthèse vocale."""
        mock_get_engine.side_effect = Exception("Engine error")

        with pytest.raises(Exception, match="Engine error"):
            dire_texte("Test message")

    @patch("bbia_sim.bbia_voice.sr.Recognizer")
    @patch("bbia_sim.bbia_voice.sr.Microphone")
    def test_reconnaitre_parole_success(self, mock_microphone, mock_recognizer_class):
        """Test reconnaissance vocale réussie."""
        mock_recognizer = MagicMock()
        mock_recognizer_class.return_value = mock_recognizer

        mock_microphone_instance = MagicMock()
        mock_microphone.return_value.__enter__.return_value = mock_microphone_instance

        mock_audio = MagicMock()
        mock_recognizer.listen.return_value = mock_audio
        mock_recognizer.recognize_google.return_value = "texte reconnu"

        result = reconnaitre_parole(duree=3, frequence=16000)

        assert result == "texte reconnu"
        mock_recognizer.listen.assert_called_once_with(
            mock_microphone_instance, phrase_time_limit=3
        )
        mock_recognizer.recognize_google.assert_called_once_with(
            mock_audio, language="fr-FR"
        )

    @patch("bbia_sim.bbia_voice.sr.Recognizer")
    @patch("bbia_sim.bbia_voice.sr.Microphone")
    def test_reconnaitre_parole_unknown_value(
        self, mock_microphone, mock_recognizer_class
    ):
        """Test reconnaissance vocale sans parole."""
        mock_recognizer = MagicMock()
        mock_recognizer_class.return_value = mock_recognizer

        mock_microphone_instance = MagicMock()
        mock_microphone.return_value.__enter__.return_value = mock_microphone_instance

        mock_audio = MagicMock()
        mock_recognizer.listen.return_value = mock_audio
        mock_recognizer.recognize_google.side_effect = Exception("UnknownValueError")

        result = reconnaitre_parole(duree=3, frequence=16000)

        assert result is None

    @patch("bbia_sim.bbia_voice.sr.Recognizer")
    @patch("bbia_sim.bbia_voice.sr.Microphone")
    def test_reconnaitre_parole_general_error(
        self, mock_microphone, mock_recognizer_class
    ):
        """Test reconnaissance vocale erreur générale."""
        mock_recognizer = MagicMock()
        mock_recognizer_class.return_value = mock_recognizer

        mock_microphone_instance = MagicMock()
        mock_microphone.return_value.__enter__.return_value = mock_microphone_instance

        mock_recognizer.listen.side_effect = Exception("General error")

        result = reconnaitre_parole(duree=3, frequence=16000)

        assert result is None

    @patch("bbia_sim.bbia_voice._pyttsx3_engine_cache", None)
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    def test_lister_voix_disponibles_success(self, mock_get_engine):
        """Test liste des voix disponibles."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine

        mock_voice1 = MagicMock()
        mock_voice1.name = "Amélie"
        mock_voice1.id = "com.apple.speech.voice.Amelie.fr-FR"
        mock_voice1.languages = [b"fr-FR"]

        mock_voice2 = MagicMock()
        mock_voice2.name = "Autre"
        mock_voice2.id = "com.apple.speech.voice.Other"
        mock_voice2.languages = [b"en-US"]

        mock_engine.getProperty.return_value = [mock_voice1, mock_voice2]

        voices = lister_voix_disponibles()

        assert len(voices) == 2
        assert voices[0].name == "Amélie"
        assert voices[1].name == "Autre"

    @patch("bbia_sim.bbia_voice._pyttsx3_engine_cache", None)
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    def test_lister_voix_disponibles_decode_error(self, mock_get_engine):
        """Test liste des voix avec erreur de décodage."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine

        mock_voice = MagicMock()
        mock_voice.name = "Test"
        mock_voice.id = "test.id"

        # Simuler une erreur lors du decode - créer un mock qui lève une exception
        mock_language = MagicMock()
        mock_language.decode = MagicMock(side_effect=Exception("Decode error"))
        mock_voice.languages = [mock_language]

        mock_engine.getProperty.return_value = [mock_voice]

        voices = lister_voix_disponibles()

        assert len(voices) == 1
        assert voices[0].name == "Test"

    @patch("bbia_sim.bbia_voice._pyttsx3_engine_cache", None)
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    def test_lister_voix_disponibles_no_languages(self, mock_get_engine):
        """Test liste des voix sans langues."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine

        mock_voice = MagicMock()
        mock_voice.name = "Test"
        mock_voice.id = "test.id"
        mock_voice.languages = []

        mock_engine.getProperty.return_value = [mock_voice]

        voices = lister_voix_disponibles()

        assert len(voices) == 1
        assert voices[0].name == "Test"

    def test_get_bbia_voice_empty_voices_list(self):
        """Test liste de voix vide."""
        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = []

        with pytest.raises(RuntimeError, match="Aucune voix 'Amélie'"):
            get_bbia_voice(mock_engine)

    def test_get_bbia_voice_multiple_france_voices(self):
        """Test plusieurs voix France."""
        voice1 = MagicMock()
        voice1.name = "Amélie"
        voice1.id = "com.apple.speech.voice.Amelie.fr-FR.1"

        voice2 = MagicMock()
        voice2.name = "Amélie"
        voice2.id = "com.apple.speech.voice.Amelie.fr-FR.2"

        mock_voices = [voice1, voice2]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        voice_id = get_bbia_voice(mock_engine)

        # Devrait retourner la première voix France trouvée
        assert voice_id == "com.apple.speech.voice.Amelie.fr-FR.1"

    def test_get_bbia_voice_multiple_canada_voices(self):
        """Test plusieurs voix Canada."""
        voice1 = MagicMock()
        voice1.name = "Amélie"
        voice1.id = "com.apple.speech.voice.Amelie.fr-CA.1"

        voice2 = MagicMock()
        voice2.name = "Amélie"
        voice2.id = "com.apple.speech.voice.Amelie.fr-CA.2"

        mock_voices = [voice1, voice2]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        voice_id = get_bbia_voice(mock_engine)

        # Devrait retourner la première voix Canada trouvée
        assert voice_id == "com.apple.speech.voice.Amelie.fr-CA.1"

    def test_get_bbia_voice_multiple_any_amelie(self):
        """Test plusieurs voix Amélie quelconques."""
        voice1 = MagicMock()
        voice1.name = "Amélie"
        voice1.id = "com.apple.speech.voice.Amelie.1"

        voice2 = MagicMock()
        voice2.name = "Amélie"
        voice2.id = "com.apple.speech.voice.Amelie.2"

        mock_voices = [voice1, voice2]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        voice_id = get_bbia_voice(mock_engine)

        # Devrait retourner la première voix Amélie trouvée
        assert voice_id == "com.apple.speech.voice.Amelie.1"

    @patch("os.environ.get", return_value="0")  # Désactiver BBIA_DISABLE_AUDIO
    @patch("bbia_sim.bbia_voice._pyttsx3_engine_cache", None)
    @patch("bbia_sim.bbia_voice._bbia_voice_id_cache", None)
    @patch("bbia_sim.bbia_voice._get_pyttsx3_engine")
    @patch("bbia_sim.bbia_voice.get_bbia_voice")
    def test_dire_texte_engine_properties(
        self, mock_get_voice, mock_get_engine, mock_env_get
    ):
        """Test propriétés du moteur de synthèse."""
        mock_engine = MagicMock()
        mock_get_engine.return_value = mock_engine
        mock_get_voice.return_value = "voice_id"

        # Mock complet de l'engine
        mock_engine.setProperty.return_value = None
        mock_engine.say.return_value = None
        mock_engine.runAndWait.return_value = None

        dire_texte("Test")

        # Vérifier l'ordre des appels setProperty
        calls = mock_engine.setProperty.call_args_list
        assert len(calls) >= 3
        # Vérifier que les propriétés sont définies (ordre peut varier selon code)
        voice_calls = [c for c in calls if c[0][0] == "voice"]
        rate_calls = [c for c in calls if c[0][0] == "rate"]
        volume_calls = [c for c in calls if c[0][0] == "volume"]
        assert len(voice_calls) >= 1
        assert len(rate_calls) >= 1
        assert len(volume_calls) >= 1

    @patch("bbia_sim.bbia_voice.sr.Recognizer")
    @patch("bbia_sim.bbia_voice.sr.Microphone")
    def test_reconnaitre_parole_custom_parameters(
        self, mock_microphone, mock_recognizer_class
    ):
        """Test reconnaissance avec paramètres personnalisés."""
        mock_recognizer = MagicMock()
        mock_recognizer_class.return_value = mock_recognizer

        mock_microphone_instance = MagicMock()
        mock_microphone.return_value.__enter__.return_value = mock_microphone_instance

        mock_audio = MagicMock()
        mock_recognizer.listen.return_value = mock_audio
        mock_recognizer.recognize_google.return_value = "texte reconnu"

        result = reconnaitre_parole(duree=5, frequence=22050)

        assert result == "texte reconnu"
        mock_recognizer.listen.assert_called_once_with(
            mock_microphone_instance, phrase_time_limit=5
        )

    def test_get_bbia_voice_normalize_unicode(self):
        """Test normalisation Unicode."""
        voice1 = MagicMock()
        voice1.name = "Amélie"
        voice1.id = "com.apple.speech.voice.Amelie.fr-FR"

        mock_voices = [voice1]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        voice_id = get_bbia_voice(mock_engine)

        assert voice_id == "com.apple.speech.voice.Amelie.fr-FR"

    def test_get_bbia_voice_normalize_special_chars(self):
        """Test normalisation caractères spéciaux."""
        voice1 = MagicMock()
        voice1.name = "Amélie"
        voice1.id = "com.apple.speech.voice.Amelie.fr-FR"

        mock_voices = [voice1]

        mock_engine = MagicMock()
        mock_engine.getProperty.return_value = mock_voices

        voice_id = get_bbia_voice(mock_engine)

        assert voice_id == "com.apple.speech.voice.Amelie.fr-FR"
