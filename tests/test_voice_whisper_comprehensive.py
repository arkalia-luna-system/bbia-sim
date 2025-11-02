#!/usr/bin/env python3
"""
Tests complets pour voice_whisper.py - Amélioration coverage 33% → 70%+
"""

import os
import tempfile
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from bbia_sim.voice_whisper import (
    VoiceCommandMapper,
    WhisperSTT,
    create_whisper_stt,
)


@pytest.mark.unit
@pytest.mark.fast
class TestWhisperSTT:
    """Tests pour WhisperSTT."""

    def test_init_without_whisper(self):
        """Test initialisation sans Whisper disponible."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False):
            stt = WhisperSTT(model_size="tiny", language="fr")
            assert stt.model_size == "tiny"
            assert stt.language == "fr"
            assert stt.model is None
            assert stt.is_loaded is False

    def test_init_with_whisper(self):
        """Test initialisation avec Whisper disponible."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            stt = WhisperSTT(model_size="base", language="en")
            assert stt.model_size == "base"
            assert stt.language == "en"
            assert stt.model is None  # Pas encore chargé
            assert stt.is_loaded is False

    @patch("bbia_sim.voice_whisper.whisper")
    def test_load_model_success(self, mock_whisper):
        """Test chargement modèle réussi."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_model = MagicMock()
            mock_whisper.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.load_model()

            assert result is True
            assert stt.is_loaded is True
            assert stt.model == mock_model
            mock_whisper.load_model.assert_called_once_with("tiny")

    @patch("bbia_sim.voice_whisper.whisper")
    def test_load_model_failure(self, mock_whisper):
        """Test chargement modèle échoué."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_whisper.load_model.side_effect = Exception("Erreur chargement")

            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.load_model()

            assert result is False
            assert stt.is_loaded is False
            assert stt.model is None

    def test_load_model_without_whisper(self):
        """Test load_model sans Whisper."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False):
            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.load_model()
            assert result is False

    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_audio_success(self, mock_whisper):
        """Test transcription audio réussie."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_model = MagicMock()
            mock_model.transcribe.return_value = {
                "text": "Bonjour, comment allez-vous?"
            }
            mock_whisper.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="fr")
            stt.model = mock_model
            stt.is_loaded = True

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                audio_path = f.name

            try:
                result = stt.transcribe_audio(audio_path)
                assert result == "Bonjour, comment allez-vous?"
                mock_model.transcribe.assert_called_once()
            finally:
                Path(audio_path).unlink(missing_ok=True)

    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_audio_with_auto_load(self, mock_whisper):
        """Test transcription avec chargement automatique modèle."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_model = MagicMock()
            mock_model.transcribe.return_value = {"text": "Hello world"}
            mock_whisper.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="en")
            assert stt.is_loaded is False

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                audio_path = f.name

            try:
                result = stt.transcribe_audio(audio_path)
                assert result == "Hello world"
                assert stt.is_loaded is True
            finally:
                Path(audio_path).unlink(missing_ok=True)

    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_audio_failure(self, mock_whisper):
        """Test transcription échouée."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_model = MagicMock()
            mock_model.transcribe.side_effect = Exception("Erreur transcription")
            mock_whisper.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="fr")
            stt.model = mock_model
            stt.is_loaded = True

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                audio_path = f.name

            try:
                result = stt.transcribe_audio(audio_path)
                assert result is None
            finally:
                Path(audio_path).unlink(missing_ok=True)

    def test_transcribe_audio_without_whisper(self):
        """Test transcription sans Whisper."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False):
            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.transcribe_audio("/fake/path.wav")
            assert result is None

    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_audio_with_language_auto(self, mock_whisper_module):
        """Test transcription avec langue 'auto'."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_model = MagicMock()
            mock_model.transcribe.return_value = {"text": "Auto detected"}
            mock_whisper_module.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="auto")
            stt.model = mock_model
            stt.is_loaded = True

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                audio_path = f.name

            try:
                result = stt.transcribe_audio(audio_path)
                assert result == "Auto detected"
                # Vérifier que language=None est passé pour auto
                call_args = mock_model.transcribe.call_args
                assert call_args[1]["language"] is None
            finally:
                Path(audio_path).unlink(missing_ok=True)

    @patch("sounddevice.wait")
    @patch("sounddevice.rec")
    @patch("soundfile.write")
    @patch("bbia_sim.voice_whisper.whisper")
    @patch("os.environ.get")
    @patch("pathlib.Path.unlink")
    def test_transcribe_microphone_success(
        self,
        mock_unlink,
        mock_env_get,
        mock_whisper,
        mock_sf_write,
        mock_sd_rec,
        mock_sd_wait,
    ):
        """Test transcription microphone réussie."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            # Mock BBIA_DISABLE_AUDIO pour désactiver le skip
            mock_env_get.side_effect = lambda key, default: (
                "0" if key == "BBIA_DISABLE_AUDIO" else default
            )

            # Mock sounddevice
            mock_sd_rec.return_value = np.array([0.1, 0.2, 0.3], dtype=np.float32)
            mock_sd_wait.return_value = None

            # Mock soundfile
            mock_sf_write.return_value = None

            # Mock Whisper
            mock_model = MagicMock()
            mock_model.transcribe.return_value = {"text": "Message depuis microphone"}
            mock_whisper.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="fr")
            stt.model = mock_model
            stt.is_loaded = True

            # Mock tempfile pour éviter problème de chemin
            with patch("tempfile.gettempdir", return_value="/tmp"):
                with patch("os.getpid", return_value=12345):
                    with patch("time.time", return_value=1234567890.0):
                        result = stt.transcribe_microphone(duration=1.0)

            assert result == "Message depuis microphone"
            mock_sd_rec.assert_called_once()
            mock_sf_write.assert_called_once()

    def test_transcribe_microphone_audio_disabled(self):
        """Test transcription microphone avec audio désactivé."""
        with patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "1"}):
            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.transcribe_microphone(duration=1.0)
            assert result is None

    def test_transcribe_microphone_without_whisper(self):
        """Test transcription microphone sans Whisper."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False):
            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.transcribe_microphone(duration=1.0)
            assert result is None


@pytest.mark.unit
@pytest.mark.fast
class TestVoiceCommandMapper:
    """Tests pour VoiceCommandMapper."""

    def test_init(self):
        """Test initialisation mappeur."""
        mapper = VoiceCommandMapper()
        assert isinstance(mapper.commands, dict)
        assert len(mapper.commands) > 0
        assert "salue" in mapper.commands
        assert "hello" in mapper.commands

    def test_map_command_exact_match_fr(self):
        """Test mapping commande exacte français."""
        mapper = VoiceCommandMapper()
        result = mapper.map_command("salue")
        assert result is not None
        assert result["action"] == "greet"
        assert result["confidence"] == 1.0

    def test_map_command_exact_match_en(self):
        """Test mapping commande exacte anglais."""
        mapper = VoiceCommandMapper()
        result = mapper.map_command("hello")
        assert result is not None
        assert result["action"] == "greet"
        assert result["confidence"] == 1.0

    def test_map_command_partial_match(self):
        """Test mapping commande partielle."""
        mapper = VoiceCommandMapper()
        result = mapper.map_command("Bonjour, peux-tu saluer?")
        assert result is not None
        assert result["action"] == "greet"
        assert result["confidence"] == 0.8

    def test_map_command_not_found(self):
        """Test commande non trouvée."""
        mapper = VoiceCommandMapper()
        result = mapper.map_command("commande inconnue")
        assert result is None

    def test_map_command_empty(self):
        """Test commande vide."""
        mapper = VoiceCommandMapper()
        result = mapper.map_command("")
        assert result is None

    def test_map_command_none(self):
        """Test commande None."""
        mapper = VoiceCommandMapper()
        result = mapper.map_command(None)  # type: ignore[arg-type]
        assert result is None

    def test_map_command_case_insensitive(self):
        """Test commande insensible à la casse."""
        mapper = VoiceCommandMapper()
        result = mapper.map_command("SALUE")
        assert result is not None
        assert result["action"] == "greet"

    def test_map_command_multiple_commands(self):
        """Test mapping plusieurs commandes."""
        mapper = VoiceCommandMapper()
        commands = ["salue", "regarde-moi", "sois content", "réveille-toi"]
        for cmd in commands:
            result = mapper.map_command(cmd)
            assert result is not None
            assert "action" in result


@pytest.mark.unit
@pytest.mark.fast
class TestCreateWhisperSTT:
    """Tests pour create_whisper_stt."""

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True)
    def test_create_with_whisper(self):
        """Test création avec Whisper disponible."""
        stt = create_whisper_stt(model_size="tiny", language="fr")
        assert stt is not None
        assert isinstance(stt, WhisperSTT)
        assert stt.model_size == "tiny"
        assert stt.language == "fr"

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False)
    def test_create_without_whisper(self):
        """Test création sans Whisper."""
        stt = create_whisper_stt(model_size="tiny", language="fr")
        assert stt is None
