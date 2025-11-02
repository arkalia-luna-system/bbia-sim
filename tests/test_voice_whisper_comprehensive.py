#!/usr/bin/env python3
"""
Tests complets pour voice_whisper.py - Amélioration coverage 33% → 50%+
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

# Désactiver audio pour CI
os.environ["BBIA_DISABLE_AUDIO"] = "1"


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
            # Vider cache avant test
            import bbia_sim.voice_whisper as voice_whisper_module

            voice_whisper_module._whisper_models_cache.clear()

            mock_model = MagicMock()
            mock_whisper.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.load_model()

            assert result is True
            assert stt.is_loaded is True
            assert stt.model == mock_model

    @patch("bbia_sim.voice_whisper.whisper")
    def test_load_model_failure(self, mock_whisper):
        """Test chargement modèle échoué."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            # Vider cache avant test
            import bbia_sim.voice_whisper as voice_whisper_module

            voice_whisper_module._whisper_models_cache.clear()

            mock_whisper.load_model.side_effect = Exception("Erreur chargement")

            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.load_model()

            assert result is False
            assert stt.is_loaded is False

    def test_load_model_without_whisper(self):
        """Test load_model sans Whisper."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False):
            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.load_model()
            assert result is False

    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_audio_success(self, mock_whisper):
        """Test transcription fichier audio réussie."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            # Vider cache
            import bbia_sim.voice_whisper as voice_whisper_module

            voice_whisper_module._whisper_models_cache.clear()

            mock_model = MagicMock()
            mock_model.transcribe.return_value = {"text": "bonjour le monde"}
            mock_whisper.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="fr")
            stt.model = mock_model
            stt.is_loaded = True

            # Créer fichier audio temporaire
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                temp_path = f.name
                f.write(b"fake audio data")

            try:
                result = stt.transcribe_audio(temp_path)

                assert result == "bonjour le monde"
                mock_model.transcribe.assert_called_once()
            finally:
                Path(temp_path).unlink(missing_ok=True)

    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_audio_model_none(self, mock_whisper):
        """Test transcription avec modèle None (couverture ligne 148-150)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            stt = WhisperSTT(model_size="tiny", language="fr")
            stt.is_loaded = True
            stt.model = None

            result = stt.transcribe_audio("/fake/path.wav")

            assert result is None

    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_audio_exception(self, mock_whisper):
        """Test gestion exception transcription (couverture lignes 169-171)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_model = MagicMock()
            mock_model.transcribe.side_effect = Exception("Erreur transcription")
            mock_whisper.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="fr")
            stt.model = mock_model
            stt.is_loaded = True

            result = stt.transcribe_audio("/fake/path.wav")

            assert result is None

    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_audio_with_auto_load(self, mock_whisper):
        """Test transcription avec chargement automatique (couverture lignes 138-141)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            # Vider cache
            import bbia_sim.voice_whisper as voice_whisper_module

            voice_whisper_module._whisper_models_cache.clear()

            mock_model = MagicMock()
            mock_model.transcribe.return_value = {"text": "test"}
            mock_whisper.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="fr")
            assert stt.is_loaded is False

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                temp_path = f.name
                f.write(b"fake audio data")

            try:
                result = stt.transcribe_audio(temp_path)

                assert stt.is_loaded is True
                assert isinstance(result, str | type(None))
            finally:
                Path(temp_path).unlink(missing_ok=True)

    def test_transcribe_microphone_disabled_audio(self):
        """Test transcription microphone avec audio désactivé (couverture lignes 184-188)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.transcribe_microphone(duration=1.0)

            assert result is None

    @patch("bbia_sim.voice_whisper.transformers_pipeline")
    def test_detect_speech_activity_speech(self, mock_pipeline):
        """Test détection VAD - parole."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_vad = MagicMock()
            mock_vad.return_value = [{"label": "SPEECH", "score": 0.95}]
            mock_pipeline.return_value = mock_vad

            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)

            audio_chunk = np.random.rand(16000).astype(np.float32)
            result = stt.detect_speech_activity(audio_chunk)

            assert result is True

    @patch("bbia_sim.voice_whisper.transformers_pipeline")
    def test_detect_speech_activity_silence(self, mock_pipeline):
        """Test détection VAD - silence."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_vad = MagicMock()
            mock_vad.return_value = [{"label": "NO_SPEECH", "score": 0.95}]
            mock_pipeline.return_value = mock_vad

            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)

            audio_chunk = np.random.rand(16000).astype(np.float32)
            result = stt.detect_speech_activity(audio_chunk)

            assert result is False

    def test_detect_speech_activity_vad_disabled(self):
        """Test VAD désactivé (couverture ligne 254-255)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=False)

            audio_chunk = np.random.rand(16000).astype(np.float32)
            result = stt.detect_speech_activity(audio_chunk)

            assert result is True  # Si VAD désactivé, toujours True

    def test_detect_speech_activity_audio_disabled(self):
        """Test VAD avec audio désactivé (couverture lignes 258-259)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)

            audio_chunk = np.random.rand(16000).astype(np.float32)
            result = stt.detect_speech_activity(audio_chunk)

            assert result is False  # Audio désactivé = False

    @patch("bbia_sim.voice_whisper.transformers_pipeline")
    @patch("bbia_sim.voice_whisper.sf")
    def test_detect_speech_activity_file_path(self, mock_sf, mock_pipeline):
        """Test VAD avec chemin fichier (couverture lignes 312-314)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_vad = MagicMock()
            mock_vad.return_value = [{"label": "SPEECH", "score": 0.95}]
            mock_pipeline.return_value = mock_vad

            mock_sf.read.return_value = (
                np.random.rand(16000).astype(np.float32),
                16000,
            )

            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                temp_path = f.name

            try:
                result = stt.detect_speech_activity(temp_path)

                assert isinstance(result, bool)
                mock_sf.read.assert_called_once()
            finally:
                Path(temp_path).unlink(missing_ok=True)

    @patch("bbia_sim.voice_whisper.transformers_pipeline")
    def test_detect_speech_activity_small_chunk(self, mock_pipeline):
        """Test VAD avec chunk trop petit (couverture lignes 322-323)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            mock_vad = MagicMock()
            mock_vad.return_value = [{"label": "SPEECH", "score": 0.95}]
            mock_pipeline.return_value = mock_vad

            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)

            audio_chunk = np.random.rand(50).astype(np.float32)  # Trop petit
            result = stt.detect_speech_activity(audio_chunk)

            assert result is False

    @patch("bbia_sim.voice_whisper.transformers_pipeline")
    def test_detect_speech_activity_vad_model_none(self, mock_pipeline):
        """Test VAD avec modèle None (couverture lignes 326-327)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            # Vider cache VAD
            import bbia_sim.voice_whisper as voice_whisper_module

            voice_whisper_module._vad_model_cache = None

            # Mock pour que VAD échoue au chargement mais stt._vad_model reste None
            mock_pipeline.side_effect = Exception("Erreur chargement VAD")

            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)
            # Forcer _vad_model à None après échec chargement
            stt._vad_model = None
            stt._vad_loaded = False

            audio_chunk = np.random.rand(16000).astype(np.float32)
            result = stt.detect_speech_activity(audio_chunk)

            assert result is True  # Fallback

    def test_transcribe_microphone_with_vad_disabled_audio(self):
        """Test transcription microphone avec VAD et audio désactivé."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)
            result = stt.transcribe_microphone_with_vad(duration=1.0)

            assert result is None

    @patch("bbia_sim.voice_whisper.sd")
    @patch("bbia_sim.voice_whisper.sf")
    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_streaming_disabled_audio(self, mock_whisper, mock_sf, mock_sd):
        """Test streaming avec audio désactivé (couverture lignes 478-480)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            stt = WhisperSTT(model_size="tiny", language="fr")
            result = stt.transcribe_streaming(max_duration=0.5)

            assert result is None

    @patch("bbia_sim.voice_whisper.sd")
    @patch("bbia_sim.voice_whisper.sf")
    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_streaming_no_whisper(self, mock_whisper, mock_sf, mock_sd):
        """Test streaming sans Whisper (couverture lignes 483-485)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False):
            # Désactiver temporairement BBIA_DISABLE_AUDIO pour ce test
            original_value = os.environ.get("BBIA_DISABLE_AUDIO", "0")
            os.environ["BBIA_DISABLE_AUDIO"] = "0"

            try:
                stt = WhisperSTT(model_size="tiny", language="fr")
                result = stt.transcribe_streaming(max_duration=0.5)

                assert result is None
            finally:
                os.environ["BBIA_DISABLE_AUDIO"] = original_value

    @patch("bbia_sim.voice_whisper.sd")
    @patch("bbia_sim.voice_whisper.sf")
    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_streaming_exception(self, mock_whisper, mock_sf, mock_sd):
        """Test gestion exception streaming (couverture lignes 630-632)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            # Désactiver temporairement BBIA_DISABLE_AUDIO pour ce test
            original_value = os.environ.get("BBIA_DISABLE_AUDIO", "0")
            os.environ["BBIA_DISABLE_AUDIO"] = "0"

            try:
                mock_sd.rec.side_effect = ImportError("Erreur sounddevice")

                stt = WhisperSTT(model_size="tiny", language="fr")
                result = stt.transcribe_streaming(max_duration=0.5)

                assert result is None
            finally:
                os.environ["BBIA_DISABLE_AUDIO"] = original_value


@pytest.mark.unit
@pytest.mark.fast
class TestVoiceCommandMapper:
    """Tests pour VoiceCommandMapper."""

    def test_init(self):
        """Test initialisation VoiceCommandMapper."""
        mapper = VoiceCommandMapper()

        assert mapper is not None
        assert hasattr(mapper, "commands")

    def test_map_command_valid(self):
        """Test mapping commande valide."""
        mapper = VoiceCommandMapper()

        # Test commande valide (utiliser une commande qui existe vraiment)
        result = mapper.map_command("salue")
        assert result is not None
        assert "action" in result
        assert result["action"] == "greet"

    def test_map_command_invalid(self):
        """Test mapping commande invalide."""
        mapper = VoiceCommandMapper()

        result = mapper.map_command("commande inexistante")
        assert result is None


@pytest.mark.unit
@pytest.mark.fast
class TestFactoryFunctions:
    """Tests pour fonctions factory."""

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True)
    def test_create_whisper_stt_with_whisper(self):
        """Test création WhisperSTT avec Whisper."""
        stt = create_whisper_stt(model_size="tiny", language="fr")

        assert stt is not None
        assert isinstance(stt, WhisperSTT)

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False)
    def test_create_whisper_stt_without_whisper(self):
        """Test création sans Whisper."""
        stt = create_whisper_stt(model_size="tiny", language="fr")

        assert stt is None
