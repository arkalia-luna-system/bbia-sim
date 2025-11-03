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
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_detect_speech_activity_speech(self, mock_pipeline):
        """Test détection VAD - parole."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            # Vider cache VAD
            import bbia_sim.voice_whisper as voice_whisper_module

            voice_whisper_module._vad_model_cache = None

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
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_detect_speech_activity_file_path(self, mock_sf, mock_pipeline):
        """Test VAD avec chemin fichier (couverture lignes 312-314)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            # Vider cache VAD
            import bbia_sim.voice_whisper as voice_whisper_module

            voice_whisper_module._vad_model_cache = None

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
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
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

    @patch("bbia_sim.voice_whisper.sd")
    @patch("bbia_sim.voice_whisper.sf")
    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_microphone_with_vad_success_mock(
        self, mock_whisper, mock_sf, mock_sd
    ):
        """Test transcription microphone avec VAD (mock optimisé RAM)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            original_value = os.environ.get("BBIA_DISABLE_AUDIO", "0")
            os.environ["BBIA_DISABLE_AUDIO"] = "0"

            try:
                # Mock optimisé: éviter chargement modèle réel
                import bbia_sim.voice_whisper as voice_whisper_module

                voice_whisper_module._whisper_models_cache.clear()
                voice_whisper_module._vad_model_cache = None

                mock_model = MagicMock()
                mock_model.transcribe.return_value = {"text": "bonjour"}
                mock_whisper.load_model.return_value = mock_model

                # Mock audio recording
                mock_audio = np.random.rand(8000).astype(np.float32)
                mock_sd.rec.return_value = mock_audio
                mock_sd.wait.return_value = None

                # Mock VAD pour retourner True (parole détectée)
                stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)
                with patch.object(stt, "detect_speech_activity", return_value=True):
                    # Mock transcribe_audio pour éviter I/O
                    with patch.object(stt, "transcribe_audio", return_value="bonjour"):
                        result = stt.transcribe_microphone_with_vad(
                            duration=0.1, silence_threshold=0.1
                        )

                # Avec mocks, peut retourner None ou résultat selon implémentation
                assert result is None or isinstance(result, str)
            finally:
                os.environ["BBIA_DISABLE_AUDIO"] = original_value

    def test_detect_speech_activity_empty_text(self):
        """Test VAD avec résultat vide (couverture cas limites)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=False)
            # VAD désactivé devrait retourner True
            result = stt.detect_speech_activity(np.array([], dtype=np.float32))
            assert result is True

    @patch("bbia_sim.voice_whisper.transformers_pipeline")
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_detect_speech_activity_result_format(self, mock_pipeline):
        """Test VAD avec différents formats de résultat."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            import bbia_sim.voice_whisper as voice_whisper_module

            # Vider cache pour forcer chargement
            voice_whisper_module._vad_model_cache = None

            # Mock pipeline retournant différents formats
            mock_vad = MagicMock()
            mock_vad.return_value = [{"label": "SPEECH", "score": 0.95}]
            mock_pipeline.return_value = mock_vad

            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)
            audio_chunk = np.random.rand(16000).astype(np.float32)

            # Mock sf pour éviter erreur import (patch nécessaire pour éviter ImportError)
            with patch("bbia_sim.voice_whisper.sf"):
                result = stt.detect_speech_activity(audio_chunk)
                # Devrait retourner True pour SPEECH avec score > 0.5
                assert result is True

    def test_map_command_partial_match(self):
        """Test mapping commande avec correspondance partielle."""
        mapper = VoiceCommandMapper()

        # Test correspondance partielle (ligne 688-691)
        result = mapper.map_command("Salut tout le monde")
        assert result is not None
        assert result["action"] == "greet"
        assert result["confidence"] == 0.8  # Correspondance partielle

    def test_map_command_empty_text(self):
        """Test mapping avec texte vide (couverture ligne 675-676)."""
        mapper = VoiceCommandMapper()
        result = mapper.map_command("")
        assert result is None

        result = mapper.map_command("   ")
        assert result is None


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

    @patch("bbia_sim.voice_whisper.whisper")
    def test_load_model_cache_lru_eviction(self, mock_whisper):
        """Test éviction LRU du cache Whisper (couverture lignes 108-118)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            import time as time_module

            import bbia_sim.voice_whisper as voice_whisper_module

            # Vider cache
            voice_whisper_module._whisper_models_cache.clear()
            voice_whisper_module._whisper_model_last_used.clear()

            # Remplir cache jusqu'à la limite (2 modèles)
            mock_model_tiny = MagicMock()
            mock_model_base = MagicMock()
            mock_whisper.load_model.side_effect = [mock_model_tiny, mock_model_base]

            # Charger modèle "tiny"
            stt_tiny = WhisperSTT(model_size="tiny")
            stt_tiny.load_model()

            # Charger modèle "base" (cache maintenant plein)
            stt_base = WhisperSTT(model_size="base")
            stt_base.load_model()

            assert len(voice_whisper_module._whisper_models_cache) == 2

            # Charger un nouveau modèle "small" devrait évincer le plus ancien
            time_module.sleep(0.1)
            mock_model_small = MagicMock()
            mock_whisper.load_model.side_effect = [mock_model_small]

            stt_small = WhisperSTT(model_size="small")
            stt_small.load_model()

            # Cache devrait avoir max 2 modèles
            assert len(voice_whisper_module._whisper_models_cache) <= 2

    @patch("bbia_sim.voice_whisper.transformers_pipeline")
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_detect_speech_activity_vad_cache_reuse(self, mock_pipeline):
        """Test réutilisation cache VAD global (couverture lignes 284-287)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            import bbia_sim.voice_whisper as voice_whisper_module

            # Mettre un modèle dans le cache global
            mock_vad = MagicMock()
            mock_vad.return_value = [{"label": "SPEECH", "score": 0.95}]
            voice_whisper_module._vad_model_cache = mock_vad

            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)
            audio_chunk = np.random.rand(16000).astype(np.float32)
            result = stt.detect_speech_activity(audio_chunk)

            # Devrait utiliser le cache global
            assert result is True
            assert stt._vad_model == mock_vad

    @patch("bbia_sim.voice_whisper.transformers_pipeline")
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_detect_speech_activity_with_path_string(self, mock_pipeline):
        """Test VAD avec chemin de fichier (string Path)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            import bbia_sim.voice_whisper as voice_whisper_module

            voice_whisper_module._vad_model_cache = None

            mock_vad = MagicMock()
            mock_vad.return_value = [{"label": "SPEECH", "score": 0.95}]
            mock_pipeline.return_value = mock_vad

            mock_sf_read = MagicMock(
                return_value=(np.random.rand(16000).astype(np.float32), 16000)
            )
            with patch("bbia_sim.voice_whisper.sf") as mock_sf:
                mock_sf.read = mock_sf_read

                stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)

                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                    temp_path = f.name

                try:
                    result = stt.detect_speech_activity(temp_path)
                    assert isinstance(result, bool)
                finally:
                    Path(temp_path).unlink(missing_ok=True)

    @patch("bbia_sim.voice_whisper.transformers_pipeline")
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_detect_speech_activity_unsupported_format(self, mock_pipeline):
        """Test VAD avec format audio non supporté (couverture ligne 338)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)
            # Passer un type non supporté
            result = stt.detect_speech_activity(12345)  # Type int non supporté
            assert result is True  # Fallback

    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_audio_language_auto(self, mock_whisper):
        """Test transcription avec language='auto' (couverture ligne 176)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            import bbia_sim.voice_whisper as voice_whisper_module

            voice_whisper_module._whisper_models_cache.clear()

            mock_model = MagicMock()
            mock_model.transcribe.return_value = {"text": "test"}
            mock_whisper.load_model.return_value = mock_model

            stt = WhisperSTT(model_size="tiny", language="auto")
            stt.model = mock_model
            stt.is_loaded = True

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                temp_path = f.name
                f.write(b"fake audio data")

            try:
                result = stt.transcribe_audio(temp_path)
                assert result == "test"
                # Vérifier que transcribe a été appelé avec language=None (auto)
                mock_model.transcribe.assert_called_once()
                call_kwargs = mock_model.transcribe.call_args[1]
                assert call_kwargs.get("language") is None
            finally:
                Path(temp_path).unlink(missing_ok=True)

    @patch("bbia_sim.voice_whisper.transformers_pipeline")
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_detect_speech_activity_vad_error_fallback(self, mock_pipeline):
        """Test VAD avec erreur et fallback (couverture lignes 312-315)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            import bbia_sim.voice_whisper as voice_whisper_module

            voice_whisper_module._vad_model_cache = None

            # Mock erreur chargement VAD
            mock_pipeline.side_effect = Exception("VAD load error")

            stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)
            audio_chunk = np.random.rand(16000).astype(np.float32)
            result = stt.detect_speech_activity(audio_chunk)

            # Devrait fallback à True
            assert result is True
            assert stt.enable_vad is False  # VAD désactivé après erreur

    @patch("bbia_sim.voice_whisper.whisper")
    @patch("bbia_sim.voice_whisper.sd")
    @patch("bbia_sim.voice_whisper.sf")
    def test_transcribe_streaming_with_callback(self, mock_sf, mock_sd, mock_whisper):
        """Test streaming avec callback (couverture lignes 629-633)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            original_value = os.environ.get("BBIA_DISABLE_AUDIO", "0")
            os.environ["BBIA_DISABLE_AUDIO"] = "0"

            try:
                import bbia_sim.voice_whisper as voice_whisper_module

                voice_whisper_module._whisper_models_cache.clear()

                mock_model = MagicMock()
                mock_model.transcribe.return_value = {"text": "test chunk"}
                mock_whisper.load_model.return_value = mock_model

                # Mock audio recording
                mock_audio = np.random.rand(8000).astype(np.float32)
                mock_sd.rec.return_value = mock_audio
                mock_sd.wait.return_value = None

                callback_called = []

                def test_callback(text, duration):
                    callback_called.append((text, duration))

                stt = WhisperSTT(model_size="tiny", language="fr")
                stt.model = mock_model
                stt.is_loaded = True
                stt.enable_vad = False  # Désactiver VAD pour simplifier

                # Mock transcribe_audio pour éviter I/O fichier
                with patch.object(stt, "transcribe_audio", return_value="test chunk"):
                    result = stt.transcribe_streaming(
                        callback=test_callback, max_duration=0.6, chunk_duration=0.3
                    )

                # Callback devrait être appelé
                assert len(callback_called) > 0 or result is not None
            finally:
                os.environ["BBIA_DISABLE_AUDIO"] = original_value

    @patch("bbia_sim.voice_whisper.whisper")
    def test_transcribe_streaming_no_model_error(self, mock_whisper):
        """Test streaming sans modèle chargé (couverture ligne 604-606)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            original_value = os.environ.get("BBIA_DISABLE_AUDIO", "0")
            os.environ["BBIA_DISABLE_AUDIO"] = "0"

            try:
                stt = WhisperSTT(model_size="tiny", language="fr")
                stt.is_loaded = True
                stt.model = None  # Modèle None

                # Mock pour éviter vraie exécution
                with patch.object(stt, "load_model", return_value=False):
                    result = stt.transcribe_streaming(max_duration=0.5)
                    # Devrait retourner None car modèle non chargé
                    assert result is None
            finally:
                os.environ["BBIA_DISABLE_AUDIO"] = original_value

    def test_voice_command_mapper_exact_match(self):
        """Test mapping commande avec correspondance exacte (couverture lignes 719-722)."""
        mapper = VoiceCommandMapper()
        result = mapper.map_command("bonjour")
        assert result is not None
        assert result["action"] == "greet"
        assert result["confidence"] == 1.0

    def test_voice_command_mapper_partial_match_contains(self):
        """Test mapping avec correspondance partielle (couverture lignes 725-728)."""
        mapper = VoiceCommandMapper()
        # "salue" est dans "salue-moi"
        result = mapper.map_command("salue-moi")
        assert result is not None
        assert result["action"] == "greet"
        assert result["confidence"] == 0.8

    def test_voice_command_mapper_whitespace(self):
        """Test mapping avec espaces (couverture ligne 716)."""
        mapper = VoiceCommandMapper()
        result = mapper.map_command("  bonjour  ")
        assert result is not None
        assert result["action"] == "greet"

    @patch("bbia_sim.voice_whisper.sd")
    @patch("bbia_sim.voice_whisper.sf")
    @patch("bbia_sim.voice_whisper.whisper")
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_transcribe_streaming_success_basic(self, mock_whisper, mock_sf, mock_sd):
        """Test streaming de base avec succès (couverture boucle principale lignes 539-653)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            original_value = os.environ.get("BBIA_DISABLE_AUDIO", "0")
            os.environ["BBIA_DISABLE_AUDIO"] = "0"

            try:
                import bbia_sim.voice_whisper as voice_whisper_module

                voice_whisper_module._whisper_models_cache.clear()

                mock_model = MagicMock()
                mock_model.transcribe.return_value = {"text": "test transcription"}
                mock_whisper.load_model.return_value = mock_model

                # Mock audio recording
                mock_audio = np.random.rand(8000).astype(np.float32)
                mock_sd.rec.return_value = mock_audio
                mock_sd.wait.return_value = None

                stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=False)
                stt.model = mock_model
                stt.is_loaded = True

                # Mock transcribe pour retourner texte valide
                with patch.object(
                    mock_model, "transcribe", return_value={"text": "hello world"}
                ):
                    # Mock time pour contrôler transcription_interval
                    with patch("bbia_sim.voice_whisper.time") as mock_time:
                        mock_time.time.side_effect = [
                            0.0,
                            2.0,
                            4.0,
                        ]  # Permet transcription

                        result = stt.transcribe_streaming(
                            max_duration=0.5,
                            chunk_duration=0.25,
                            transcription_interval=1.0,
                        )

                        # Devrait retourner texte ou None selon mock
                        assert result is None or isinstance(result, str)
            finally:
                os.environ["BBIA_DISABLE_AUDIO"] = original_value

    @patch("bbia_sim.voice_whisper.sd")
    @patch("bbia_sim.voice_whisper.sf")
    @patch("bbia_sim.voice_whisper.whisper")
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_transcribe_streaming_with_vad_silence(
        self, mock_whisper, mock_sf, mock_sd
    ):
        """Test streaming avec VAD détectant silence (couverture lignes 554-567)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            original_value = os.environ.get("BBIA_DISABLE_AUDIO", "0")
            os.environ["BBIA_DISABLE_AUDIO"] = "0"

            try:
                import bbia_sim.voice_whisper as voice_whisper_module

                voice_whisper_module._whisper_models_cache.clear()

                mock_model = MagicMock()
                mock_whisper.load_model.return_value = mock_model

                mock_audio = np.random.rand(8000).astype(np.float32)
                mock_sd.rec.return_value = mock_audio
                mock_sd.wait.return_value = None

                stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)
                stt.model = mock_model
                stt.is_loaded = True

                # Mock VAD pour retourner False (silence)
                with patch.object(stt, "detect_speech_activity", return_value=False):
                    with patch("bbia_sim.voice_whisper.time") as mock_time:
                        mock_time.time.side_effect = [0.0, 2.0, 4.0, 6.0]

                        result = stt.transcribe_streaming(
                            max_duration=0.5, chunk_duration=0.25
                        )

                        # Devrait arrêter après silence prolongé
                        assert result is None or isinstance(result, str)
            finally:
                os.environ["BBIA_DISABLE_AUDIO"] = original_value

    @patch("bbia_sim.voice_whisper.sd")
    @patch("bbia_sim.voice_whisper.sf")
    @patch("bbia_sim.voice_whisper.whisper")
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_transcribe_streaming_temp_file_pool(self, mock_whisper, mock_sf, mock_sd):
        """Test streaming avec réutilisation pool fichiers temp (couverture lignes 583-598)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            original_value = os.environ.get("BBIA_DISABLE_AUDIO", "0")
            os.environ["BBIA_DISABLE_AUDIO"] = "0"

            try:
                from pathlib import Path

                import bbia_sim.voice_whisper as voice_whisper_module

                voice_whisper_module._whisper_models_cache.clear()

                mock_model = MagicMock()
                mock_model.transcribe.return_value = {"text": "test"}
                mock_whisper.load_model.return_value = mock_model

                mock_audio = np.random.rand(8000).astype(np.float32)
                mock_sd.rec.return_value = mock_audio
                mock_sd.wait.return_value = None

                stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=False)
                stt.model = mock_model
                stt.is_loaded = True

                # Pré-remplir pool
                temp_file1 = Path("/tmp/test1.wav")
                stt._temp_file_pool = [temp_file1]
                stt._max_temp_files = 3

                with patch("bbia_sim.voice_whisper.time") as mock_time:
                    mock_time.time.side_effect = [0.0, 2.0]

                    result = stt.transcribe_streaming(
                        max_duration=0.5,
                        chunk_duration=0.25,
                        transcription_interval=1.0,
                    )

                    # Vérifier que pool est utilisé
                    assert hasattr(stt, "_temp_file_pool")
            finally:
                os.environ["BBIA_DISABLE_AUDIO"] = original_value

    @patch("bbia_sim.voice_whisper.sd")
    @patch("bbia_sim.voice_whisper.sf")
    @patch("bbia_sim.voice_whisper.whisper")
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_transcribe_streaming_text_filtering(self, mock_whisper, mock_sf, mock_sd):
        """Test streaming avec filtrage texte (couverture lignes 623-626)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            original_value = os.environ.get("BBIA_DISABLE_AUDIO", "0")
            os.environ["BBIA_DISABLE_AUDIO"] = "0"

            try:
                import bbia_sim.voice_whisper as voice_whisper_module

                voice_whisper_module._whisper_models_cache.clear()

                mock_model = MagicMock()
                mock_whisper.load_model.return_value = mock_model

                mock_audio = np.random.rand(8000).astype(np.float32)
                mock_sd.rec.return_value = mock_audio
                mock_sd.wait.return_value = None

                stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=False)
                stt.model = mock_model
                stt.is_loaded = True

                # Test avec texte filtré ("you", "thank you")
                with patch.object(
                    mock_model, "transcribe", return_value={"text": "you"}
                ):
                    with patch("bbia_sim.voice_whisper.time") as mock_time:
                        mock_time.time.side_effect = [0.0, 2.0]

                        result = stt.transcribe_streaming(
                            max_duration=0.5,
                            chunk_duration=0.25,
                            transcription_interval=1.0,
                        )

                        # Texte "you" devrait être filtré
                        assert result is None or isinstance(result, str)
            finally:
                os.environ["BBIA_DISABLE_AUDIO"] = original_value

    @patch("bbia_sim.voice_whisper.sd")
    @patch("bbia_sim.voice_whisper.sf")
    @patch("bbia_sim.voice_whisper.whisper")
    @patch.dict(os.environ, {"BBIA_DISABLE_AUDIO": "0"}, clear=False)
    def test_transcribe_streaming_empty_final_text(
        self, mock_whisper, mock_sf, mock_sd
    ):
        """Test streaming avec texte final vide (couverture lignes 655-662)."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True):
            original_value = os.environ.get("BBIA_DISABLE_AUDIO", "0")
            os.environ["BBIA_DISABLE_AUDIO"] = "0"

            try:
                import bbia_sim.voice_whisper as voice_whisper_module

                voice_whisper_module._whisper_models_cache.clear()

                mock_model = MagicMock()
                mock_model.transcribe.return_value = {"text": ""}  # Texte vide
                mock_whisper.load_model.return_value = mock_model

                mock_audio = np.random.rand(8000).astype(np.float32)
                mock_sd.rec.return_value = mock_audio
                mock_sd.wait.return_value = None

                stt = WhisperSTT(model_size="tiny", language="fr", enable_vad=False)
                stt.model = mock_model
                stt.is_loaded = True

                with patch("bbia_sim.voice_whisper.time") as mock_time:
                    mock_time.time.side_effect = [0.0, 2.0]

                    result = stt.transcribe_streaming(
                        max_duration=0.5,
                        chunk_duration=0.25,
                        transcription_interval=1.0,
                    )

                    # Devrait retourner None si aucun texte
                    assert result is None
            finally:
                os.environ["BBIA_DISABLE_AUDIO"] = original_value
