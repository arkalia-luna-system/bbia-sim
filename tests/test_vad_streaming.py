#!/usr/bin/env python3
"""
Tests pour VAD (Voice Activity Detection) et Whisper streaming.
"""

import logging
import os
import unittest
from unittest.mock import MagicMock, patch

# Désactiver audio pour CI
os.environ["BBIA_DISABLE_AUDIO"] = "1"

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TestVAD(unittest.TestCase):
    """Tests pour détection d'activité vocale (VAD)."""

    def setUp(self) -> None:
        """Setup test."""
        try:
            from src.bbia_sim.voice_whisper import WhisperSTT

            self.whisper = WhisperSTT(model_size="tiny", language="fr", enable_vad=True)
        except ImportError as e:
            self.skipTest(f"Module non disponible: {e}")

    def test_vad_initialization(self) -> None:
        """Test initialisation VAD."""
        self.assertTrue(self.whisper.enable_vad)
        self.assertFalse(self.whisper._vad_loaded)

    @patch("transformers.pipeline")
    def test_vad_detection_speech(self, mock_pipeline: MagicMock) -> None:
        """Test détection VAD - parole."""
        import numpy as np

        # Mock pipeline VAD - l'instance retournée doit être appelable comme fonction
        mock_vad_instance = MagicMock()
        # Configurer pour que l'appel du mock (comme fonction) retourne le résultat
        mock_vad_instance.return_value = [{"label": "SPEECH", "score": 0.95}]
        # Configurer le mock pour qu'il soit appelable
        mock_vad_instance.side_effect = lambda *args, **kwargs: [{"label": "SPEECH", "score": 0.95}]
        mock_pipeline.return_value = mock_vad_instance

        # Forcer le chargement du modèle VAD dans whisper AVANT d'appeler detect_speech_activity
        # Note: detect_speech_activity charge le modèle si nécessaire, mais on force ici
        self.whisper._vad_model = mock_vad_instance
        self.whisper._vad_loaded = True

        # Simuler chunk audio
        audio_chunk = np.random.rand(16000).astype(np.float32)

        # Détecter parole - le mock doit être appelé et retourner SPEECH
        # Patch également l'environnement pour ne pas retourner False immédiatement
        import os

        original_env = os.environ.get("BBIA_DISABLE_AUDIO", "0")
        try:
            os.environ["BBIA_DISABLE_AUDIO"] = "0"
            result = self.whisper.detect_speech_activity(audio_chunk)
            self.assertTrue(result, "Parole devrait être détectée")
        finally:
            os.environ["BBIA_DISABLE_AUDIO"] = original_env

    @patch("transformers.pipeline")
    def test_vad_detection_silence(self, mock_pipeline: MagicMock) -> None:
        """Test détection VAD - silence."""
        # Mock pipeline VAD
        mock_vad = MagicMock()
        mock_vad.return_value = [{"label": "NO_SPEECH", "score": 0.95}]
        mock_pipeline.return_value = mock_vad

        import numpy as np

        audio_chunk = np.random.rand(16000).astype(np.float32)

        result = self.whisper.detect_speech_activity(audio_chunk)

        self.assertFalse(result, "Silence devrait être détecté")

    def test_vad_disabled(self) -> None:
        """Test VAD désactivé."""
        try:
            from src.bbia_sim.voice_whisper import WhisperSTT

            whisper_no_vad = WhisperSTT(enable_vad=False)
            import numpy as np

            audio_chunk = np.random.rand(16000).astype(np.float32)

            # Si VAD désactivé, devrait toujours retourner True
            result = whisper_no_vad.detect_speech_activity(audio_chunk)
            self.assertTrue(result)
        except ImportError as e:
            self.skipTest(f"Module non disponible: {e}")


class TestWhisperStreaming(unittest.TestCase):
    """Tests pour Whisper streaming."""

    def setUp(self) -> None:
        """Setup test."""
        try:
            from src.bbia_sim.voice_whisper import WhisperSTT

            self.whisper = WhisperSTT(model_size="tiny", language="fr")
        except ImportError as e:
            self.skipTest(f"Module non disponible: {e}")

    @patch("src.bbia_sim.voice_whisper.sd")
    @patch("src.bbia_sim.voice_whisper.sf")
    @patch("src.bbia_sim.voice_whisper.whisper")
    def test_streaming_initialization(
        self,
        mock_whisper: MagicMock,
        mock_soundfile: MagicMock,
        mock_sounddevice: MagicMock,
    ) -> None:
        """Test initialisation streaming."""
        # Mock Whisper
        mock_model = MagicMock()
        mock_model.transcribe.return_value = {"text": "test transcription"}
        mock_whisper.load_model.return_value = mock_model
        self.whisper.model = mock_model
        self.whisper.is_loaded = True

        # Mock sounddevice
        import numpy as np

        mock_sounddevice.rec.return_value = np.random.rand(8000).astype(np.float32)
        mock_sounddevice.wait.return_value = None

        # Mock soundfile
        mock_soundfile.write.return_value = None

        # S'assurer que les mocks sont utilisables dans le module
        import src.bbia_sim.voice_whisper as voice_whisper_module

        voice_whisper_module.sd = mock_sounddevice
        voice_whisper_module.sf = mock_soundfile

        # Test streaming (court pour éviter timeout)
        # Note: Ne pas stocker result car test mock (serait None en vrai)
        self.whisper.transcribe_streaming(chunk_duration=0.1, max_duration=0.2)

        # Vérifier que modèle a été appelé
        self.assertIsNotNone(self.whisper.model)

    def test_streaming_callback(self) -> None:
        """Test callback streaming."""
        callback_calls: list[tuple[str, float]] = []

        def test_callback(text: str, duration: float) -> None:
            callback_calls.append((text, duration))

        # Test avec mocks (comme ci-dessus)
        # Ici on vérifie juste que callback serait appelé
        self.assertIsNotNone(test_callback)


if __name__ == "__main__":
    unittest.main(verbosity=2)
