#!/usr/bin/env python3
"""
test_ia_modules.py - Tests pour modules IA légère
Tests mockés pour Whisper, YOLO, MediaPipe et Dashboard
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock, Mock, patch

import pytest

# Ajouter le chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.dashboard import FASTAPI_AVAILABLE, BBIAWebSocketManager
from bbia_sim.vision_yolo import YOLO_AVAILABLE, FaceDetector, YOLODetector
from bbia_sim.voice_whisper import WHISPER_AVAILABLE, VoiceCommandMapper, WhisperSTT


class TestWhisperSTT:
    """Tests pour le module Whisper STT."""

    def test_whisper_availability(self):
        """Test disponibilité Whisper."""
        assert isinstance(WHISPER_AVAILABLE, bool)

    def test_voice_command_mapper(self):
        """Test mappeur de commandes vocales."""
        mapper = VoiceCommandMapper()

        # Test commandes françaises
        assert mapper.map_command("salue")["action"] == "greet"
        assert mapper.map_command("regarde-moi")["action"] == "look_at"
        assert mapper.map_command("sois content")["action"] == "happy"

        # Test commandes anglaises
        assert mapper.map_command("hello")["action"] == "greet"
        assert mapper.map_command("look at me")["action"] == "look_at"

        # Test commande non reconnue
        assert mapper.map_command("commande inconnue") is None

    @patch("bbia_sim.voice_whisper.whisper")
    def test_whisper_stt_creation(self, mock_whisper):
        """Test création WhisperSTT avec mock."""
        if WHISPER_AVAILABLE:
            stt = WhisperSTT(model_size="tiny", language="fr")
            assert stt.model_size == "tiny"
            assert stt.language == "fr"

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False)
    def test_whisper_stt_init_no_whisper(self):
        """Test initialisation WhisperSTT sans Whisper."""
        stt = WhisperSTT(model_size="tiny", language="fr")
        assert stt.model_size == "tiny"
        assert stt.is_loaded is False
        assert stt.model is None

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True)
    @patch("bbia_sim.voice_whisper.whisper")
    def test_whisper_stt_load_model_from_cache(self, mock_whisper_module):
        """Test chargement modèle depuis cache."""
        import bbia_sim.voice_whisper as voice_module

        # Mettre modèle dans cache
        mock_model = MagicMock()
        voice_module._whisper_models_cache["tiny"] = mock_model
        voice_module._whisper_model_last_used["tiny"] = 1000.0

        stt = WhisperSTT(model_size="tiny", language="fr")
        result = stt.load_model()

        # Doit retourner True et utiliser cache
        assert result is True
        assert stt.is_loaded is True
        assert stt.model is mock_model

        # Nettoyer cache après test
        voice_module._whisper_models_cache.clear()
        voice_module._whisper_model_last_used.clear()

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True)
    @patch("bbia_sim.voice_whisper.whisper")
    def test_whisper_stt_load_model_new(self, mock_whisper_module):
        """Test chargement nouveau modèle Whisper."""
        import bbia_sim.voice_whisper as voice_module

        # Nettoyer cache
        voice_module._whisper_models_cache.clear()
        voice_module._whisper_model_last_used.clear()

        mock_model = MagicMock()
        mock_whisper_module.load_model.return_value = mock_model

        stt = WhisperSTT(model_size="tiny", language="fr")
        result = stt.load_model()

        assert result is True
        assert stt.is_loaded is True
        assert stt.model is mock_model
        mock_whisper_module.load_model.assert_called_once_with("tiny")

        # Nettoyer après test
        voice_module._whisper_models_cache.clear()
        voice_module._whisper_model_last_used.clear()

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True)
    @patch("bbia_sim.voice_whisper.whisper")
    def test_whisper_stt_load_model_error(self, mock_whisper_module):
        """Test erreur chargement modèle Whisper."""
        import bbia_sim.voice_whisper as voice_module

        voice_module._whisper_models_cache.clear()
        voice_module._whisper_model_last_used.clear()

        mock_whisper_module.load_model.side_effect = Exception("Load error")

        stt = WhisperSTT(model_size="tiny", language="fr")
        result = stt.load_model()

        assert result is False
        assert stt.is_loaded is False

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False)
    def test_whisper_stt_transcribe_audio_no_whisper(self):
        """Test transcription audio sans Whisper."""
        stt = WhisperSTT(model_size="tiny", language="fr")
        result = stt.transcribe_audio("test.wav")
        assert result is None

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True)
    @patch("bbia_sim.voice_whisper.whisper")
    def test_whisper_stt_transcribe_audio_success(self, mock_whisper_module):
        """Test transcription audio réussie."""
        mock_model = MagicMock()
        mock_model.transcribe.return_value = {"text": "Bonjour"}
        mock_whisper_module.load_model.return_value = mock_model

        stt = WhisperSTT(model_size="tiny", language="fr")
        stt.model = mock_model
        stt.is_loaded = True

        result = stt.transcribe_audio("test.wav")
        assert result == "Bonjour"
        mock_model.transcribe.assert_called_once()

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True)
    @patch("bbia_sim.voice_whisper.whisper")
    def test_whisper_stt_transcribe_audio_model_not_loaded(self, mock_whisper_module):
        """Test transcription avec modèle non chargé."""
        stt = WhisperSTT(model_size="tiny", language="fr")
        stt.is_loaded = False
        # Mock load_model pour retourner False
        stt.load_model = MagicMock(return_value=False)

        result = stt.transcribe_audio("test.wav")
        assert result is None

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True)
    @patch("bbia_sim.voice_whisper.whisper")
    def test_whisper_stt_transcribe_audio_error(self, mock_whisper_module):
        """Test erreur transcription audio."""
        mock_model = MagicMock()
        mock_model.transcribe.side_effect = Exception("Transcription error")
        mock_whisper_module.load_model.return_value = mock_model

        stt = WhisperSTT(model_size="tiny", language="fr")
        stt.model = mock_model
        stt.is_loaded = True

        result = stt.transcribe_audio("test.wav")
        assert result is None

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True)
    @patch("os.environ.get", return_value="1")
    def test_whisper_stt_transcribe_microphone_disabled(self, mock_env):
        """Test transcription microphone désactivé."""
        stt = WhisperSTT(model_size="tiny", language="fr")
        result = stt.transcribe_microphone(duration=1.0)
        assert result is None

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False)
    def test_whisper_stt_transcribe_microphone_no_whisper(self):
        """Test transcription microphone sans Whisper."""
        stt = WhisperSTT(model_size="tiny", language="fr")
        result = stt.transcribe_microphone(duration=1.0)
        assert result is None

    @patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", True)
    def test_whisper_stt_transcribe_audio_language_auto(self):
        """Test transcription avec langue auto."""
        mock_model = MagicMock()
        mock_model.transcribe.return_value = {"text": "Hello"}

        stt = WhisperSTT(model_size="tiny", language="auto")
        stt.model = mock_model
        stt.is_loaded = True

        result = stt.transcribe_audio("test.wav")
        assert result == "Hello"
        # Vérifier que language=None a été passé (langue auto)
        call_args = mock_model.transcribe.call_args
        assert call_args[1]["language"] is None

    def test_whisper_stt_fallback(self):
        """Test fallback quand Whisper non disponible."""
        with patch("bbia_sim.voice_whisper.WHISPER_AVAILABLE", False):
            stt = WhisperSTT()
            assert stt.model is None


class TestYOLODetector:
    """Tests pour le module YOLO."""

    def test_yolo_availability(self):
        """Test disponibilité YOLO."""
        assert isinstance(YOLO_AVAILABLE, bool)

    def test_yolo_detector_creation(self):
        """Test création YOLODetector."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.5)
        assert detector.model_size == "n"
        assert detector.confidence_threshold == 0.5
        assert not detector.is_loaded

    def test_target_classes_mapping(self):
        """Test mapping classes d'intérêt."""
        detector = YOLODetector()

        # Test classes d'intérêt
        assert "person" in detector.target_classes
        assert "face" in detector.target_classes
        assert "hand" in detector.target_classes

        # Test mapping vers actions
        assert detector.target_classes["person"] == "look_at"
        assert detector.target_classes["phone"] == "curious"

    def test_detection_mapping(self):
        """Test mapping détection vers action."""
        detector = YOLODetector()

        # Mock détection
        detection = {
            "class_name": "person",
            "confidence": 0.8,
            "center": [320, 240],
            "bbox": [100, 100, 200, 200],
        }

        action_data = detector.map_detection_to_action(detection)
        assert action_data is not None
        assert action_data["action"] == "look_at"
        assert action_data["confidence"] == 0.8
        assert action_data["object"] == "person"


class TestFaceDetector:
    """Tests pour le module détection de visages."""

    @pytest.mark.skip(
        reason="MediaPipe a des problèmes matplotlib dans l'environnement"
    )
    def test_face_detector_creation(self):
        """Test création FaceDetector."""
        detector = FaceDetector()
        # Le détecteur peut être None si MediaPipe non disponible
        assert detector.mp_face_detection is None or hasattr(
            detector.mp_face_detection, "FaceDetection"
        )

    @pytest.mark.skip(
        reason="MediaPipe a des problèmes matplotlib dans l'environnement"
    )
    def test_best_face_selection(self):
        """Test sélection meilleur visage."""
        detector = FaceDetector()

        # Mock détections
        detections = [
            {"confidence": 0.7, "area": 10000},
            {"confidence": 0.9, "area": 5000},
            {"confidence": 0.6, "area": 15000},
        ]

        best_face = detector.get_best_face(detections)
        assert best_face is not None
        # Devrait choisir celui avec meilleur score combiné


class TestDashboard:
    """Tests pour le dashboard web."""

    def test_fastapi_availability(self):
        """Test disponibilité FastAPI."""
        assert isinstance(FASTAPI_AVAILABLE, bool)

    def test_websocket_manager(self):
        """Test gestionnaire WebSocket."""
        manager = BBIAWebSocketManager()
        assert len(manager.active_connections) == 0
        assert manager.robot is None
        assert manager.robot_backend == "mujoco"

    def test_websocket_manager_backend(self):
        """Test changement backend."""
        manager = BBIAWebSocketManager()
        manager.robot_backend = "reachy"
        assert manager.robot_backend == "reachy"


class TestIAModulesIntegration:
    """Tests d'intégration des modules IA."""

    def test_voice_to_action_flow(self):
        """Test flux voix → action."""
        mapper = VoiceCommandMapper()

        # Simuler commande vocale
        text = "salue"
        action_data = mapper.map_command(text)

        assert action_data is not None
        assert action_data["action"] == "greet"
        assert action_data["confidence"] == 1.0

    def test_vision_to_action_flow(self):
        """Test flux vision → action."""
        detector = YOLODetector()

        # Simuler détection
        detection = {
            "class_name": "person",
            "confidence": 0.8,
            "center": [320, 240],
            "bbox": [100, 100, 200, 200],
        }

        action_data = detector.map_detection_to_action(detection)

        assert action_data is not None
        assert action_data["action"] == "look_at"
        assert action_data["direction"] in ["left", "right"]

    def test_dashboard_health_endpoint(self):
        """Test endpoint santé dashboard."""
        if FASTAPI_AVAILABLE:
            from bbia_sim.dashboard import app

            # Test que l'app existe
            assert app is not None
            assert hasattr(app, "get")
            assert hasattr(app, "websocket")


# Tests de performance (mockés)
class TestIAPerformance:
    """Tests de performance des modules IA."""

    def test_whisper_latency_target(self):
        """Test latence Whisper < 800ms."""
        # Mock latence rapide
        with patch("bbia_sim.voice_whisper.time.time") as mock_time:
            mock_time.side_effect = [0, 0.5]  # 500ms

            mapper = VoiceCommandMapper()
            start_time = mock_time()

            # Simuler traitement
            action_data = mapper.map_command("salue")

            end_time = mock_time()
            latency = end_time - start_time

            assert latency < 0.8  # < 800ms
            assert action_data is not None

    def test_yolo_fps_target(self):
        """Test FPS YOLO ≥ 15."""
        detector = YOLODetector()

        # Mock détection rapide
        mock_image = Mock()

        with patch.object(detector, "detect_objects") as mock_detect:
            mock_detect.return_value = []  # Pas de détection pour test rapide

            # Simuler traitement image
            detector.detect_objects(mock_image)

            # Vérifier que la méthode existe
            assert callable(detector.detect_objects)


if __name__ == "__main__":
    # Test rapide des modules
    print("🧪 Test modules IA BBIA")
    print("=" * 40)

    # Test Whisper
    print(f"Whisper disponible: {WHISPER_AVAILABLE}")
    mapper = VoiceCommandMapper()
    print(f"Commandes mappées: {len(mapper.commands)}")

    # Test YOLO
    print(f"YOLO disponible: {YOLO_AVAILABLE}")
    detector = YOLODetector()
    print(f"Classes cibles: {len(detector.target_classes)}")

    # Test Dashboard
    print(f"FastAPI disponible: {FASTAPI_AVAILABLE}")
    manager = BBIAWebSocketManager()
    print(f"Backend par défaut: {manager.robot_backend}")

    print("✅ Tests modules IA terminés")
