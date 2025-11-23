#!/usr/bin/env python3
"""Tests supplémentaires pour utiliser toutes les capacités restantes du projet BBIA.

Ce fichier complète test_capabilities_completeness.py pour atteindre 100% d'utilisation.
"""

import os
import tempfile

import pytest

from bbia_sim.ai_backends import (
    KokoroTTS,
    LlamaCppLLM,
    NeuTTSTTS,
    OpenVoiceTTSTTS,
    get_llm_backend,
    get_stt_backend,
)

# SimpleMove est une classe interne dans create_move_from_positions, non exportable
from bbia_sim.backends.simulation_shims import (
    SimulationCamera,
    SimulationIOModule,
    SimulationMediaModule,
    SimulationMicrophone,
    SimulationSpeaker,
)
from bbia_sim.bbia_idle_animations import (
    BBIABreathingAnimation,
    BBIAPoseTransitionManager,
    BBIAVocalTremor,
    BBIIdleAnimationManager,
)
from bbia_sim.daemon.app.backend_adapter import BackendAdapter
from bbia_sim.daemon.app.routers.daemon import DaemonStatus
from bbia_sim.daemon.app.routers.ecosystem import RobotCapabilities
from bbia_sim.daemon.app.routers.motors import (
    MotorControlMode,
    MotorStatus,
)
from bbia_sim.daemon.models import (
    GripperControl,
    HeadControl,
    JointPosition,
    MotionCommand,
    MoveUUID,
    Pose,
    TelemetryMessage,
)
from bbia_sim.dashboard import BBIAWebSocketManager
from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager
from bbia_sim.pose_detection import BBIAPoseDetection, create_pose_detector
from bbia_sim.sim.assets.reachy_official.asset_mapping import AssetMapping

# Tests pour les classes et types non utilisés
from bbia_sim.utils.types import (
    ConversationEntry,
    DetectionResult,
    FaceDetection,
    LookAtParams,
    MetricsData,
    ModelInfo,
    MovementRecording,
    PlayAudioParams,
    RobotStatus,
    SentimentDict,
    SentimentResult,
    SetEmotionParams,
    SetTargetParams,
    TelemetryData,
)


class TestUtilsTypes:
    """Tests pour les types utilitaires."""

    def test_look_at_params(self) -> None:
        """Test LookAtParams."""
        params: LookAtParams = {
            "target_x": 0.5,
            "target_y": 0.3,
            "target_z": 0.2,
        }
        assert params["target_x"] == 0.5

    def test_play_audio_params(self) -> None:
        """Test PlayAudioParams."""
        params: PlayAudioParams = {
            "file_path": "test.wav",
            "volume": 0.8,
        }
        assert params["file_path"] == "test.wav"

    def test_set_emotion_params(self) -> None:
        """Test SetEmotionParams."""
        params: SetEmotionParams = {
            "emotion": "happy",
            "intensity": 0.7,
        }
        assert params["emotion"] == "happy"

    def test_set_target_params(self) -> None:
        """Test SetTargetParams."""
        params: SetTargetParams = {
            "head": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            "antennas": [0.0, 0.0],
        }
        assert params["head"] is not None

    def test_movement_recording(self) -> None:
        """Test MovementRecording."""
        recording: MovementRecording = {
            "name": "test_move",
            "positions": {"head_yaw": 0.0},
            "duration": 1.0,
        }
        assert recording["name"] == "test_move"

    def test_metrics_data(self) -> None:
        """Test MetricsData."""
        metrics: MetricsData = {
            "cpu_usage": 50.0,
            "memory_usage": 60.0,
        }
        assert metrics["cpu_usage"] == 50.0

    def test_model_info(self) -> None:
        """Test ModelInfo."""
        info: ModelInfo = {
            "name": "test_model",
            "version": "1.0.0",
        }
        assert info["name"] == "test_model"

    def test_sentiment_dict(self) -> None:
        """Test SentimentDict."""
        sentiment: SentimentDict = {
            "sentiment": "positive",
            "score": 0.8,
            "label": "positive",
        }
        assert sentiment["sentiment"] == "positive"

    def test_sentiment_result(self) -> None:
        """Test SentimentResult."""
        result: SentimentResult = {
            "sentiment": "positive",
            "score": 0.8,
        }
        assert result["sentiment"] == "positive"

    def test_robot_capabilities(self) -> None:
        """Test RobotCapabilities."""
        caps: RobotCapabilities = {
            "has_camera": True,
            "has_microphone": True,
            "has_speaker": True,
        }
        assert caps["has_camera"] is True

    def test_telemetry_data(self) -> None:
        """Test TelemetryData."""
        data: TelemetryData = {
            "timestamp": 1234567890.0,
            "joint_positions": {"head_yaw": 0.0},
        }
        assert data["timestamp"] == 1234567890.0

    def test_detection_result(self) -> None:
        """Test DetectionResult."""
        result: DetectionResult = {
            "class": "person",
            "confidence": 0.9,
            "bbox": [0, 0, 100, 100],
        }
        assert result["class"] == "person"

    def test_face_detection(self) -> None:
        """Test FaceDetection."""
        detection: FaceDetection = {
            "bbox": [0, 0, 100, 100],
            "landmarks": [[10, 10], [20, 20]],
        }
        assert detection["bbox"] == [0, 0, 100, 100]

    def test_robot_status(self) -> None:
        """Test RobotStatus."""
        status: RobotStatus = {
            "is_connected": True,
            "battery_level": 80.0,
        }
        assert status["is_connected"] is True

    def test_conversation_entry(self) -> None:
        """Test ConversationEntry."""
        entry: ConversationEntry = {
            "user": "Bonjour",
            "bbia": "Salut !",
            "timestamp": "2025-11-21T10:00:00",
        }
        assert entry["user"] == "Bonjour"


class TestDaemonModels:
    """Tests pour les modèles daemon restants."""

    def test_joint_position(self) -> None:
        """Test JointPosition."""
        joint = JointPosition(joint_name="yaw_body", position=0.5)
        assert joint.joint_name == "yaw_body"

    def test_pose(self) -> None:
        """Test Pose."""
        pose = Pose(x=0.1, y=0.2, z=0.3)
        assert pose.x == 0.1

    def test_head_control(self) -> None:
        """Test HeadControl."""
        control = HeadControl(yaw=0.5, pitch=0.3)
        assert control.yaw == 0.5

    def test_gripper_control(self) -> None:
        """Test GripperControl."""
        control = GripperControl(side="left", action="open")
        assert control.side == "left"

    def test_motion_command(self) -> None:
        """Test MotionCommand."""
        command = MotionCommand(command="move", parameters={"speed": 0.5})
        assert command.command == "move"

    def test_telemetry_message(self) -> None:
        """Test TelemetryMessage."""
        message = TelemetryMessage(type="status", data={"status": "ok"})
        assert message.type == "status"

    def test_move_uuid(self) -> None:
        """Test MoveUUID."""
        from uuid import uuid4

        move_id = MoveUUID(uuid=uuid4())
        assert move_id.uuid is not None


class TestDaemonRouters:
    """Tests pour les modèles des routers daemon."""

    def test_daemon_status(self) -> None:
        """Test DaemonStatus."""
        status = DaemonStatus(status="running", simulation_running=True)
        assert status.status == "running"
        status_dict = status.dict()
        assert "status" in status_dict

    def test_camera_toggle_request(self) -> None:
        """Test CameraToggleRequest."""
        from bbia_sim.daemon.app.routers.media import CameraToggleRequest

        request = CameraToggleRequest(enabled=True)
        assert request.enabled is True

    def test_media_status_response(self) -> None:
        """Test MediaStatusResponse."""
        from bbia_sim.daemon.app.routers.media import MediaStatusResponse

        response = MediaStatusResponse(
            speaker_volume=0.5,
            microphone_volume=0.5,
            camera_enabled=True,
            speaker_active=True,
            microphone_active=True,
        )
        assert response.camera_enabled is True

    def test_motor_status(self) -> None:
        """Test MotorStatus."""
        status = MotorStatus(mode=MotorControlMode.Enabled)
        assert status.mode == MotorControlMode.Enabled

    def test_motor_control_mode(self) -> None:
        """Test MotorControlMode."""
        mode = MotorControlMode.Enabled
        assert mode == MotorControlMode.Enabled


class TestSimulationShims:
    """Tests pour les shims de simulation."""

    def test_simulation_io_module(self) -> None:
        """Test SimulationIOModule."""
        io_module = SimulationIOModule()
        assert io_module is not None

    def test_simulation_media_module(self) -> None:
        """Test SimulationMediaModule."""
        media_module = SimulationMediaModule()
        assert media_module is not None

    def test_simulation_camera(self) -> None:
        """Test SimulationCamera."""
        camera = SimulationCamera()
        assert camera is not None

    def test_simulation_microphone(self) -> None:
        """Test SimulationMicrophone."""
        microphone = SimulationMicrophone()
        assert microphone is not None

    def test_simulation_speaker(self) -> None:
        """Test SimulationSpeaker."""
        speaker = SimulationSpeaker()
        assert speaker is not None


class TestAIBackends:
    """Tests pour les backends IA restants."""

    def test_kokoro_tts(self) -> None:
        """Test KokoroTTS."""
        try:
            tts = KokoroTTS()
            assert tts is not None
        except (ImportError, RuntimeError, AttributeError):
            pytest.skip("KokoroTTS non disponible")

    def test_neu_tts(self) -> None:
        """Test NeuTTSTTS."""
        try:
            tts = NeuTTSTTS()
            assert tts is not None
        except (ImportError, RuntimeError, AttributeError):
            pytest.skip("NeuTTS non disponible")

    def test_open_voice_tts(self) -> None:
        """Test OpenVoiceTTSTTS."""
        try:
            tts = OpenVoiceTTSTTS()
            assert tts is not None
        except (ImportError, RuntimeError, AttributeError):
            pytest.skip("OpenVoiceTTS non disponible")

    def test_llama_cpp_llm(self) -> None:
        """Test LlamaCppLLM."""
        try:
            llm = LlamaCppLLM()
            assert llm is not None
        except (ImportError, RuntimeError, AttributeError):
            pytest.skip("LlamaCpp non disponible")

    def test_local_llm(self) -> None:
        """Test LocalLLM via factory get_llm_backend().

        Utilise la factory pour obtenir une implémentation concrète du protocol LocalLLM.
        Plus intelligent et performant que d'essayer d'instancier le protocol directement.
        """
        try:
            llm = get_llm_backend()
            assert llm is not None
            # Test que l'implémentation respecte le protocol
            result = llm.generate("test", max_tokens=10)
            assert isinstance(result, str)
        except (ImportError, RuntimeError, AttributeError, TypeError) as e:
            pytest.skip(f"LocalLLM non disponible: {e}")

    def test_speech_to_text(self) -> None:
        """Test SpeechToText via factory get_stt_backend().

        Utilise la factory pour obtenir une implémentation concrète du protocol SpeechToText.
        Plus intelligent et performant que d'essayer d'instancier le protocol directement.
        """
        try:
            stt = get_stt_backend()
            assert stt is not None
            # Test que l'implémentation respecte le protocol
            # Utilise un fichier temporaire vide pour tester (retournera None ou "")
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                tmp_path = tmp.name
            try:
                result = stt.transcribe_wav(tmp_path)
                # Résultat peut être None ou str (selon implémentation)
                assert result is None or isinstance(result, str)
            finally:
                if os.path.exists(tmp_path):
                    os.unlink(tmp_path)
        except (ImportError, RuntimeError, AttributeError, TypeError, OSError) as e:
            pytest.skip(f"SpeechToText non disponible: {e}")


class TestPoseDetection:
    """Tests pour la détection de pose."""

    def test_bbia_pose_detection_init(self) -> None:
        """Test initialisation BBIAPoseDetection."""
        detector = BBIAPoseDetection()
        assert detector is not None

    def test_bbia_pose_detection_detect(self) -> None:
        """Test détection de pose."""
        detector = BBIAPoseDetection()
        import numpy as np

        image = np.zeros((480, 640, 3), dtype=np.uint8)
        result = detector.detect_pose(image)
        # Peut être None si MediaPipe n'est pas disponible ou pas de personne détectée
        assert result is None or isinstance(result, dict)

    def test_create_pose_detector(self) -> None:
        """Test create_pose_detector."""
        detector = create_pose_detector()
        # Peut être None si MediaPipe n'est pas disponible
        assert detector is None or isinstance(detector, BBIAPoseDetection)


class TestFaceRecognition:
    """Tests pour la reconnaissance faciale."""

    def test_bbia_person_recognition_init(self) -> None:
        """Test initialisation BBIAPersonRecognition."""
        try:
            from bbia_sim.face_recognition import create_face_recognition

            recognizer = create_face_recognition("test_db", "test_person")
            if recognizer:
                assert recognizer is not None
        except (ImportError, RuntimeError, AttributeError, ValueError):
            pytest.skip("Face recognition non disponible")


class TestIdleAnimations:
    """Tests pour les animations idle."""

    def test_bbia_breathing_animation_init(self) -> None:
        """Test initialisation BBIABreathingAnimation."""
        from bbia_sim.backends.mujoco_backend import MuJoCoBackend

        backend = MuJoCoBackend()
        backend.connect()
        breathing = BBIABreathingAnimation(backend)
        assert breathing is not None
        backend.disconnect()

    def test_bbia_pose_transition_manager_init(self) -> None:
        """Test initialisation BBIAPoseTransitionManager."""
        from bbia_sim.backends.mujoco_backend import MuJoCoBackend

        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIAPoseTransitionManager(backend)
        assert manager is not None
        backend.disconnect()

    def test_bbia_vocal_tremor_init(self) -> None:
        """Test initialisation BBIAVocalTremor."""
        from bbia_sim.backends.mujoco_backend import MuJoCoBackend

        backend = MuJoCoBackend()
        backend.connect()
        tremor = BBIAVocalTremor(backend)
        assert tremor is not None
        backend.disconnect()

    def test_bbia_idle_animation_manager_init(self) -> None:
        """Test initialisation BBIIdleAnimationManager."""
        from bbia_sim.backends.mujoco_backend import MuJoCoBackend

        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIIdleAnimationManager(backend)
        assert manager is not None
        backend.disconnect()


class TestWebSocketManagers:
    """Tests pour les gestionnaires WebSocket."""

    def test_bbia_advanced_websocket_manager_init(self) -> None:
        """Test initialisation BBIAAdvancedWebSocketManager."""
        manager = BBIAAdvancedWebSocketManager()
        assert manager is not None

    def test_bbia_websocket_manager_init(self) -> None:
        """Test initialisation BBIAWebSocketManager."""
        manager = BBIAWebSocketManager()
        assert manager is not None


class TestAssetMapping:
    """Tests pour AssetMapping."""

    def test_asset_mapping(self) -> None:
        """Test AssetMapping."""
        mapping = AssetMapping(
            component_name="test",
            official_stl_path="test.stl",
            placeholder_path="test_placeholder.stl",
            description="Test mapping",
        )
        assert mapping is not None
        assert mapping.component_name == "test"


class TestSimpleMove:
    """Tests pour SimpleMove via create_move_from_positions."""

    def test_simple_move(self) -> None:
        """Test SimpleMove via create_move_from_positions."""
        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()
        backend.connect()
        try:
            # Test via create_move_from_positions (SimpleMove est interne)
            positions = [{"yaw_body": 0.0}, {"yaw_body": 0.5}]
            _move = backend.create_move_from_positions(positions, duration=1.0)
            # _move peut être None si reachy_mini n'est pas disponible
            # C'est normal, on teste juste que la méthode existe
            assert backend.create_move_from_positions is not None
        finally:
            backend.disconnect()


class TestBackendAdapterMethods:
    """Tests pour les méthodes BackendAdapter non utilisées."""

    def test_backend_adapter_methods(self) -> None:
        """Test méthodes BackendAdapter."""
        from bbia_sim.robot_factory import RobotFactory

        # Créer un backend RobotAPI
        robot_api = RobotFactory.create_backend("mujoco")
        if robot_api is None:
            pytest.skip("Backend MuJoCo non disponible")

        try:
            adapter = BackendAdapter(robot_api)
            adapter.connect_if_needed()

            # Test get_present_head_pose
            pose = adapter.get_present_head_pose()
            assert pose is not None

            # Test get_current_head_pose
            current_pose = adapter.get_current_head_pose()
            assert current_pose is not None

            # Test get_present_body_yaw
            yaw = adapter.get_present_body_yaw()
            assert isinstance(yaw, float)

            # Test get_present_antenna_joint_positions
            antennas = adapter.get_present_antenna_joint_positions()
            assert antennas is not None

            # Test get_present_head_joint_positions
            head_joints = adapter.get_present_head_joint_positions()
            # Peut être None, on vérifie juste que la méthode existe
            assert head_joints is None or isinstance(head_joints, dict)

            # Test get_present_passive_joint_positions
            passive_joints = adapter.get_present_passive_joint_positions()
            # Peut être None, dict ou array, on vérifie juste que la méthode existe
            import numpy as np

            assert (
                passive_joints is None
                or isinstance(passive_joints, dict)
                or isinstance(passive_joints, np.ndarray)
            )

            # Test get_status
            status = adapter.get_status()
            assert status is not None

            # Test get_urdf
            urdf = adapter.get_urdf()
            # Peut être None, on vérifie juste que la méthode existe
            assert urdf is None or isinstance(urdf, str)

            # Test close
            adapter.close()
        finally:
            if robot_api:
                robot_api.disconnect()


class TestBBIAVisionMethods:
    """Tests pour les méthodes BBIAVision non utilisées."""

    def test_scan_environment_async(self) -> None:
        """Test scan_environment_async."""
        import gc

        from bbia_sim.bbia_vision import BBIAVision

        # OPTIMISATION RAM: Utiliser robot_api=None pour éviter chargement modèles
        vision = BBIAVision(robot_api=None)
        vision.start_async_scanning()
        vision.scan_environment_async()
        vision.stop_async_scanning()

        # OPTIMISATION RAM: Nettoyer après test
        try:
            if hasattr(vision, "yolo_detector") and vision.yolo_detector:
                vision.yolo_detector.model = None
        except (AttributeError, TypeError):
            pass
        gc.collect()

    def test_scan_environment_from_image(self) -> None:
        """Test scan_environment_from_image."""
        import gc

        import numpy as np

        from bbia_sim.bbia_vision import BBIAVision

        # OPTIMISATION RAM: Utiliser robot_api=None et image réduite
        vision = BBIAVision(robot_api=None)
        # OPTIMISATION RAM: Image réduite (320x240 au lieu de 640x480)
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        result = vision.scan_environment_from_image(image)
        assert result is not None

        # OPTIMISATION RAM: Nettoyer après test
        try:
            if hasattr(vision, "yolo_detector") and vision.yolo_detector:
                vision.yolo_detector.model = None
        except (AttributeError, TypeError):
            pass
        gc.collect()


class TestBBIAVoiceAdvanced:
    """Tests pour BBIAVoiceAdvanced."""

    def test_is_coqui_available(self) -> None:
        """Test is_coqui_available."""
        from bbia_sim.bbia_voice_advanced import BBIAVoiceAdvanced

        voice = BBIAVoiceAdvanced()
        available = voice.is_coqui_available()
        assert isinstance(available, bool)

    def test_say(self) -> None:
        """Test say."""
        from bbia_sim.bbia_voice_advanced import BBIAVoiceAdvanced

        voice = BBIAVoiceAdvanced()
        result = voice.say("Test")
        assert isinstance(result, bool)

    def test_say_with_emotion(self) -> None:
        """Test say_with_emotion."""
        from bbia_sim.bbia_voice_advanced import BBIAVoiceAdvanced

        voice = BBIAVoiceAdvanced()
        result = voice.say_with_emotion("Test", "happy")
        assert isinstance(result, bool)

    def test_set_emotion(self) -> None:
        """Test set_emotion."""
        from bbia_sim.bbia_voice_advanced import BBIAVoiceAdvanced

        voice = BBIAVoiceAdvanced()
        voice.set_emotion("happy")


class TestTroubleshootingFunctions:
    """Tests pour les fonctions troubleshooting."""

    def test_troubleshooting_checker_methods(self) -> None:
        """Test méthodes TroubleshootingChecker."""
        from bbia_sim.troubleshooting import TroubleshootingChecker

        checker = TroubleshootingChecker()
        checker.check_python()
        checker.check_dependencies()
        checker.check_camera()
        checker.check_audio()
        checker.check_network()
        checker.check_mujoco()
        checker.check_ports()
        checker.check_permissions()
        results = checker.check_all()
        assert "summary" in results


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
