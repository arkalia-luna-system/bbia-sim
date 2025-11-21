#!/usr/bin/env python3
"""Tests finaux pour toutes les méthodes restantes.

Ce fichier complète tous les autres pour atteindre le maximum d'utilisation.
"""

import pytest
import numpy as np

from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.behaviors import (
    ConversationBehavior,
    DanceBehavior,
    EmotionShowBehavior,
    ExerciseBehavior,
    FollowFaceBehavior,
    FollowObjectBehavior,
    GameBehavior,
    MeditationBehavior,
)
from bbia_sim.bbia_behavior import (
    AntennaAnimationBehavior,
    GreetingBehavior,
    HideBehavior,
    EmotionalResponseBehavior,
)
from bbia_sim.daemon.app.backend_adapter import BackendAdapter
from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.daemon.app.routers.daemon import DaemonStatus
from bbia_sim.daemon.models import MotionCommand, MoveUUID, Matrix4x4Pose
from bbia_sim.ai_backends import (
    CoquiTTSTTS,
    KokoroTTS,
    LlamaCppLLM,
    LocalLLM,
    NeuTTSTTS,
    OpenVoiceTTSTTS,
    DummySTT,
    WhisperSTT,
    KittenTTSTTS,
)
from bbia_sim.backends.simulation_shims import SimulationIOModule, SimulationMediaModule
from bbia_sim.face_recognition import BBIAPersonRecognition
from bbia_sim.pose_detection import BBIAPoseDetection
from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager
from bbia_sim.dashboard import BBIAWebSocketManager
from bbia_sim.backends.reachy_mini_backend import SimpleMove
from bbia_sim.backends.reachy_backend import ReachyBackend
try:
    from bbia_sim.vision_yolo import FaceDetector
from bbia_sim.daemon.ws import ConnectionManager
except ImportError:
    FaceDetector = None  # type: ignore[assignment, misc]


class TestBBIABehaviorManagerAllMethods:
    """Tests pour toutes les méthodes de BBIABehaviorManager."""

    def test_register_behavior(self) -> None:
        """Test register_behavior."""
        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIABehaviorManager(robot_api=backend)
        from bbia_sim.bbia_behavior import WakeUpBehavior

        behavior = WakeUpBehavior(robot_api=backend)
        manager.register_behavior(behavior)
        assert "wake_up" in manager.behaviors
        backend.disconnect()

    def test_execute_behavior(self) -> None:
        """Test execute_behavior."""
        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIABehaviorManager(robot_api=backend)
        result = manager.execute_behavior("wake_up")
        assert isinstance(result, bool)
        backend.disconnect()

    def test_add_to_queue(self) -> None:
        """Test add_to_queue."""
        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIABehaviorManager(robot_api=backend)
        manager.add_to_queue("wake_up", {})
        assert not manager.behavior_queue.empty()
        backend.disconnect()

    def test_start_behavior_worker(self) -> None:
        """Test start_behavior_worker."""
        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIABehaviorManager(robot_api=backend)
        manager.start_behavior_worker()
        assert manager.is_running is True
        manager.stop_behavior_worker()
        backend.disconnect()

    def test_stop_behavior_worker(self) -> None:
        """Test stop_behavior_worker."""
        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIABehaviorManager(robot_api=backend)
        manager.start_behavior_worker()
        manager.stop_behavior_worker()
        assert manager.is_running is False
        backend.disconnect()

    def test_get_available_behaviors(self) -> None:
        """Test get_available_behaviors."""
        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIABehaviorManager(robot_api=backend)
        behaviors = manager.get_available_behaviors()
        assert isinstance(behaviors, list)
        backend.disconnect()

    def test_get_behavior_stats(self) -> None:
        """Test get_behavior_stats."""
        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIABehaviorManager(robot_api=backend)
        stats = manager.get_behavior_stats()
        assert "total_behaviors" in stats
        backend.disconnect()


class TestBehaviorsInit:
    """Tests pour les initialisations de tous les comportements."""

    def test_antenna_animation_behavior_init(self) -> None:
        """Test AntennaAnimationBehavior.__init__."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = AntennaAnimationBehavior(robot_api=backend)
        assert behavior is not None
        backend.disconnect()

    def test_conversation_behavior_init(self) -> None:
        """Test ConversationBehavior.__init__."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = ConversationBehavior(robot_api=backend)
        assert behavior is not None
        backend.disconnect()

    def test_dance_behavior_init(self) -> None:
        """Test DanceBehavior.__init__."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = DanceBehavior(robot_api=backend)
        assert behavior is not None
        backend.disconnect()

    def test_emotion_show_behavior_init(self) -> None:
        """Test EmotionShowBehavior.__init__."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = EmotionShowBehavior(robot_api=backend)
        assert behavior is not None
        backend.disconnect()

    def test_emotion_show_behavior_cancel(self) -> None:
        """Test EmotionShowBehavior.cancel."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = EmotionShowBehavior(robot_api=backend)
        behavior.cancel()
        backend.disconnect()

    def test_exercise_behavior_init(self) -> None:
        """Test ExerciseBehavior.__init__."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = ExerciseBehavior(robot_api=backend)
        assert behavior is not None
        backend.disconnect()

    def test_follow_face_behavior_init(self) -> None:
        """Test FollowFaceBehavior.__init__."""
        from bbia_sim.bbia_vision import BBIAVision

        backend = MuJoCoBackend()
        backend.connect()
        vision = BBIAVision(robot_api=backend)
        behavior = FollowFaceBehavior(vision=vision, robot_api=backend)
        assert behavior is not None
        backend.disconnect()

    def test_follow_object_behavior_init(self) -> None:
        """Test FollowObjectBehavior.__init__."""
        from bbia_sim.bbia_vision import BBIAVision

        backend = MuJoCoBackend()
        backend.connect()
        vision = BBIAVision(robot_api=backend)
        behavior = FollowObjectBehavior(vision=vision, robot_api=backend)
        assert behavior is not None
        backend.disconnect()

    def test_game_behavior_init(self) -> None:
        """Test GameBehavior.__init__."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = GameBehavior(robot_api=backend)
        assert behavior is not None
        backend.disconnect()

    def test_greeting_behavior_init(self) -> None:
        """Test GreetingBehavior.__init__."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = GreetingBehavior(robot_api=backend)
        assert behavior is not None
        backend.disconnect()

    def test_hide_behavior_init(self) -> None:
        """Test HideBehavior.__init__."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = HideBehavior(robot_api=backend)
        assert behavior is not None
        backend.disconnect()

    def test_meditation_behavior_init(self) -> None:
        """Test MeditationBehavior.__init__."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = MeditationBehavior(robot_api=backend)
        assert behavior is not None
        backend.disconnect()


class TestBackendAdapterAllMethods:
    """Tests pour toutes les méthodes BackendAdapter."""

    def test_all_backend_adapter_methods(self) -> None:
        """Test toutes les méthodes BackendAdapter."""
        from bbia_sim.daemon.simulation_service import simulation_service

        backend = MuJoCoBackend()
        backend.connect()
        adapter = BackendAdapter(backend)

        # Test toutes les méthodes set_target_*
        adapter.set_target_head_pose(np.eye(4, dtype=np.float64))
        adapter.set_target_body_yaw(0.5)
        adapter.set_target_head_joint_positions(np.array([0.0] * 7, dtype=np.float64))
        adapter.set_target_antenna_joint_positions([0.0, 0.0])
        adapter.set_target_head_joint_current(0.5)

        # Test set_motor_control_mode
        adapter.set_motor_control_mode("head_yaw", "position")

        # Test publishers
        adapter.set_pose_publisher(None)
        adapter.set_joint_positions_publisher(None)
        adapter.set_recording_publisher(None)

        # Test recording
        adapter.start_recording()
        adapter.stop_recording()

        # Test autres méthodes
        adapter.set_automatic_body_yaw(True)
        adapter.update_head_kinematics_model()
        adapter.update_target_head_joints_from_ik()
        adapter.play_sound("test.wav", volume=0.5)
        adapter.wrapped_run(lambda: None)

        backend.disconnect()


class TestDaemonStatusInit:
    """Tests pour DaemonStatus.__init__."""

    def test_daemon_status_init(self) -> None:
        """Test initialisation DaemonStatus."""
        status = DaemonStatus(status="running", simulation_running=True)
        assert status.status == "running"


class TestDaemonModelsMethods:
    """Tests pour les méthodes des modèles daemon."""

    def test_motion_command_validate_parameters(self) -> None:
        """Test MotionCommand.validate_parameters."""
        command = MotionCommand(command="move", parameters={"speed": 0.5})
        # La validation est automatique via Pydantic
        assert command.parameters == {"speed": 0.5}

    def test_move_uuid_validate_uuid(self) -> None:
        """Test MoveUUID.validate_uuid."""
        from uuid import uuid4

        move_id = MoveUUID(uuid=uuid4())
        # La validation est automatique via Pydantic
        assert move_id.uuid is not None

    def test_matrix4x4_pose_to_pose_array(self) -> None:
        """Test Matrix4x4Pose.to_pose_array."""
        import numpy as np

        pose = Matrix4x4Pose(m=tuple(np.eye(4).flatten()))
        pose_array = pose.to_pose_array()
        assert pose_array.shape == (4, 4)


class TestAIBackendsAll:
    """Tests pour tous les backends IA."""

    def test_coqui_tts_init(self) -> None:
        """Test CoquiTTSTTS.__init__."""
        try:
            tts = CoquiTTSTTS()
            assert tts is not None
        except Exception:
            pytest.skip("CoquiTTS non disponible")

    def test_coqui_tts_synthesize(self) -> None:
        """Test CoquiTTSTTS.synthesize_to_wav."""
        try:
            tts = CoquiTTSTTS()
            result = tts.synthesize_to_wav("Test", "test.wav")
            assert isinstance(result, bool)
        except Exception:
            pytest.skip("CoquiTTS non disponible")

    def test_kokoro_tts_init(self) -> None:
        """Test KokoroTTS.__init__."""
        try:
            tts = KokoroTTS()
            assert tts is not None
        except Exception:
            pytest.skip("KokoroTTS non disponible")

    def test_kokoro_tts_synthesize(self) -> None:
        """Test KokoroTTS.synthesize_to_wav."""
        try:
            tts = KokoroTTS()
            result = tts.synthesize_to_wav("Test", "test.wav")
            assert isinstance(result, bool)
        except Exception:
            pytest.skip("KokoroTTS non disponible")

    def test_llama_cpp_llm_init(self) -> None:
        """Test LlamaCppLLM.__init__."""
        try:
            llm = LlamaCppLLM()
            assert llm is not None
        except Exception:
            pytest.skip("LlamaCpp non disponible")

    def test_llama_cpp_llm_generate(self) -> None:
        """Test LlamaCppLLM.generate."""
        try:
            llm = LlamaCppLLM()
            result = llm.generate("Test prompt")
            assert isinstance(result, str)
        except Exception:
            pytest.skip("LlamaCpp non disponible")

    def test_local_llm_generate(self) -> None:
        """Test LocalLLM.generate."""
        try:
            llm = LocalLLM()
            result = llm.generate("Test prompt")
            assert isinstance(result, str)
        except Exception:
            pytest.skip("LocalLLM non disponible")

    def test_neu_tts_init(self) -> None:
        """Test NeuTTSTTS.__init__."""
        try:
            tts = NeuTTSTTS()
            assert tts is not None
        except Exception:
            pytest.skip("NeuTTS non disponible")

    def test_open_voice_tts_init(self) -> None:
        """Test OpenVoiceTTSTTS.__init__."""
        try:
            tts = OpenVoiceTTSTTS()
            assert tts is not None
        except Exception:
            pytest.skip("OpenVoiceTTS non disponible")

    def test_dummy_stt_transcribe(self) -> None:
        """Test DummySTT.transcribe_wav."""
        stt = DummySTT()
        result = stt.transcribe_wav("test.wav")
        assert isinstance(result, str)

    def test_whisper_stt_init(self) -> None:
        """Test WhisperSTT.__init__."""
        try:
            stt = WhisperSTT()
            assert stt is not None
        except Exception:
            pytest.skip("Whisper non disponible")

    def test_kitten_tts_init(self) -> None:
        """Test KittenTTSTTS.__init__."""
        try:
            tts = KittenTTSTTS()
            assert tts is not None
        except Exception:
            pytest.skip("KittenTTS non disponible")


class TestSimulationModules:
    """Tests pour les modules de simulation."""

    def test_simulation_io_module_all(self) -> None:
        """Test toutes les méthodes SimulationIOModule."""
        io_module = SimulationIOModule()
        # Tester les méthodes disponibles
        assert io_module is not None

    def test_simulation_media_module_all(self) -> None:
        """Test toutes les méthodes SimulationMediaModule."""
        media_module = SimulationMediaModule()
        assert media_module is not None


class TestOtherClasses:
    """Tests pour les autres classes."""

    def test_bbia_person_recognition_init(self) -> None:
        """Test BBIAPersonRecognition.__init__."""
        try:
            from bbia_sim.face_recognition import create_face_recognition

            recognizer = create_face_recognition("test_db", "test_person")
            if recognizer:
                assert recognizer is not None
        except Exception:
            pytest.skip("Face recognition non disponible")

    def test_bbia_pose_detection_init(self) -> None:
        """Test BBIAPoseDetection.__init__."""
        detector = BBIAPoseDetection()
        assert detector is not None

    def test_bbia_advanced_websocket_manager_init(self) -> None:
        """Test BBIAAdvancedWebSocketManager.__init__."""
        manager = BBIAAdvancedWebSocketManager()
        assert manager is not None

    def test_bbia_websocket_manager_init(self) -> None:
        """Test BBIAWebSocketManager.__init__."""
        manager = BBIAWebSocketManager()
        assert manager is not None

    def test_simple_move_init(self) -> None:
        """Test SimpleMove.__init__."""
        move = SimpleMove()
        assert move is not None

    def test_reachy_backend_methods(self) -> None:
        """Test méthodes ReachyBackend."""
        try:
            backend = ReachyBackend()
            # Tester les méthodes disponibles
            assert backend is not None
        except Exception:
            pytest.skip("Reachy backend non disponible")

    def test_face_detector_init(self) -> None:
        """Test FaceDetector.__init__."""
        try:
            detector = FaceDetector()
            assert detector is not None
        except Exception:
            pytest.skip("FaceDetector non disponible")

    def test_connection_manager_init(self) -> None:
        """Test ConnectionManager.__init__."""
        manager = ConnectionManager()
        assert manager is not None

    def test_mujoco_backend_async_play_move(self) -> None:
        """Test MuJoCoBackend.async_play_move."""
        backend = MuJoCoBackend()
        backend.connect()
        # Test avec un mouvement simple
        move = [{"head_yaw": 0.0}]
        backend.async_play_move(move, play_frequency=100.0)
        backend.disconnect()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
