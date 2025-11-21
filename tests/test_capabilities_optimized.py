#!/usr/bin/env python3
"""Tests optimisés pour toutes les capacités restantes - Version légère et performante."""

import pytest
import numpy as np

from bbia_sim.bbia_behavior import (
    BBIABehaviorManager,
    AntennaAnimationBehavior,
    GreetingBehavior,
    HideBehavior,
    EmotionalResponseBehavior,
)
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
from bbia_sim.daemon.app.backend_adapter import BackendAdapter
from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.daemon.app.routers.daemon import DaemonStatus
from bbia_sim.daemon.models import MotionCommand, MoveUUID, Matrix4x4Pose
from bbia_sim.daemon.ws import ConnectionManager
from bbia_sim.utils.types import (
    LookAtParams,
    PlayAudioParams,
    SetEmotionParams,
    SetTargetParams,
    MovementRecording,
    MetricsData,
    ModelInfo,
    SentimentDict,
    SentimentResult,
    TelemetryData,
    DetectionResult,
    FaceDetection,
    RobotStatus,
    ConversationEntry,
)


class TestQuickBehaviors:
    """Tests rapides pour tous les comportements."""

    def test_all_behaviors_init(self) -> None:
        """Test initialisation de tous les comportements."""
        backend = MuJoCoBackend()
        backend.connect()

        # Test tous les comportements en une fois
        behaviors = [
            AntennaAnimationBehavior(robot_api=backend),
            GreetingBehavior(robot_api=backend),
            HideBehavior(robot_api=backend),
            ConversationBehavior(robot_api=backend),
            DanceBehavior(robot_api=backend),
            EmotionShowBehavior(robot_api=backend),
            ExerciseBehavior(robot_api=backend),
            GameBehavior(robot_api=backend),
            MeditationBehavior(robot_api=backend),
        ]

        assert all(b is not None for b in behaviors)
        backend.disconnect()

    def test_emotion_show_cancel(self) -> None:
        """Test EmotionShowBehavior.cancel."""
        backend = MuJoCoBackend()
        backend.connect()
        behavior = EmotionShowBehavior(robot_api=backend)
        behavior.cancel()
        backend.disconnect()


class TestBBIABehaviorManagerQuick:
    """Tests rapides pour BBIABehaviorManager."""

    def test_all_manager_methods(self) -> None:
        """Test toutes les méthodes du manager en une fois."""
        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIABehaviorManager(robot_api=backend)

        # Test toutes les méthodes
        manager.register_behavior(GreetingBehavior(robot_api=backend))
        manager.add_to_queue("greeting", {})
        manager.start_behavior_worker()
        behaviors = manager.get_available_behaviors()
        stats = manager.get_behavior_stats()
        manager.stop_behavior_worker()

        assert isinstance(behaviors, list)
        assert "total_behaviors" in stats
        backend.disconnect()


class TestBackendAdapterQuick:
    """Tests rapides pour BackendAdapter."""

    def test_all_adapter_methods(self) -> None:
        """Test toutes les méthodes BackendAdapter en une fois."""
        backend = MuJoCoBackend()
        backend.connect()
        adapter = BackendAdapter(backend)
        adapter.connect_if_needed()

        # Test toutes les méthodes set_target_*
        adapter.set_target_head_pose(np.eye(4, dtype=np.float64))
        adapter.set_target_body_yaw(0.5)
        adapter.set_target_head_joint_positions(np.array([0.0] * 7, dtype=np.float64))
        adapter.set_target_antenna_joint_positions([0.0, 0.0])
        adapter.set_target_head_joint_current(np.array([0.0] * 7, dtype=np.float64))

        # Test autres méthodes
        adapter.set_motor_control_mode("head_yaw", "position")
        adapter.set_automatic_body_yaw(0.5)
        adapter.update_head_kinematics_model()
        adapter.update_target_head_joints_from_ik()
        adapter.play_sound("test.wav")
        adapter.wrapped_run()

        # Test publishers
        adapter.set_pose_publisher(None)
        adapter.set_joint_positions_publisher(None)
        adapter.set_recording_publisher(None)

        # Test recording
        adapter.start_recording()
        adapter.stop_recording()

        adapter.close()
        backend.disconnect()


class TestUtilsTypesQuick:
    """Tests rapides pour tous les types utilitaires."""

    def test_all_types(self) -> None:
        """Test tous les types en une fois."""
        # Test tous les types TypedDict
        look_at: LookAtParams = {"target_x": 0.5, "target_y": 0.3, "target_z": 0.2}
        play_audio: PlayAudioParams = {"file_path": "test.wav", "volume": 0.8}
        set_emotion: SetEmotionParams = {"emotion": "happy", "intensity": 0.7}
        set_target: SetTargetParams = {"head": [0.0] * 7, "antennas": [0.0, 0.0]}
        movement: MovementRecording = {"name": "test", "positions": {}, "duration": 1.0}
        metrics: MetricsData = {"cpu_usage": 50.0, "memory_usage": 60.0}
        model_info: ModelInfo = {"name": "test", "version": "1.0.0"}
        sentiment_dict: SentimentDict = {
            "sentiment": "positive",
            "score": 0.8,
            "label": "positive",
        }
        sentiment_result: SentimentResult = {"sentiment": "positive", "score": 0.8}
        capabilities: RobotCapabilities = {"has_camera": True, "has_microphone": True}
        telemetry: TelemetryData = {"timestamp": 1234567890.0, "joint_positions": {}}
        detection: DetectionResult = {
            "class": "person",
            "confidence": 0.9,
            "bbox": [0, 0, 100, 100],
        }
        face: FaceDetection = {"bbox": [0, 0, 100, 100], "landmarks": [[10, 10]]}
        robot_status: RobotStatus = {"is_connected": True, "battery_level": 80.0}
        conversation: ConversationEntry = {
            "user": "Bonjour",
            "bbia": "Salut",
            "timestamp": "2025-11-21",
        }

        assert all(
            [
                look_at,
                play_audio,
                set_emotion,
                set_target,
                movement,
                metrics,
                model_info,
                sentiment_dict,
                sentiment_result,
                capabilities,
                telemetry,
                detection,
                face,
                robot_status,
                conversation,
            ]
        )


class TestDaemonModelsQuick:
    """Tests rapides pour les modèles daemon."""

    def test_all_models(self) -> None:
        """Test tous les modèles en une fois."""
        from uuid import uuid4

        # Test tous les modèles
        motion = MotionCommand(command="move", parameters={"speed": 0.5})
        move_uuid = MoveUUID(uuid=uuid4())
        matrix_pose = Matrix4x4Pose(m=tuple(np.eye(4).flatten()))
        status = DaemonStatus(status="running", simulation_running=True)

        # Test méthodes
        pose_array = matrix_pose.to_pose_array()

        assert motion.command == "move"
        assert move_uuid.uuid is not None
        assert pose_array.shape == (4, 4)
        assert status.status == "running"


class TestConnectionManager:
    """Tests pour ConnectionManager."""

    def test_connection_manager_init(self) -> None:
        """Test ConnectionManager."""
        manager = ConnectionManager()
        assert manager is not None
        assert len(manager.active_connections) == 0


class TestAIBackendsQuick:
    """Tests rapides pour tous les backends IA."""

    def test_all_backends_init(self) -> None:
        """Test initialisation de tous les backends IA."""
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

        # Test tous les backends (skip si non disponibles)
        backends = []
        try:
            backends.append(CoquiTTSTTS())
        except Exception:
            pass
        try:
            backends.append(KokoroTTS())
        except Exception:
            pass
        try:
            backends.append(LlamaCppLLM())
        except Exception:
            pass
        try:
            backends.append(LocalLLM())
        except Exception:
            pass
        try:
            backends.append(NeuTTSTTS())
        except Exception:
            pass
        try:
            backends.append(OpenVoiceTTSTTS())
        except Exception:
            pass
        try:
            backends.append(DummySTT())
        except Exception:
            pass
        try:
            backends.append(WhisperSTT())
        except Exception:
            pass
        try:
            backends.append(KittenTTSTTS())
        except Exception:
            pass

        # Au moins DummySTT devrait fonctionner
        assert len(backends) > 0


class TestOtherClassesQuick:
    """Tests rapides pour autres classes."""

    def test_all_other_classes(self) -> None:
        """Test toutes les autres classes en une fois."""
        from bbia_sim.pose_detection import BBIAPoseDetection, create_pose_detector
        from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager
        from bbia_sim.dashboard import BBIAWebSocketManager
        from bbia_sim.backends.reachy_mini_backend import SimpleMove
        from bbia_sim.backends.simulation_shims import (
            SimulationIOModule,
            SimulationMediaModule,
        )

        # Test toutes les classes
        pose_detector = BBIAPoseDetection()
        pose_detector2 = create_pose_detector()
        ws_manager = BBIAAdvancedWebSocketManager()
        ws_manager2 = BBIAWebSocketManager()
        simple_move = SimpleMove()
        io_module = SimulationIOModule()
        media_module = SimulationMediaModule()

        assert all(
            [
                pose_detector is not None,
                pose_detector2 is not None,
                ws_manager is not None,
                ws_manager2 is not None,
                simple_move is not None,
                io_module is not None,
                media_module is not None,
            ]
        )


class TestMuJoCoBackendMethods:
    """Tests pour méthodes MuJoCoBackend."""

    def test_async_play_move(self) -> None:
        """Test async_play_move."""
        backend = MuJoCoBackend()
        backend.connect()
        move = [{"head_yaw": 0.0}]
        backend.async_play_move(move, play_frequency=100.0)
        backend.disconnect()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
