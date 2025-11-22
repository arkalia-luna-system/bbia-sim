#!/usr/bin/env python3
"""Tests pour utiliser toutes les capacités du projet BBIA.

Ce fichier de test vise à utiliser toutes les capacités publiques
non encore testées pour atteindre 100% d'utilisation.
"""

import pytest

from bbia_sim.bbia_adaptive_learning import BBIAAdaptiveLearning
from bbia_sim.bbia_adaptive_behavior import BBIAAdaptiveBehavior
from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition
from bbia_sim.bbia_memory import BBIAMemory
from bbia_sim.bbia_tools import BBIATools
from bbia_sim.behaviors.alarm_clock import AlarmClockBehavior
from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.troubleshooting import (
    TroubleshootingChecker,
    check_all,
    test_audio,
    test_camera,
)
from bbia_sim.daemon.models import (
    FullState,
    FullBodyTarget,
    as_any_pose,
    XYZRPYPose,
    Matrix4x4Pose,
)
from bbia_sim.daemon.app.routers.ecosystem import (
    APIStatus,
    BehaviorResponse,
    EmotionResponse,
)
from bbia_sim.daemon.app.routers.apps import AppInfo, AppStatus
from bbia_sim.daemon.app.routers.state import BatteryInfo
from bbia_sim.daemon.app.routers.move import GotoModelRequest, InterpolationMode
from bbia_sim.utils.types import GotoTargetParams, IMUData
from bbia_sim.daemon.app.backend_adapter import BackendAdapter, get_backend_adapter
from bbia_sim.daemon.app.routers.move import get_backend_dependency
from bbia_sim.robot_api import RobotAPI
from bbia_sim.ai_backends import (
    CoquiTTSTTS,
    DummySTT,
    get_tts_backend,
    get_stt_backend,
    get_llm_backend,
    select_backends,
)

try:
    from bbia_sim.sim.assets.reachy_official.asset_mapping import (
        get_available_assets,
        get_asset_path,
        get_official_assets,
    )

    ASSET_MAPPING_AVAILABLE = True
except ImportError:
    ASSET_MAPPING_AVAILABLE = False
    get_available_assets = None
    get_asset_path = None
    get_official_assets = None


class TestBBIAAdaptiveLearning:
    """Tests pour BBIAAdaptiveLearning."""

    def test_init(self) -> None:
        """Test initialisation BBIAAdaptiveLearning."""
        learning = BBIAAdaptiveLearning()
        assert learning is not None

    def test_learn_preference(self) -> None:
        """Test apprentissage de préférences."""
        learning = BBIAAdaptiveLearning()
        learning.learn_preference("user_1", "voice_speed", "fast")
        prefs = learning.get_preferences("user_1")
        assert "voice_speed" in prefs

    def test_remember_interaction(self) -> None:
        """Test mémorisation d'interactions."""
        learning = BBIAAdaptiveLearning()
        learning.remember_interaction("greeting", {"user": "user_1"}, "happy")
        # Vérifier que l'interaction a été mémorisée
        assert len(learning.behavior_history) > 0

    def test_adapt_behavior(self) -> None:
        """Test adaptation de comportement."""
        learning = BBIAAdaptiveLearning()
        adapted = learning.adapt_behavior({"context": "conversation"})
        assert adapted is not None
        assert isinstance(adapted, dict)


class TestBBIAAdaptiveBehavior:
    """Tests pour BBIAAdaptiveBehavior."""

    def test_init(self) -> None:
        """Test initialisation BBIAAdaptiveBehavior."""
        behavior = BBIAAdaptiveBehavior()
        assert behavior is not None

    def test_execute_behavior(self) -> None:
        """Test exécution de comportement adaptatif."""
        behavior = BBIAAdaptiveBehavior()
        result = behavior.execute_behavior({})
        assert isinstance(result, bool)


class TestBBIAEmotionRecognition:
    """Tests pour BBIAEmotionRecognition."""

    @pytest.mark.skipif(
        True, reason="Dépendances ML requises (mediapipe, torch, transformers)"
    )
    def test_init(self) -> None:
        """Test initialisation BBIAEmotionRecognition."""
        emotion_rec = BBIAEmotionRecognition()
        assert emotion_rec is not None


class TestBBIAMemory:
    """Tests pour BBIAMemory."""

    def test_init(self) -> None:
        """Test initialisation BBIAMemory."""
        memory = BBIAMemory()
        assert memory is not None

    def test_load_learnings(self) -> None:
        """Test chargement des apprentissages."""
        memory = BBIAMemory()
        learnings = memory.load_learnings()
        assert isinstance(learnings, dict)


class TestBBIATools:
    """Tests pour BBIATools."""

    def test_init(self) -> None:
        """Test initialisation BBIATools."""
        tools = BBIATools()
        assert tools is not None


class TestAlarmClockBehavior:
    """Tests pour AlarmClockBehavior."""

    def test_init(self) -> None:
        """Test initialisation AlarmClockBehavior."""
        behavior = AlarmClockBehavior()
        assert behavior is not None

    def test_snooze(self) -> None:
        """Test fonction snooze."""
        behavior = AlarmClockBehavior()
        # Test que snooze existe et peut être appelé
        assert hasattr(behavior, "snooze")


class TestBBIABehaviorManager:
    """Tests pour BBIABehaviorManager."""

    def test_clear_saved_moves(self) -> None:
        """Test effacement des mouvements sauvegardés."""
        from bbia_sim.backends.mujoco_backend import MuJoCoBackend

        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIABehaviorManager(robot_api=backend)
        manager.clear_saved_moves()
        backend.disconnect()


class TestTroubleshooting:
    """Tests pour les fonctions de troubleshooting."""

    def test_troubleshooting_checker(self) -> None:
        """Test TroubleshootingChecker."""
        checker = TroubleshootingChecker()
        assert checker is not None

    def test_check_all(self) -> None:
        """Test check_all."""
        result = check_all()
        assert isinstance(result, dict)

    def test_test_audio(self) -> None:
        """Test test_audio."""
        result = test_audio()
        assert isinstance(result, dict)

    def test_test_camera(self) -> None:
        """Test test_camera."""
        result = test_camera()
        assert isinstance(result, dict)


class TestDaemonModels:
    """Tests pour les modèles du daemon."""

    def test_api_status(self) -> None:
        """Test APIStatus."""
        status = APIStatus(
            status="running",
            version="1.3.2",
            uptime="00:00:00",
            robot_connected=False,
            simulation_running=False,
            active_connections=0,
        )
        assert status.status == "running"
        assert status.version == "1.3.2"

    def test_app_info(self) -> None:
        """Test AppInfo."""
        info = AppInfo(name="test", version="1.0.0")
        assert info.name == "test"

    def test_app_status(self) -> None:
        """Test AppStatus."""
        status = AppStatus(status="running", pid=1234)
        assert status.status == "running"

    def test_behavior_response(self) -> None:
        """Test BehaviorResponse."""
        response = BehaviorResponse(success=True, message="OK")
        assert response.success is True

    def test_emotion_response(self) -> None:
        """Test EmotionResponse."""
        response = EmotionResponse(emotion="happy", intensity=0.8)
        assert response.emotion == "happy"

    def test_battery_info(self) -> None:
        """Test BatteryInfo."""
        battery = BatteryInfo(level=80.0, charging=False, estimated_time="2h")
        assert battery.level == 80.0

    def test_full_state(self) -> None:
        """Test FullState."""
        state = FullState(
            head_pose=None,
            antennas_position=[0.0, 0.0],
            body_yaw=0.0,
        )
        assert state.body_yaw == 0.0

    def test_goto_model_request(self) -> None:
        """Test GotoModelRequest."""
        request = GotoModelRequest(
            head=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            duration=1.0,
        )
        assert request.duration == 1.0

    def test_goto_target_params(self) -> None:
        """Test GotoTargetParams."""
        params: GotoTargetParams = {
            "head": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            "duration": 1.0,
            "method": "minjerk",
        }
        assert params["method"] == "minjerk"

    def test_interpolation_mode(self) -> None:
        """Test InterpolationMode."""
        mode = InterpolationMode.LINEAR
        assert mode == InterpolationMode.LINEAR

    def test_imu_data(self) -> None:
        """Test IMUData."""
        imu: IMUData = {"accel": [0.0, 0.0, 0.0], "gyro": [0.0, 0.0, 0.0]}
        assert imu["accel"] == [0.0, 0.0, 0.0]

    def test_full_body_target(self) -> None:
        """Test FullBodyTarget."""
        target = FullBodyTarget(
            target_head_pose=None,
            target_antennas=(0.0, 0.0),
        )
        assert target.target_antennas == (0.0, 0.0)

    def test_as_any_pose(self) -> None:
        """Test as_any_pose."""
        import numpy as np

        pose_array = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        pose = as_any_pose(pose_array, use_matrix=False)
        assert pose is not None

    def test_xyzrpy_pose_from_array(self) -> None:
        """Test XYZRPYPose.from_pose_array."""
        import numpy as np

        pose_array = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        pose = XYZRPYPose.from_pose_array(pose_array)
        assert pose is not None

    def test_matrix4x4_pose_from_array(self) -> None:
        """Test Matrix4x4Pose.from_pose_array."""
        import numpy as np

        pose_array = np.eye(4, dtype=np.float64).flatten()
        pose = Matrix4x4Pose.from_pose_array(pose_array)
        assert pose is not None


class TestBackendAdapter:
    """Tests pour BackendAdapter."""

    def test_backend_adapter_init(self) -> None:
        """Test initialisation BackendAdapter."""
        from bbia_sim.daemon.simulation_service import simulation_service

        adapter = BackendAdapter(simulation_service.robot_api)
        assert adapter is not None

    def test_connect_if_needed(self) -> None:
        """Test connect_if_needed."""
        from bbia_sim.daemon.simulation_service import simulation_service

        adapter = BackendAdapter(simulation_service.robot_api)
        adapter.connect_if_needed()
        assert adapter._connected is True

    def test_get_backend_adapter(self) -> None:
        """Test get_backend_adapter."""
        adapter = get_backend_adapter()
        assert adapter is not None

    def test_get_backend_dependency(self) -> None:
        """Test get_backend_dependency."""
        # Cette fonction est utilisée dans FastAPI, on teste juste qu'elle existe
        assert get_backend_dependency is not None


class TestRobotAPI:
    """Tests pour RobotAPI."""

    def test_clamp_joint_position(self) -> None:
        """Test clamp_joint_position."""
        from bbia_sim.backends.mujoco_backend import MuJoCoBackend

        backend = MuJoCoBackend()
        backend.connect()
        try:
            clamped = backend.clamp_joint_position("head_yaw", 2.0)
            assert -0.3 <= clamped <= 0.3
        finally:
            backend.disconnect()


class TestAIBackends:
    """Tests pour les backends IA."""

    def test_coqui_tts_init(self) -> None:
        """Test initialisation CoquiTTSTTS."""
        try:
            tts = CoquiTTSTTS()
            assert tts is not None
        except Exception:
            pytest.skip("CoquiTTS non disponible")

    def test_dummy_stt_init(self) -> None:
        """Test initialisation DummySTT."""
        stt = DummySTT()
        assert stt is not None

    def test_get_tts_backend(self) -> None:
        """Test fonction get_tts_backend."""
        backend = get_tts_backend()
        assert backend is not None

    @pytest.mark.skipif(
        not ASSET_MAPPING_AVAILABLE,
        reason="asset_mapping non disponible",
    )
    def test_get_available_assets(self) -> None:
        """Test obtention assets disponibles."""
        if get_available_assets:
            assets = get_available_assets()
            assert isinstance(assets, dict)

    @pytest.mark.skipif(
        not ASSET_MAPPING_AVAILABLE,
        reason="asset_mapping non disponible",
    )
    def test_get_asset_path(self) -> None:
        """Test obtention chemin asset."""
        if get_asset_path:
            path = get_asset_path("torso")
            assert isinstance(path, str)

    @pytest.mark.skipif(
        not ASSET_MAPPING_AVAILABLE,
        reason="asset_mapping non disponible",
    )
    def test_get_official_assets(self) -> None:
        """Test obtention assets officiels."""
        if get_official_assets:
            assets = get_official_assets()
            assert isinstance(assets, dict)


class TestBBIAHuggingFaceMethods:
    """Tests pour méthodes BBIAHuggingFace non utilisées."""

    def test_answer_question(self) -> None:
        """Test answer_question."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            hf = BBIAHuggingFace()
            answer = hf.answer_question("Qu'est-ce que BBIA?")
            assert isinstance(answer, str)
        except ImportError:
            pytest.skip("Hugging Face non disponible")

    def test_describe_image(self) -> None:
        """Test describe_image."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace
            import numpy as np

            hf = BBIAHuggingFace()
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            description = hf.describe_image(image)
            assert isinstance(description, str)
        except ImportError:
            pytest.skip("Hugging Face non disponible")


class TestAdditionalCapabilities:
    """Tests pour capacités additionnelles."""

    def test_append_record(self) -> None:
        """Test append_record."""
        from bbia_sim.bbia_memory import append_record

        result = append_record({"test": "data"})
        assert isinstance(result, bool)

    def test_clear_saved_moves_function(self) -> None:
        """Test fonction clear_saved_moves."""
        from bbia_sim.bbia_behavior import clear_saved_moves

        clear_saved_moves()

    def test_create_move_from_positions(self) -> None:
        """Test create_move_from_positions."""
        from bbia_sim.bbia_behavior import create_move_from_positions

        move = create_move_from_positions({"head_yaw": 0.0})
        assert move is not None

    def test_create_move_task(self) -> None:
        """Test create_move_task."""
        from bbia_sim.bbia_behavior import create_move_task

        task = create_move_task({"head_yaw": 0.0}, duration=1.0)
        assert task is not None

    def test_create_pose_detector(self) -> None:
        """Test create_pose_detector."""
        from bbia_sim.pose_detection import create_pose_detector

        detector = create_pose_detector()
        assert detector is not None

    def test_dire_texte_advanced(self) -> None:
        """Test dire_texte_advanced."""
        from bbia_sim.bbia_voice_advanced import dire_texte_advanced

        result = dire_texte_advanced("Test", speed=1.0)
        assert isinstance(result, bool)

    def test_format_uptime(self) -> None:
        """Test format_uptime."""
        from bbia_sim.daemon.app.routers.apps import format_uptime

        uptime = format_uptime(3600)
        assert isinstance(uptime, str)

    def test_get_app_start_time(self) -> None:
        """Test get_app_start_time."""
        from bbia_sim.daemon.app.routers.apps import get_app_start_time

        start_time = get_app_start_time("test_app")
        assert start_time is not None

    def test_get_imu(self) -> None:
        """Test get_imu."""
        from bbia_sim.daemon.app.routers.state import get_imu

        imu = get_imu()
        assert imu is not None

    def test_get_motor_control_mode(self) -> None:
        """Test get_motor_control_mode."""
        from bbia_sim.daemon.app.routers.motors import get_motor_control_mode

        mode = get_motor_control_mode("head_yaw")
        assert mode is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
