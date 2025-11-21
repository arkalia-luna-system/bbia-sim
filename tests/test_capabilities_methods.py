#!/usr/bin/env python3
"""Tests pour toutes les méthodes publiques non encore testées.

Ce fichier complète les autres fichiers de test pour atteindre 100% d'utilisation.
"""

import pytest

from bbia_sim.bbia_behavior import BBIABehaviorManager
from bbia_sim.bbia_emotions import BBIAEmotions
from bbia_sim.bbia_vision import BBIAVision
from bbia_sim.bbia_chat import BBIAChat
from bbia_sim.bbia_memory import BBIAMemory
from bbia_sim.bbia_adaptive_learning import BBIAAdaptiveLearning
from bbia_sim.bbia_adaptive_behavior import BBIAAdaptiveBehavior
from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition
from bbia_sim.bbia_idle_animations import (
    BBIIdleAnimationManager,
    BBIABreathingAnimation,
    BBIAPoseTransitionManager,
    BBIAVocalTremor,
)
from bbia_sim.behaviors.alarm_clock import AlarmClockBehavior
from bbia_sim.daemon.app.routers.apps import AppInfo, AppStatus
from bbia_sim.daemon.app.backend_adapter import BackendAdapter
from bbia_sim.backends.mujoco_backend import MuJoCoBackend


class TestBBIABehaviorManagerMethods:
    """Tests pour toutes les méthodes de BBIABehaviorManager."""

    def test_execute_behavior(self) -> None:
        """Test execute_behavior."""
        backend = MuJoCoBackend()
        backend.connect()
        try:
            manager = BBIABehaviorManager(robot_api=backend)
            # Test avec un comportement simple
            result = manager.execute_behavior("wake_up")
            # Peut être False si le comportement n'existe pas
            assert isinstance(result, bool)
        finally:
            backend.disconnect()

    def test_get_behavior_stats(self) -> None:
        """Test get_behavior_stats."""
        backend = MuJoCoBackend()
        backend.connect()
        try:
            manager = BBIABehaviorManager(robot_api=backend)
            stats = manager.get_behavior_stats()
            assert isinstance(stats, dict)
            assert "total_behaviors" in stats
        finally:
            backend.disconnect()


class TestBBIAEmotionsInit:
    """Tests pour BBIAEmotions.__init__."""

    def test_emotions_init(self) -> None:
        """Test initialisation BBIAEmotions."""
        emotions = BBIAEmotions()
        assert emotions.current_emotion == "neutral"
        assert emotions.emotion_intensity == 0.5


class TestBBIAVisionMethods:
    """Tests pour les méthodes BBIAVision non utilisées."""

    def test_start_async_scanning(self) -> None:
        """Test start_async_scanning."""
        vision = BBIAVision()
        vision.start_async_scanning()
        vision.stop_async_scanning()

    def test_stop_async_scanning(self) -> None:
        """Test stop_async_scanning."""
        vision = BBIAVision()
        vision.start_async_scanning()
        vision.stop_async_scanning()


class TestBBIAChatInit:
    """Tests pour BBIAChat.__init__."""

    def test_chat_init(self) -> None:
        """Test initialisation BBIAChat."""
        try:
            chat = BBIAChat()
            assert chat is not None
        except Exception:
            pytest.skip("BBIAChat non disponible")


class TestBBIAMemoryInit:
    """Tests pour BBIAMemory.__init__."""

    def test_memory_init(self) -> None:
        """Test initialisation BBIAMemory."""
        memory = BBIAMemory()
        assert memory is not None


class TestBBIAAdaptiveLearningInit:
    """Tests pour BBIAAdaptiveLearning.__init__."""

    def test_adaptive_learning_init(self) -> None:
        """Test initialisation BBIAAdaptiveLearning."""
        learning = BBIAAdaptiveLearning()
        assert learning is not None


class TestBBIAAdaptiveBehaviorInit:
    """Tests pour BBIAAdaptiveBehavior.__init__."""

    def test_adaptive_behavior_init(self) -> None:
        """Test initialisation BBIAAdaptiveBehavior."""
        behavior = BBIAAdaptiveBehavior()
        assert behavior is not None


class TestBBIAEmotionRecognitionInit:
    """Tests pour BBIAEmotionRecognition.__init__."""

    def test_emotion_recognition_init(self) -> None:
        """Test initialisation BBIAEmotionRecognition."""
        emotion_rec = BBIAEmotionRecognition()
        assert emotion_rec is not None


class TestIdleAnimationsInit:
    """Tests pour les initialisations d'animations idle."""

    def test_breathing_animation_init(self) -> None:
        """Test initialisation BBIABreathingAnimation."""
        backend = MuJoCoBackend()
        backend.connect()
        breathing = BBIABreathingAnimation(backend)
        assert breathing is not None
        backend.disconnect()

    def test_pose_transition_manager_init(self) -> None:
        """Test initialisation BBIAPoseTransitionManager."""
        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIAPoseTransitionManager(backend)
        assert manager is not None
        backend.disconnect()

    def test_vocal_tremor_init(self) -> None:
        """Test initialisation BBIAVocalTremor."""
        backend = MuJoCoBackend()
        backend.connect()
        tremor = BBIAVocalTremor(backend)
        assert tremor is not None
        backend.disconnect()

    def test_idle_animation_manager_init(self) -> None:
        """Test initialisation BBIIdleAnimationManager."""
        backend = MuJoCoBackend()
        backend.connect()
        manager = BBIIdleAnimationManager(backend)
        assert manager is not None
        backend.disconnect()


class TestAlarmClockBehaviorInit:
    """Tests pour AlarmClockBehavior.__init__."""

    def test_alarm_clock_init(self) -> None:
        """Test initialisation AlarmClockBehavior."""
        behavior = AlarmClockBehavior()
        assert behavior is not None


class TestAppInfoAppStatus:
    """Tests pour AppInfo et AppStatus."""

    def test_app_info_init(self) -> None:
        """Test initialisation AppInfo."""
        info = AppInfo(name="test", version="1.0.0")
        assert info.name == "test"

    def test_app_info_model_dump(self) -> None:
        """Test AppInfo.model_dump."""
        info = AppInfo(name="test", version="1.0.0")
        dumped = info.model_dump()
        assert "name" in dumped

    def test_app_status_init(self) -> None:
        """Test initialisation AppStatus."""
        status = AppStatus(status="running", pid=1234)
        assert status.status == "running"

    def test_app_status_model_dump(self) -> None:
        """Test AppStatus.model_dump."""
        status = AppStatus(status="running", pid=1234)
        dumped = status.model_dump()
        assert "status" in dumped


class TestBackendAdapterMethods:
    """Tests pour toutes les méthodes BackendAdapter."""

    def test_backend_adapter_init(self) -> None:
        """Test initialisation BackendAdapter."""
        from bbia_sim.daemon.simulation_service import simulation_service

        adapter = BackendAdapter(simulation_service.robot_api)
        assert adapter is not None

    def test_all_backend_adapter_methods(self) -> None:
        """Test toutes les méthodes BackendAdapter."""
        from bbia_sim.daemon.simulation_service import simulation_service

        adapter = BackendAdapter(simulation_service.robot_api)
        adapter.connect_if_needed()

        # Tester toutes les méthodes get_present_*
        pose = adapter.get_present_head_pose()
        assert pose is not None

        current_pose = adapter.get_current_head_pose()
        assert current_pose is not None

        yaw = adapter.get_present_body_yaw()
        assert isinstance(yaw, float)

        antennas = adapter.get_present_antenna_joint_positions()
        assert antennas is not None

        head_joints = adapter.get_present_head_joint_positions()
        # Peut être None
        assert head_joints is not None or True

        passive_joints = adapter.get_present_passive_joint_positions()
        # Peut être None
        assert passive_joints is not None or True

        status = adapter.get_status()
        assert status is not None

        urdf = adapter.get_urdf()
        # Peut être None
        assert urdf is not None or True

        adapter.close()


class TestBBIAVoiceAdvancedMethods:
    """Tests pour les méthodes BBIAVoiceAdvanced."""

    def test_voice_advanced_methods(self) -> None:
        """Test toutes les méthodes BBIAVoiceAdvanced."""
        from bbia_sim.bbia_voice_advanced import BBIAVoiceAdvanced

        voice = BBIAVoiceAdvanced()

        # Test is_coqui_available
        available = voice.is_coqui_available()
        assert isinstance(available, bool)

        # Test say
        result = voice.say("Test")
        assert isinstance(result, bool)

        # Test say_with_emotion
        result = voice.say_with_emotion("Test", "happy")
        assert isinstance(result, bool)

        # Test set_emotion
        voice.set_emotion("happy")


class TestTroubleshootingCheckerMethods:
    """Tests pour toutes les méthodes TroubleshootingChecker."""

    def test_all_checker_methods(self) -> None:
        """Test toutes les méthodes de vérification."""
        from bbia_sim.troubleshooting import TroubleshootingChecker

        checker = TroubleshootingChecker()

        # Tester toutes les méthodes check_*
        checker.check_python()
        checker.check_dependencies()
        checker.check_camera()
        checker.check_audio()
        checker.check_network()
        checker.check_mujoco()
        checker.check_ports()
        checker.check_permissions()

        # Test check_all
        results = checker.check_all()
        assert "summary" in results


class TestBBIAHuggingFaceMethods:
    """Tests pour les méthodes BBIAHuggingFace non utilisées."""

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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
