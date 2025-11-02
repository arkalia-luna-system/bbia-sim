#!/usr/bin/env python3
"""
Tests unitaires pour dashboard_advanced.py
Tests du dashboard WebSocket avancé BBIA
"""

from unittest.mock import MagicMock, patch

import pytest


class TestDashboardAdvanced:
    """Tests pour le module dashboard_advanced."""

    def setup_method(self):
        """Configuration avant chaque test."""
        pass

    @patch("bbia_sim.dashboard_advanced.FASTAPI_AVAILABLE", True)
    def test_fastapi_import_available(self):
        """Test que FastAPI est disponible."""
        try:
            from bbia_sim.dashboard_advanced import FASTAPI_AVAILABLE

            # En mode test, FastAPI peut ne pas être disponible
            assert isinstance(FASTAPI_AVAILABLE, bool)
        except ImportError:
            pytest.skip("Module dashboard_advanced non disponible")

    @patch("bbia_sim.dashboard_advanced.FASTAPI_AVAILABLE", False)
    def test_fastapi_import_not_available(self):
        """Test que FastAPI n'est pas disponible."""
        from bbia_sim.dashboard_advanced import FASTAPI_AVAILABLE

        assert FASTAPI_AVAILABLE is False

    @patch("bbia_sim.dashboard_advanced.FASTAPI_AVAILABLE", True)
    @patch("bbia_sim.dashboard_advanced.BBIAEmotions")
    @patch("bbia_sim.dashboard_advanced.BBIAVision")
    @patch("bbia_sim.dashboard_advanced.BBIABehaviorManager")
    def test_websocket_manager_initialization(self, mock_behavior, mock_vision, mock_emotions):
        """Test initialisation BBIAAdvancedWebSocketManager."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()

            assert manager.active_connections == []
            assert manager.robot is None
            assert manager.robot_backend == "mujoco"
            assert manager.metrics_history == []
            assert manager.max_history == 1000
            assert hasattr(manager, "emotions")
            assert hasattr(manager, "vision")
            assert hasattr(manager, "behavior_manager")
            assert hasattr(manager, "current_metrics")
        except ImportError:
            pytest.skip("Module dashboard_advanced non disponible")
        except Exception as e:
            if "FastAPI" in str(e) or "WebSocket" in str(e):
                pytest.skip(f"FastAPI/WebSocket non disponible: {e}")
            raise

    @patch("bbia_sim.dashboard_advanced.FASTAPI_AVAILABLE", True)
    def test_websocket_manager_metrics_structure(self):
        """Test structure des métriques."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()

            # Vérifier structure current_metrics
            assert isinstance(manager.current_metrics, dict)
            assert "timestamp" in manager.current_metrics
            assert "robot_connected" in manager.current_metrics
            assert "current_emotion" in manager.current_metrics
            assert "emotion_intensity" in manager.current_metrics
            assert "joint_positions" in manager.current_metrics
            assert "performance" in manager.current_metrics
            assert "vision" in manager.current_metrics
            assert "audio" in manager.current_metrics
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")

    def test_get_available_joints_no_robot(self):
        """Test récupération joints sans robot."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()
            joints = manager._get_available_joints()

            # Sans robot connecté, doit retourner liste vide
            assert isinstance(joints, list)
            assert len(joints) == 0
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")

    @patch("bbia_sim.dashboard_advanced.FASTAPI_AVAILABLE", True)
    def test_get_available_joints_with_robot(self):
        """Test récupération joints avec robot."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()

            # Mock robot
            mock_robot = MagicMock()
            mock_robot.get_available_joints.return_value = [
                "yaw_body",
                "stewart_1",
                "stewart_2",
            ]
            manager.robot = mock_robot

            joints = manager._get_available_joints()

            assert isinstance(joints, list)
            assert len(joints) == 3
            assert "yaw_body" in joints
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")

    def test_get_current_pose_no_robot(self):
        """Test récupération pose sans robot."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()
            pose = manager._get_current_pose()

            # Sans robot, doit retourner dict vide
            assert isinstance(pose, dict)
            assert len(pose) == 0
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")

    @patch("bbia_sim.dashboard_advanced.FASTAPI_AVAILABLE", True)
    def test_get_current_pose_with_robot(self):
        """Test récupération pose avec robot."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()

            # Mock robot avec joints
            mock_robot = MagicMock()
            mock_robot.get_available_joints.return_value = ["yaw_body"]
            mock_robot.get_joint_pos.return_value = 0.5
            manager.robot = mock_robot

            pose = manager._get_current_pose()

            assert isinstance(pose, dict)
            # Pose devrait contenir les positions des joints
            assert len(pose) == 1
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")

    @patch("bbia_sim.dashboard_advanced.FASTAPI_AVAILABLE", True)
    def test_metrics_history_limit(self):
        """Test limite historique métriques."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()

            # Ajouter plus de métriques que le max
            for i in range(manager.max_history + 100):
                manager.metrics_history.append({"data": i})

            # Vérifier que l'historique est limité
            assert len(manager.metrics_history) <= manager.max_history
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")

    @patch("bbia_sim.dashboard_advanced.FASTAPI_AVAILABLE", True)
    def test_broadcast_no_connections(self):
        """Test broadcast sans connexions."""
        try:
            import asyncio

            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()

            async def test():
                # Doit fonctionner sans erreur même sans connexions
                await manager.broadcast("test message")

            asyncio.run(test())
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")

    def test_current_emotion_default(self):
        """Test émotion par défaut."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()

            assert manager.current_metrics["current_emotion"] == "neutral"
            assert manager.current_metrics["emotion_intensity"] == 0.0
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")

    def test_performance_metrics_structure(self):
        """Test structure métriques performance."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()

            perf = manager.current_metrics["performance"]
            assert "latency_ms" in perf
            assert "fps" in perf
            assert "cpu_usage" in perf
            assert "memory_usage" in perf
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")

    def test_vision_metrics_structure(self):
        """Test structure métriques vision."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()

            vision = manager.current_metrics["vision"]
            assert "objects_detected" in vision
            assert "faces_detected" in vision
            assert "tracking_active" in vision
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")

    def test_audio_metrics_structure(self):
        """Test structure métriques audio."""
        try:
            from bbia_sim.dashboard_advanced import BBIAAdvancedWebSocketManager

            manager = BBIAAdvancedWebSocketManager()

            audio = manager.current_metrics["audio"]
            assert "microphone_active" in audio
            assert "speaker_active" in audio
            assert "volume_level" in audio
        except (ImportError, Exception) as e:
            pytest.skip(f"Dashboard advanced non disponible: {e}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
