#!/usr/bin/env python3
"""
Tests pour le backend Reachy-Mini SDK officiel
Validation complète de la conformité avec le SDK officiel
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend
from bbia_sim.mapping_reachy import ReachyMapping
from bbia_sim.robot_api import RobotFactory


class TestReachyMiniBackend:
    """Tests pour le backend Reachy-Mini SDK officiel."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.robot = RobotFactory.create_backend("reachy_mini")
        self.mapping = ReachyMapping()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_backend_creation(self):
        """Test création du backend."""
        assert isinstance(self.robot, ReachyMiniBackend)
        assert hasattr(self.robot, "joint_mapping")
        assert hasattr(self.robot, "joint_limits")
        assert hasattr(self.robot, "forbidden_joints")

    @pytest.mark.unit
    @pytest.mark.fast
    def test_joint_mapping(self):
        """Test du mapping des joints."""
        # Vérifier que tous les joints officiels sont mappés (SDK officiel)
        expected_joints = {
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "stewart_4",
            "stewart_5",
            "stewart_6",
            "left_antenna",
            "right_antenna",
            "yaw_body",
        }
        actual_joints = set(self.robot.get_available_joints())
        assert actual_joints == expected_joints

    def test_joint_limits(self):
        """Test des limites des joints."""
        # Vérifier que les limites sont définies (SDK officiel)
        for joint in ["stewart_1", "stewart_2", "yaw_body"]:
            assert joint in self.robot.joint_limits
            min_limit, max_limit = self.robot.joint_limits[joint]
            assert min_limit < max_limit

    def test_forbidden_joints(self):
        """Test des joints interdits."""
        expected_forbidden = {"left_antenna", "right_antenna"}
        assert self.robot.forbidden_joints == expected_forbidden

    def test_safe_amplitude_limit(self):
        """Test de la limite d'amplitude sécurisée."""
        assert self.robot.safe_amplitude_limit == 0.3

    def test_get_joint_pos_simulation(self):
        """Test lecture position joint en mode simulation."""
        # En mode simulation, doit retourner 0.0
        pos = self.robot.get_joint_pos("stewart_1")
        assert pos == 0.0

    def test_set_joint_pos_simulation(self):
        """Test définition position joint en mode simulation."""
        # En mode simulation, doit toujours réussir
        success = self.robot.set_joint_pos("stewart_1", 0.1)
        assert success is True

    def test_set_joint_pos_forbidden(self):
        """Test définition position joint interdit."""
        # Les joints interdits doivent être rejetés
        success = self.robot.set_joint_pos("left_antenna", 0.1)
        assert success is False

    def test_set_joint_pos_amplitude_clamp(self):
        """Test clamp de l'amplitude."""
        # Position au-delà de la limite doit être clampée
        success = self.robot.set_joint_pos("stewart_1", 0.5)  # > 0.3
        assert success is True  # Doit réussir mais être clampée

    def test_set_emotion_simulation(self):
        """Test définition émotion en mode simulation."""
        emotions = ["happy", "sad", "neutral", "excited", "curious", "calm"]

        for emotion in emotions:
            success = self.robot.set_emotion(emotion, 0.7)
            assert success is True
            assert self.robot.current_emotion == emotion
            assert self.robot.emotion_intensity == 0.7

    def test_set_emotion_invalid(self):
        """Test émotion invalide."""
        success = self.robot.set_emotion("invalid_emotion", 0.5)
        assert success is False

    def test_look_at_simulation(self):
        """Test look_at en mode simulation."""
        success = self.robot.look_at(0.1, 0.2, 0.3)
        assert success is True

    def test_run_behavior_simulation(self):
        """Test comportements en mode simulation."""
        behaviors = ["wake_up", "goto_sleep", "nod"]

        for behavior in behaviors:
            success = self.robot.run_behavior(behavior, 2.0)
            assert success is True

    def test_run_behavior_invalid(self):
        """Test comportement invalide."""
        success = self.robot.run_behavior("invalid_behavior", 2.0)
        assert success is False

    def test_step(self):
        """Test pas de simulation."""
        initial_count = self.robot.step_count
        success = self.robot.step()
        assert success is True
        assert self.robot.step_count == initial_count + 1

    def test_get_telemetry(self):
        """Test récupération télémétrie."""
        telemetry = self.robot.get_telemetry()
        assert isinstance(telemetry, dict)
        assert "step_count" in telemetry
        assert "current_emotion" in telemetry
        assert "emotion_intensity" in telemetry

    def test_connect_disconnect(self):
        """Test connexion/déconnexion."""
        # Test déconnexion
        success = self.robot.disconnect()
        assert success is True
        assert self.robot.is_connected is False

    def test_robot_factory_integration(self):
        """Test intégration avec RobotFactory."""
        # Test création via factory
        robot = RobotFactory.create_backend("reachy_mini")
        assert isinstance(robot, ReachyMiniBackend)

        # Test backends disponibles
        backends = RobotFactory.get_available_backends()
        assert "reachy_mini" in backends

    def test_mapping_integration(self):
        """Test intégration avec ReachyMapping."""
        # Test joints recommandés
        recommended = self.mapping.get_recommended_joints()
        assert isinstance(recommended, set)

        # Test validation position (SDK officiel)
        is_valid, clamped_pos = self.mapping.validate_position("stewart_1", 0.1)
        assert is_valid is True
        assert clamped_pos == 0.1

        # Test position invalide (au-delà de safe_amplitude)
        is_valid, clamped_pos = self.mapping.validate_position(
            "stewart_1", 1.0
        )  # > 0.2 safe_amplitude
        assert is_valid is True  # Valide mais clampée
        assert clamped_pos == 0.2  # Clampée à safe_amplitude


class TestReachyMiniBackendIntegration:
    """Tests d'intégration pour le backend Reachy-Mini."""

    def test_full_workflow_simulation(self):
        """Test workflow complet en mode simulation."""
        robot = RobotFactory.create_backend("reachy_mini")

        # Connexion simulée
        robot.is_connected = True

        # Test émotion
        success = robot.set_emotion("happy", 0.8)
        assert success is True

        # Test mouvement
        success = robot.set_joint_pos("stewart_1", 0.1)
        assert success is True

        # Test look_at
        success = robot.look_at(0.1, 0.0, 0.2)
        assert success is True

        # Test comportement
        success = robot.run_behavior("nod", 1.0)
        assert success is True

        # Test télémétrie
        telemetry = robot.get_telemetry()
        assert telemetry["current_emotion"] == "happy"
        assert telemetry["emotion_intensity"] == 0.8

    def test_safety_validation(self):
        """Test validation de sécurité."""
        robot = RobotFactory.create_backend("reachy_mini")

        # Test joints interdits
        for joint in robot.forbidden_joints:
            success = robot.set_joint_pos(joint, 0.1)
            assert success is False

        # Test amplitude limite
        success = robot.set_joint_pos("stewart_1", 0.4)  # > 0.3
        assert success is True  # Doit être clampée automatiquement

    def test_performance_simulation(self):
        """Test performance en mode simulation."""
        robot = RobotFactory.create_backend("reachy_mini")
        robot.is_connected = True

        import time

        # Test latence
        start_time = time.time()
        for _ in range(100):
            robot.set_joint_pos("stewart_1", 0.1)
            robot.get_joint_pos("stewart_1")
        end_time = time.time()

        avg_latency = (end_time - start_time) / 100 * 1000  # ms
        assert avg_latency < 1.0  # < 1ms en simulation


@pytest.mark.skip(reason="Test nécessite SDK reachy_mini installé")
class TestReachyMiniBackendReal:
    """Tests pour le backend avec SDK réel (nécessite robot physique)."""

    def test_real_connection(self):
        """Test connexion réelle (nécessite robot)."""
        robot = RobotFactory.create_backend("reachy_mini")

        # Ce test nécessite un robot physique connecté
        success = robot.connect()
        # Ne peut pas être testé sans robot physique
        assert isinstance(success, bool)

    def test_real_joint_control(self):
        """Test contrôle réel des joints (nécessite robot)."""
        robot = RobotFactory.create_backend("reachy_mini")

        # Ce test nécessite un robot physique connecté
        if robot.connect():
            success = robot.set_joint_pos("stewart_1", 0.1)
            assert isinstance(success, bool)
            robot.disconnect()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
