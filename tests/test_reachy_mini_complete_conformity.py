#!/usr/bin/env python3
"""
Test de conformité complète avec le SDK officiel Reachy-Mini
Validation de toutes les méthodes et signatures
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


class TestReachyMiniCompleteConformity:
    """Test de conformité complète avec le SDK officiel."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.backend = RobotFactory.create_backend("reachy_mini")
        assert self.backend is not None

    def test_robot_factory_integration(self):
        """Test intégration RobotFactory."""
        # Vérifier que reachy_mini est disponible
        backends = RobotFactory.get_available_backends()
        assert "reachy_mini" in backends

        # Vérifier les informations du backend
        info = RobotFactory.get_backend_info("reachy_mini")
        assert info["name"] == "Reachy-Mini SDK Officiel"
        assert info["real_robot"] is True

    def test_core_methods_conformity(self):
        """Test conformité méthodes principales."""
        # Méthodes de base
        assert hasattr(self.backend, "get_joint_pos")
        assert hasattr(self.backend, "set_joint_pos")
        assert hasattr(self.backend, "get_available_joints")
        assert hasattr(self.backend, "set_emotion")
        assert hasattr(self.backend, "look_at")
        assert hasattr(self.backend, "run_behavior")
        assert hasattr(self.backend, "get_telemetry")

    def test_sdk_official_methods_conformity(self):
        """Test conformité méthodes SDK officiel."""
        # Méthodes SDK officiel ajoutées
        sdk_methods = [
            "get_current_head_pose",
            "get_present_antenna_joint_positions",
            "set_target_body_yaw",
            "set_target_antenna_joint_positions",
            "look_at_image",
            "goto_target",
            "enable_motors",
            "disable_motors",
            "enable_gravity_compensation",
            "disable_gravity_compensation",
        ]

        for method_name in sdk_methods:
            assert hasattr(
                self.backend, method_name
            ), f"Méthode {method_name} manquante"

    def test_joint_mapping_conformity(self):
        """Test conformité mapping joints."""
        # Vérifier les joints officiels
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
        assert self.backend is not None  # Type narrowing pour mypy
        actual_joints = set(self.backend.get_available_joints())
        assert actual_joints == expected_joints

    def test_joint_positions_api_conformity(self):
        """Test conformité API positions joints."""
        assert self.backend is not None  # Type narrowing pour mypy
        # Test get_joint_pos avec tous les joints
        for joint in ["stewart_1", "stewart_2", "yaw_body"]:
            pos = self.backend.get_joint_pos(joint)
            assert isinstance(pos, float | type(None))
            if pos is not None:
                assert isinstance(pos, float)

    def test_emotion_api_conformity(self):
        """Test conformité API émotions."""
        assert self.backend is not None  # Type narrowing pour mypy
        valid_emotions = ["happy", "sad", "neutral", "excited", "curious", "calm"]

        for emotion in valid_emotions:
            result = self.backend.set_emotion(emotion, 0.8)
            assert result is True

        # Test émotion invalide
        result = self.backend.set_emotion("invalid_emotion", 0.5)
        assert result is False

    def test_behavior_api_conformity(self):
        """Test conformité API comportements."""
        assert self.backend is not None  # Type narrowing pour mypy
        valid_behaviors = ["wake_up", "goto_sleep", "nod"]

        for behavior in valid_behaviors:
            result = self.backend.run_behavior(behavior, 2.0)
            assert result is True

        # Test comportement invalide
        result = self.backend.run_behavior("invalid_behavior", 2.0)
        assert result is False

    def test_look_at_api_conformity(self):
        """Test conformité API look_at."""
        assert self.backend is not None  # Type narrowing pour mypy
        # Test look_at_world
        result = self.backend.look_at(0.1, 0.2, 0.3)
        assert result is True  # Notre méthode look_at retourne bool

        # Test look_at_image (SDK officiel retourne numpy array)
        result = self.backend.look_at_image(100, 200)  # type: ignore[attr-defined]
        assert hasattr(result, "shape")  # Doit être un numpy array

    def test_motor_control_conformity(self):
        """Test conformité contrôle moteurs."""
        assert self.backend is not None  # Type narrowing pour mypy
        # Test enable/disable motors (SDK officiel retourne None)
        result = self.backend.enable_motors()  # type: ignore[attr-defined]
        assert result is None

        result = self.backend.disable_motors()  # type: ignore[attr-defined]
        assert result is None

    def test_gravity_compensation_conformity(self):
        """Test conformité compensation gravité."""
        assert self.backend is not None  # Type narrowing pour mypy
        # Test enable/disable gravity compensation (SDK officiel retourne None)
        result = self.backend.enable_gravity_compensation()  # type: ignore[attr-defined]
        assert result is None

        result = self.backend.disable_gravity_compensation()  # type: ignore[attr-defined]
        assert result is None

    def test_target_control_conformity(self):
        """Test conformité contrôle cibles."""
        assert self.backend is not None  # Type narrowing pour mypy
        # Test set_target_body_yaw (SDK officiel retourne None)
        result = self.backend.set_target_body_yaw(0.1)  # type: ignore[attr-defined]
        assert result is None

        # Test set_target_antenna_joint_positions (SDK officiel retourne None)
        result = self.backend.set_target_antenna_joint_positions([0.1, 0.2])  # type: ignore[attr-defined]
        assert result is None

    def test_goto_target_conformity(self):
        """Test conformité goto_target."""
        assert self.backend is not None  # Type narrowing pour mypy
        # Test goto_target avec différents paramètres (SDK officiel retourne None)
        result = self.backend.goto_target()  # type: ignore[attr-defined]
        assert result is None

        result = self.backend.goto_target(body_yaw=0.1)  # type: ignore[attr-defined]
        assert result is None

        result = self.backend.goto_target(antennas=[0.1, 0.2])  # type: ignore[attr-defined]
        assert result is None

    def test_telemetry_conformity(self):
        """Test conformité télémétrie."""
        assert self.backend is not None  # Type narrowing pour mypy
        telemetry = self.backend.get_telemetry()  # type: ignore[attr-defined]
        assert isinstance(telemetry, dict)

        required_keys = [
            "step_count",
            "elapsed_time",
            "steps_per_second",
            "current_emotion",
            "emotion_intensity",
            "is_connected",
        ]
        for key in required_keys:
            assert key in telemetry

    def test_safety_conformity(self):
        """Test conformité sécurité."""
        assert self.backend is not None  # Type narrowing pour mypy
        # Test joints interdits
        forbidden_joints = self.backend.forbidden_joints
        # Note: Antennes maintenant optionnelles (commentées dans forbidden_joints)
        # Vérifier que les joints passifs sont toujours bloqués
        assert "passive_1" in forbidden_joints

        # Test amplitude limite
        assert self.backend.safe_amplitude_limit == 0.3

        # Test joints interdits rejetés
        for joint in forbidden_joints:
            result = self.backend.set_joint_pos(joint, 0.1)
            assert result is False

    def test_simulation_mode_conformity(self):
        """Test conformité mode simulation."""
        # En mode simulation, toutes les méthodes doivent fonctionner (SDK officiel retourne None)
        # IMPORTANT: Les joints stewart ne peuvent pas être contrôlés individuellement
        assert self.backend is not None  # Type narrowing pour mypy
        # Utiliser goto_target() ou look_at_world() à la place
        result = self.backend.set_joint_pos(
            "yaw_body", 0.1
        )  # Utiliser yaw_body au lieu de stewart_1
        assert result is True
        assert self.backend.set_emotion("happy", 0.8) is True
        assert self.backend.look_at(0.1, 0.2, 0.3) is True
        assert self.backend.run_behavior("wake_up") is True
        assert self.backend.enable_motors() is None  # type: ignore[attr-defined]  # SDK officiel retourne None
        assert (
            self.backend.enable_gravity_compensation() is None  # type: ignore[attr-defined]
        )  # SDK officiel retourne None

    def test_performance_conformity(self):
        """Test conformité performance."""
        import time

        assert self.backend is not None  # Type narrowing pour mypy
        # Test latence des méthodes critiques
        # Utiliser yaw_body au lieu de stewart_1 (stewart joints nécessitent IK)
        start_time = time.time()
        for _ in range(100):
            self.backend.set_joint_pos("yaw_body", 0.1)
            self.backend.get_joint_pos("yaw_body")
        end_time = time.time()

        avg_latency = (end_time - start_time) / 100 * 1000  # ms
        assert avg_latency < 1.0  # < 1ms en simulation


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
