#!/usr/bin/env python3
"""
Test des limites et du drift pour RobotAPI
Vérifie que les guards fonctionnent correctement
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


class TestRobotAPILimits:
    """Tests des limites et guards RobotAPI."""

    def test_joint_amplitude_limits(self):
        """Test que les amplitudes sont bien clampées."""
        robot = RobotFactory.create_backend("mujoco")
        assert robot is not None

        if not robot.connect():
            pytest.skip("MuJoCo non disponible")

        try:
            # Test amplitude trop élevée
            result = robot.set_joint_pos("yaw_body", 1.0)  # > 0.3 rad
            assert result is True  # Doit être clampée mais réussir

            # Vérifier que la position est bien clampée
            pos = robot.get_joint_pos("yaw_body")
            assert pos is not None
            assert abs(pos) <= 0.3  # Doit être dans les limites

        finally:
            robot.disconnect()

    def test_forbidden_joints_rejection(self):
        """Test que les joints interdits sont rejetés."""
        robot = RobotFactory.create_backend("mujoco")
        assert robot is not None

        if not robot.connect():
            pytest.skip("MuJoCo non disponible")

        try:
            # Test joint interdit
            result = robot.set_joint_pos("left_antenna", 0.1)
            assert result is False  # Doit échouer

            result = robot.set_joint_pos("passive_1", 0.1)
            assert result is False  # Doit échouer

        finally:
            robot.disconnect()

    def test_emotion_validation(self):
        """Test que les émotions sont bien validées."""
        robot = RobotFactory.create_backend("reachy")  # Plus rapide
        assert robot is not None

        if not robot.connect():
            pytest.skip("Reachy backend non disponible")

        try:
            # Test émotion valide
            result = robot.set_emotion("happy", 0.8)
            assert result is True

            # Test émotion invalide
            result = robot.set_emotion("invalid_emotion", 0.5)
            assert result is False

            # Test intensité hors limites
            result = robot.set_emotion("happy", 1.5)  # > 1.0
            assert result is True  # Doit être clampée

        finally:
            robot.disconnect()

    def test_drift_prevention(self):
        """Test que le drift est contrôlé."""
        robot = RobotFactory.create_backend("mujoco")
        assert robot is not None

        if not robot.connect():
            pytest.skip("MuJoCo non disponible")

        try:
            # Test plusieurs mouvements pour vérifier le drift
            positions = []
            for i in range(10):
                angle = 0.2 * (i % 2)  # Alternance 0.0 / 0.2
                robot.set_joint_pos("yaw_body", angle)
                robot.step()

                pos = robot.get_joint_pos("yaw_body")
                positions.append(pos)

            # Vérifier que les positions restent dans les limites
            for pos in positions:
                assert pos is not None
                assert abs(pos) <= 0.3

            # Vérifier qu'il n'y a pas de drift excessif
            filtered_positions = [p for p in positions if p is not None]
            max_pos = max(abs(p) for p in filtered_positions)
            assert max_pos <= 0.3

        finally:
            robot.disconnect()

    def test_backend_consistency(self):
        """Test que les deux backends ont le même comportement."""
        mujoco_robot = RobotFactory.create_backend("mujoco")
        reachy_robot = RobotFactory.create_backend("reachy")

        assert mujoco_robot is not None
        assert reachy_robot is not None

        # Test des joints interdits (même liste)
        assert mujoco_robot.forbidden_joints == reachy_robot.forbidden_joints

        # Test des limites d'amplitude (même valeur)
        assert mujoco_robot.safe_amplitude_limit == reachy_robot.safe_amplitude_limit

        # Test des émotions (même validation)
        mujoco_robot.connect()
        reachy_robot.connect()

        try:
            # Même émotion doit être valide/invalide sur les deux
            assert mujoco_robot.set_emotion("happy", 0.5) == reachy_robot.set_emotion(
                "happy", 0.5
            )
            assert mujoco_robot.set_emotion("invalid", 0.5) == reachy_robot.set_emotion(
                "invalid", 0.5
            )

        finally:
            mujoco_robot.disconnect()
            reachy_robot.disconnect()
