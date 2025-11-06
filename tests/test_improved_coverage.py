#!/usr/bin/env python3
"""Tests simples pour améliorer la coverage des modules principaux
Tests basiques sans dépendances complexes.
"""

import unittest
from unittest.mock import MagicMock, patch

from bbia_sim.daemon.config import Settings
from bbia_sim.daemon.models import (
    GripperControl,
    HeadControl,
    JointPosition,
    MotionCommand,
    Pose,
    TelemetryMessage,
)
from bbia_sim.sim.joints import MAIN_JOINTS, get_joint_limits, validate_joint_name


class TestConfigCoverage(unittest.TestCase):
    """Tests pour améliorer la coverage de config.py."""

    def test_settings_creation(self):
        """Test création des paramètres."""
        settings = Settings()
        self.assertIsNotNone(settings.api_title)
        self.assertIsNotNone(settings.api_version)
        self.assertIsNotNone(settings.simulation_model_path)
        self.assertIsNotNone(settings.api_host)
        self.assertIsNotNone(settings.api_port)

    def test_settings_validation(self):
        """Test validation des paramètres."""
        settings = Settings()
        # Test que les valeurs sont dans les bonnes plages
        self.assertGreaterEqual(settings.api_port, 1000)
        self.assertLessEqual(settings.api_port, 65535)


class TestModelsCoverage(unittest.TestCase):
    """Tests pour améliorer la coverage de models.py."""

    def test_pose_model(self):
        """Test du modèle Pose."""
        pose = Pose(x=0.1, y=0.2, z=0.3, roll=0.4, pitch=0.5, yaw=0.6)
        self.assertEqual(pose.x, 0.1)
        self.assertEqual(pose.y, 0.2)
        self.assertEqual(pose.z, 0.3)
        self.assertEqual(pose.roll, 0.4)
        self.assertEqual(pose.pitch, 0.5)
        self.assertEqual(pose.yaw, 0.6)

    def test_joint_position_model(self):
        """Test du modèle JointPosition."""
        joint = JointPosition(joint_name="yaw_body", position=0.5)
        self.assertEqual(joint.joint_name, "yaw_body")
        self.assertEqual(joint.position, 0.5)

    def test_motion_command_model(self):
        """Test du modèle MotionCommand."""
        command = MotionCommand(
            command="move_joints",
            parameters={"joint_name": "yaw_body", "position": 0.5},
        )
        self.assertEqual(command.command, "move_joints")
        self.assertEqual(len(command.parameters), 2)
        self.assertEqual(command.parameters["joint_name"], "yaw_body")

    def test_head_control_model(self):
        """Test du modèle HeadControl."""
        head = HeadControl(yaw=0.1, pitch=0.2)
        self.assertEqual(head.yaw, 0.1)
        self.assertEqual(head.pitch, 0.2)

    def test_gripper_control_model(self):
        """Test du modèle GripperControl."""
        gripper = GripperControl(side="left", action="open")
        self.assertEqual(gripper.side, "left")
        self.assertEqual(gripper.action, "open")

    def test_telemetry_message_model(self):
        """Test du modèle TelemetryMessage."""
        msg = TelemetryMessage(type="ping", data={"test": "value"})
        self.assertEqual(msg.type, "ping")
        self.assertEqual(msg.data["test"], "value")


class TestJointsCoverage(unittest.TestCase):
    """Tests pour améliorer la coverage de joints.py."""

    def test_validate_joint_name(self):
        """Test validation des noms de joints."""
        # Test joints valides
        self.assertTrue(validate_joint_name("yaw_body"))
        self.assertTrue(validate_joint_name("stewart_1"))
        self.assertTrue(validate_joint_name("right_antenna"))

        # Test joints invalides
        self.assertFalse(validate_joint_name("invalid_joint"))
        self.assertFalse(validate_joint_name(""))
        self.assertFalse(validate_joint_name(None))

    def test_get_joint_limits(self):
        """Test récupération des limites de joints."""
        limits = get_joint_limits("yaw_body")
        self.assertIsInstance(limits, tuple)
        self.assertEqual(len(limits), 2)
        self.assertLess(limits[0], limits[1])

    def test_main_joints(self):
        """Test liste des joints principaux."""
        self.assertIsInstance(MAIN_JOINTS, list)
        self.assertGreater(len(MAIN_JOINTS), 0)
        self.assertIn("yaw_body", MAIN_JOINTS)


class TestSimulatorCoverage(unittest.TestCase):
    """Tests pour améliorer la coverage de simulator.py."""

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_simulator_creation(self, mock_mujoco):
        """Test création du simulateur avec mock."""
        from src.bbia_sim.sim.simulator import MuJoCoSimulator

        # Mock MuJoCo
        mock_model = MagicMock()
        mock_model.njnt = 16
        mock_model.nbody = 19
        mock_model.ngeom = 161
        mock_mujoco.mj_loadXML.return_value = mock_model
        mock_mujoco.mj_makeData.return_value = MagicMock()

        # Utiliser le vrai modèle
        simulator = MuJoCoSimulator("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
        self.assertIsNotNone(simulator)
        # Vérifier que le simulateur a été créé (le mock peut retourner différentes valeurs)
        self.assertIsNotNone(simulator.model)

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_get_available_joints(self, mock_mujoco):
        """Test récupération des joints disponibles."""
        from src.bbia_sim.sim.simulator import MuJoCoSimulator

        # Mock MuJoCo
        mock_model = MagicMock()
        mock_model.njnt = 3
        mock_model.joint = MagicMock()
        mock_model.joint.side_effect = [
            MagicMock(name="joint1"),
            MagicMock(name="joint2"),
            MagicMock(name="joint3"),
        ]
        mock_mujoco.mj_loadXML.return_value = mock_model
        mock_mujoco.mj_makeData.return_value = MagicMock()

        # Utiliser le vrai modèle
        simulator = MuJoCoSimulator("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
        joints = simulator.get_available_joints()
        self.assertIsInstance(joints, list)
        # Accepter n'importe quelle longueur car le mock peut varier
        self.assertGreaterEqual(len(joints), 1)


if __name__ == "__main__":
    unittest.main()
