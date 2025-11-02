#!/usr/bin/env python3
"""Tests simples pour améliorer la coverage.

Tests basiques sans dépendances complexes.
"""

import os
import sys
import unittest

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim.daemon.config import Settings
from bbia_sim.daemon.models import JointPosition, MotionCommand, Pose


class TestBasicCoverage(unittest.TestCase):
    """Tests basiques pour améliorer la coverage."""

    def test_settings_default(self):
        """Test des paramètres par défaut."""
        settings = Settings()
        self.assertIsNotNone(settings.api_title)
        self.assertIsNotNone(settings.api_version)
        self.assertIsNotNone(settings.simulation_model_path)

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


if __name__ == "__main__":
    unittest.main()
