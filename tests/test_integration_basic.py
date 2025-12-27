#!/usr/bin/env python3
"""Tests d'intégration basiques pour BBIA-SIM."""

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.robot_factory import RobotFactory


class TestBasicIntegration:
    """Tests d'intégration basiques."""

    def test_robot_factory_creates_backend(self):
        """Test que RobotFactory crée correctement un backend."""
        backend = RobotFactory.create_backend("mujoco")
        assert backend is not None
        assert isinstance(backend, MuJoCoBackend)

    def test_backend_connect_disconnect_cycle(self):
        """Test cycle complet connect/disconnect."""
        backend = MuJoCoBackend()
        assert backend.connect() is True
        assert backend.is_connected is True
        assert backend.disconnect() is True
        assert backend.is_connected is False

    def test_backend_joint_operations(self):
        """Test opérations sur les joints."""
        backend = MuJoCoBackend()
        connected = backend.connect()
        assert connected is True, "La connexion au backend doit réussir"
        assert backend.is_connected is True

        joints = backend.get_available_joints()
        assert len(joints) > 0, "Le backend doit avoir au moins un joint disponible"

        # Test set/get joint position
        if joints:
            test_joint = joints[0]
            assert backend.set_joint_pos(test_joint, 0.1) is True
            new_pos = backend.get_joint_pos(test_joint)
            assert new_pos is not None

        backend.disconnect()

    def test_backend_goto_target(self):
        """Test goto_target."""
        backend = MuJoCoBackend()
        backend.connect()

        # Test goto_target avec antennes
        backend.goto_target(antennas=[0.1, -0.1], duration=0.5)

        backend.disconnect()

    def test_backend_status(self):
        """Test get_status."""
        backend = MuJoCoBackend()
        backend.connect()

        status = backend.get_status()
        assert "connected" in status
        assert "current_emotion" in status
        assert status["connected"] is True

        backend.disconnect()
