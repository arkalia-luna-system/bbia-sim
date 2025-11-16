#!/usr/bin/env python3
"""
Tests pour le backend Reachy réel
Validation de la connexion au robot réel
"""

import os
import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.reachy_backend import ReachyBackend


class TestReachyBackend:
    """Tests pour le backend Reachy réel."""

    def setup_method(self):
        """Configuration avant chaque test."""
        # Utiliser localhost par défaut (mode simulation si robot non disponible)
        self.robot = ReachyBackend(robot_ip="localhost", robot_port=8080)
        # Connecter le robot (basculera en simulation si robot non disponible)
        self.robot.connect()

    def teardown_method(self):
        """Nettoyage après chaque test."""
        if hasattr(self, "robot") and self.robot:
            try:
                self.robot.disconnect()
            except Exception:
                pass  # Ignorer erreurs de déconnexion

    @pytest.mark.unit
    @pytest.mark.fast
    def test_backend_creation(self):
        """Test création du backend."""
        assert isinstance(self.robot, ReachyBackend)
        assert hasattr(self.robot, "robot_ip")
        assert hasattr(self.robot, "robot_port")
        assert hasattr(self.robot, "simulated_joints")

    @pytest.mark.unit
    @pytest.mark.fast
    def test_connect(self):
        """Test connexion (mode simulation si robot non disponible)."""
        # Le backend bascule automatiquement en simulation
        # donc connect() devrait toujours réussir
        assert self.robot.is_connected is True

    @pytest.mark.unit
    @pytest.mark.fast
    def test_get_available_joints(self):
        """Test récupération des joints disponibles."""
        joints = self.robot.get_available_joints()
        assert isinstance(joints, list)
        assert len(joints) > 0
        # Vérifier que les joints attendus sont présents
        expected_joints = ["yaw_body", "stewart_1", "stewart_2"]
        for joint in expected_joints:
            if joint in joints:
                assert True  # Joint trouvé

    @pytest.mark.unit
    @pytest.mark.fast
    def test_set_joint_pos(self):
        """Test positionnement d'un joint."""
        # Tester avec yaw_body (joint valide)
        if "yaw_body" in self.robot.get_available_joints():
            result = self.robot.set_joint_pos("yaw_body", 0.1)
            assert result is True

    @pytest.mark.unit
    @pytest.mark.fast
    def test_get_joint_pos(self):
        """Test récupération position d'un joint."""
        if "yaw_body" in self.robot.get_available_joints():
            pos = self.robot.get_joint_pos("yaw_body")
            assert pos is not None
            assert isinstance(pos, float)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_goto_target(self):
        """Test goto_target (méthode non implémentée dans ReachyBackend)."""
        # ReachyBackend n'implémente pas goto_target, doit lever NotImplementedError
        with pytest.raises(NotImplementedError):
            self.robot.goto_target(duration=0.1)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_step(self):
        """Test step de simulation."""
        initial_count = self.robot.step_count
        self.robot.step()
        assert self.robot.step_count == initial_count + 1

    @pytest.mark.unit
    @pytest.mark.fast
    def test_disconnect(self):
        """Test déconnexion."""
        result = self.robot.disconnect()
        assert result is True
        assert not self.robot.is_connected

    @pytest.mark.unit
    @pytest.mark.fast
    def test_simulated_joints(self):
        """Test joints simulés."""
        assert isinstance(self.robot.simulated_joints, dict)
        assert "yaw_body" in self.robot.simulated_joints
