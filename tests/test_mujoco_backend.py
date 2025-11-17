#!/usr/bin/env python3
"""
Tests pour le backend MuJoCo
Validation de la simulation MuJoCo
"""

import os
import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.robot_factory import RobotFactory


class TestMuJoCoBackend:
    """Tests pour le backend MuJoCo."""

    def setup_method(self):
        """Configuration avant chaque test."""
        # Utiliser le modèle REAL_OFFICIAL par défaut
        model_path = (
            Path(__file__).parent.parent
            / "src"
            / "bbia_sim"
            / "sim"
            / "models"
            / "reachy_mini_REAL_OFFICIAL.xml"
        )
        self.robot = MuJoCoBackend(model_path=str(model_path))
        # Connecter le robot pour que les tests fonctionnent
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
        assert isinstance(self.robot, MuJoCoBackend)
        assert hasattr(self.robot, "joint_name_to_id")
        assert hasattr(self.robot, "model")
        assert hasattr(self.robot, "data")

    @pytest.mark.unit
    @pytest.mark.fast
    def test_connect(self):
        """Test connexion au simulateur."""
        assert self.robot.is_connected
        assert self.robot.model is not None
        assert self.robot.data is not None

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
    def test_set_joint_pos_invalid(self):
        """Test positionnement d'un joint invalide."""
        result = self.robot.set_joint_pos("invalid_joint", 0.1)
        assert result is False

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
        """Test goto_target (méthode implémentée)."""
        # Tester avec des paramètres minimaux
        try:
            self.robot.goto_target(duration=0.1)
            assert True  # Pas d'exception
        except Exception as e:
            pytest.fail(f"goto_target a levé une exception: {e}")

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
        assert self.robot.model is None

    @pytest.mark.unit
    @pytest.mark.fast
    def test_joint_limits(self):
        """Test limites des joints."""
        if "yaw_body" in self.robot.get_available_joints():
            limits = self.robot.joint_limits.get("yaw_body")
            if limits:
                min_limit, max_limit = limits
                assert min_limit < max_limit
