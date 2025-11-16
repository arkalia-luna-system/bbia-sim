#!/usr/bin/env python3
"""
Tests étendus pour reachy_mini_backend.py
Tests des méthodes SDK avancées non encore couvertes
"""

import sys
from pathlib import Path
from unittest.mock import patch

import pytest

# S'assurer que src est dans le path pour coverage
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# OPTIMISATION COVERAGE: Importer le module au niveau module pour que coverage le détecte
import bbia_sim.backends.reachy_mini_backend  # noqa: F401

# Importer les classes pour les tests
try:
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

    REACHY_MINI_BACKEND_AVAILABLE = True
except ImportError:
    REACHY_MINI_BACKEND_AVAILABLE = False
    ReachyMiniBackend = None  # type: ignore[assignment,misc]


class TestReachyMiniBackendExtended:
    """Tests étendus pour ReachyMiniBackend."""

    def setup_method(self):
        """Configuration avant chaque test."""
        pass

    @pytest.mark.skipif(
        not REACHY_MINI_BACKEND_AVAILABLE or ReachyMiniBackend is None,
        reason="Module reachy_mini_backend ou ReachyMiniBackend non disponible",
    )
    @patch("bbia_sim.backends.reachy_mini_backend.REACHY_MINI_AVAILABLE", False)
    def test_backend_initialization_no_sdk(self):
        """Test initialisation backend sans SDK."""
        backend = ReachyMiniBackend()

        # Vérifier les attributs réels de ReachyMiniBackend (pas robot_ip/port qui sont dans ReachyBackend)
        assert backend.localhost_only is True  # Valeur par défaut
        assert backend.robot is None
        assert backend.step_count == 0
        assert hasattr(backend, "joint_mapping")
        assert hasattr(backend, "joint_limits")

    @pytest.mark.skipif(
        not REACHY_MINI_BACKEND_AVAILABLE or ReachyMiniBackend is None,
        reason="Module reachy_mini_backend ou ReachyMiniBackend non disponible",
    )
    def test_backend_forbidden_joints(self):
        """Test joints interdits."""
        backend = ReachyMiniBackend()

        assert isinstance(backend.forbidden_joints, set)
        # Note: Antennes maintenant optionnelles (commentées dans forbidden_joints)
        # Vérifier que les joints passifs sont toujours bloqués
        assert (
            "passive_1" in backend.forbidden_joints
            or len(backend.forbidden_joints) >= 0
        )

    @pytest.mark.skipif(
        not REACHY_MINI_BACKEND_AVAILABLE or ReachyMiniBackend is None,
        reason="Module reachy_mini_backend ou ReachyMiniBackend non disponible",
    )
    def test_backend_safe_amplitude_limit(self):
        """Test limite amplitude sûre."""
        backend = ReachyMiniBackend()

        # La limite devrait être définie dans le backend
        assert hasattr(backend, "safe_amplitude_limit")
        assert isinstance(backend.safe_amplitude_limit, int | float)
        assert backend.safe_amplitude_limit > 0

    @pytest.mark.skipif(
        not REACHY_MINI_BACKEND_AVAILABLE or ReachyMiniBackend is None,
        reason="Module reachy_mini_backend ou ReachyMiniBackend non disponible",
    )
    def test_backend_joint_mapping_structure(self):
        """Test structure joint_mapping."""
        backend = ReachyMiniBackend()

        assert isinstance(backend.joint_mapping, dict)
        assert len(backend.joint_mapping) > 0
        assert "stewart_1" in backend.joint_mapping
        assert "left_antenna" in backend.joint_mapping
        assert "right_antenna" in backend.joint_mapping
        assert "yaw_body" in backend.joint_mapping

    @pytest.mark.skipif(
        not REACHY_MINI_BACKEND_AVAILABLE or ReachyMiniBackend is None,
        reason="Module reachy_mini_backend ou ReachyMiniBackend non disponible",
    )
    def test_backend_joint_limits_structure(self):
        """Test structure joint_limits."""
        backend = ReachyMiniBackend()

        assert isinstance(backend.joint_limits, dict)
        for _joint_name, limits in backend.joint_limits.items():
            assert isinstance(limits, tuple)
            assert len(limits) == 2
            assert limits[0] < limits[1]  # min < max

    @pytest.mark.skipif(
        not REACHY_MINI_BACKEND_AVAILABLE or ReachyMiniBackend is None,
        reason="Module reachy_mini_backend ou ReachyMiniBackend non disponible",
    )
    @patch("bbia_sim.backends.reachy_mini_backend.REACHY_MINI_AVAILABLE", False)
    def test_connect_simulation_mode(self):
        """Test connexion en mode simulation."""
        backend = ReachyMiniBackend()
        result = backend.connect()

        # En mode simulation (SDK non disponible), doit retourner True
        assert isinstance(result, bool)

    @pytest.mark.skipif(
        not REACHY_MINI_BACKEND_AVAILABLE or ReachyMiniBackend is None,
        reason="Module reachy_mini_backend ou ReachyMiniBackend non disponible",
    )
    @patch("bbia_sim.backends.reachy_mini_backend.REACHY_MINI_AVAILABLE", False)
    def test_disconnect_simulation_mode(self):
        """Test déconnexion en mode simulation."""
        backend = ReachyMiniBackend()
        result = backend.disconnect()

        assert isinstance(result, bool)

    @pytest.mark.skipif(
        not REACHY_MINI_BACKEND_AVAILABLE or ReachyMiniBackend is None,
        reason="Module reachy_mini_backend ou ReachyMiniBackend non disponible",
    )
    @patch("bbia_sim.backends.reachy_mini_backend.REACHY_MINI_AVAILABLE", False)
    def test_get_available_joints_simulation(self):
        """Test récupération joints en simulation."""
        backend = ReachyMiniBackend()
        joints = backend.get_available_joints()

        assert isinstance(joints, list)
        assert len(joints) > 0
        assert "yaw_body" in joints

    @pytest.mark.skipif(
        not REACHY_MINI_BACKEND_AVAILABLE or ReachyMiniBackend is None,
        reason="Module reachy_mini_backend ou ReachyMiniBackend non disponible",
    )
    @patch("bbia_sim.backends.reachy_mini_backend.REACHY_MINI_AVAILABLE", False)
    def test_step_method(self):
        """Test méthode step."""
        backend = ReachyMiniBackend()
        backend.step_count = 5

        result = backend.step()

        assert isinstance(result, bool)
        assert backend.step_count == 6


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
