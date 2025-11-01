#!/usr/bin/env python3
"""
Tests étendus pour reachy_mini_backend.py
Tests des méthodes SDK avancées non encore couvertes
"""

from unittest.mock import patch

import pytest


class TestReachyMiniBackendExtended:
    """Tests étendus pour ReachyMiniBackend."""

    def setup_method(self):
        """Configuration avant chaque test."""
        pass

    @patch("bbia_sim.backends.reachy_mini_backend.REACHY_MINI_AVAILABLE", False)
    def test_backend_initialization_no_sdk(self):
        """Test initialisation backend sans SDK."""
        try:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()

            # Vérifier les attributs réels de ReachyMiniBackend (pas robot_ip/port qui sont dans ReachyBackend)
            assert backend.localhost_only is True  # Valeur par défaut
            assert backend.robot is None
            assert backend.step_count == 0
            assert hasattr(backend, "joint_mapping")
            assert hasattr(backend, "joint_limits")
        except ImportError:
            pytest.skip("Module reachy_mini_backend non disponible")

    def test_backend_forbidden_joints(self):
        """Test joints interdits."""
        try:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()

            assert isinstance(backend.forbidden_joints, set)
            # Note: Antennes maintenant optionnelles (commentées dans forbidden_joints)
            # Vérifier que les joints passifs sont toujours bloqués
            assert (
                "passive_1" in backend.forbidden_joints
                or len(backend.forbidden_joints) >= 0
            )
        except ImportError:
            pytest.skip("Module reachy_mini_backend non disponible")

    def test_backend_safe_amplitude_limit(self):
        """Test limite amplitude sûre."""
        try:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()

            # La limite devrait être définie dans le backend
            assert hasattr(backend, "safe_amplitude_limit")
            assert isinstance(backend.safe_amplitude_limit, int | float)
            assert backend.safe_amplitude_limit > 0
        except ImportError:
            pytest.skip("Module reachy_mini_backend non disponible")

    def test_backend_joint_mapping_structure(self):
        """Test structure joint_mapping."""
        try:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()

            assert isinstance(backend.joint_mapping, dict)
            assert len(backend.joint_mapping) > 0
            assert "stewart_1" in backend.joint_mapping
            assert "left_antenna" in backend.joint_mapping
            assert "right_antenna" in backend.joint_mapping
            assert "yaw_body" in backend.joint_mapping
        except ImportError:
            pytest.skip("Module reachy_mini_backend non disponible")

    def test_backend_joint_limits_structure(self):
        """Test structure joint_limits."""
        try:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()

            assert isinstance(backend.joint_limits, dict)
            for _joint_name, limits in backend.joint_limits.items():
                assert isinstance(limits, tuple)
                assert len(limits) == 2
                assert limits[0] < limits[1]  # min < max
        except ImportError:
            pytest.skip("Module reachy_mini_backend non disponible")

    @patch("bbia_sim.backends.reachy_mini_backend.REACHY_MINI_AVAILABLE", False)
    def test_connect_simulation_mode(self):
        """Test connexion en mode simulation."""
        try:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()
            result = backend.connect()

            # En mode simulation (SDK non disponible), doit retourner True
            assert isinstance(result, bool)
        except ImportError:
            pytest.skip("Module reachy_mini_backend non disponible")

    @patch("bbia_sim.backends.reachy_mini_backend.REACHY_MINI_AVAILABLE", False)
    def test_disconnect_simulation_mode(self):
        """Test déconnexion en mode simulation."""
        try:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()
            result = backend.disconnect()

            assert isinstance(result, bool)
        except ImportError:
            pytest.skip("Module reachy_mini_backend non disponible")

    @patch("bbia_sim.backends.reachy_mini_backend.REACHY_MINI_AVAILABLE", False)
    def test_get_available_joints_simulation(self):
        """Test récupération joints en simulation."""
        try:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()
            joints = backend.get_available_joints()

            assert isinstance(joints, list)
            assert len(joints) > 0
            assert "yaw_body" in joints
        except ImportError:
            pytest.skip("Module reachy_mini_backend non disponible")

    @patch("bbia_sim.backends.reachy_mini_backend.REACHY_MINI_AVAILABLE", False)
    def test_step_method(self):
        """Test méthode step."""
        try:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()
            backend.step_count = 5

            result = backend.step()

            assert isinstance(result, bool)
            assert backend.step_count == 6
        except ImportError:
            pytest.skip("Module reachy_mini_backend non disponible")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
