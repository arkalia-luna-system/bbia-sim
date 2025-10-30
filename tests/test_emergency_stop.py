#!/usr/bin/env python3
"""
Tests unitaires pour emergency_stop
Créé suite audit BBIA → Reachy Integration
"""

import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend
from bbia_sim.robot_api import RobotAPI


class TestEmergencyStop:
    """Tests arrêt d'urgence."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_emergency_stop_interface(self):
        """Test que emergency_stop existe dans RobotAPI."""
        assert hasattr(RobotAPI, "emergency_stop")

    @pytest.mark.unit
    @pytest.mark.fast
    def test_emergency_stop_simulation_mode(self):
        """Test emergency_stop en mode simulation."""
        backend = ReachyMiniBackend(use_sim=True)
        backend.connect()

        # En simulation sans robot, devrait retourner False
        result = backend.emergency_stop()
        assert result is False

        # Après emergency_stop, robot doit être déconnecté
        assert not backend.is_connected

    @pytest.mark.unit
    @pytest.mark.fast
    def test_emergency_stop_mujoco_backend(self):
        """Test emergency_stop avec MuJoCo backend."""
        from bbia_sim.backends.mujoco_backend import MuJoCoBackend

        backend = MuJoCoBackend()
        if backend.connect():
            result = backend.emergency_stop()
            assert result is True
            assert not backend.is_connected

    @pytest.mark.unit
    @pytest.mark.fast
    @pytest.mark.skip(reason="Nécessite robot physique ou mock complet")
    def test_emergency_stop_real_robot(self):
        """Test emergency_stop avec robot réel (skip si non disponible)."""
        backend = ReachyMiniBackend(use_sim=False)
        if backend.connect():
            result = backend.emergency_stop()
            assert result is True
            assert not backend.is_connected
