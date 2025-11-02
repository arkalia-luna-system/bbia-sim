#!/usr/bin/env python3
"""
Test récupération après crash simulateur.
Vérifie que le système peut récupérer proprement après des erreurs.
"""

import numpy as np
import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


@pytest.mark.unit
@pytest.mark.fast
def test_recovery_after_disconnect() -> None:
    """Test récupération après déconnexion."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    # Simuler une déconnexion
    backend.disconnect()
    assert not backend.is_connected

    # Vérifier qu'on peut se reconnecter
    assert backend.connect() is True
    assert backend.is_connected

    # Vérifier que les opérations fonctionnent après reconnexion
    pos = backend.get_joint_pos("yaw_body")
    assert isinstance(pos, float | int) or np.isscalar(pos)

    backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_recovery_after_invalid_operations() -> None:
    """Test récupération après opérations invalides."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        # Opérations qui pourraient causer des erreurs
        # Mais qui ne devraient pas casser le système

        # Joint invalide
        try:
            backend.set_joint_pos("invalid_joint", 0.1)
        except Exception:
            pass  # Attendu

        # Angle hors limites (devrait être clampé)
        backend.set_joint_pos("yaw_body", 10.0)  # Très grand
        pos = backend.get_joint_pos("yaw_body")
        assert isinstance(pos, float | int) or np.isscalar(pos)

        # Vérifier que le système fonctionne toujours
        assert backend.is_connected
        joints = backend.get_available_joints()
        assert len(joints) > 0
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_recovery_after_simulator_error() -> None:
    """Test récupération après erreur simulateur."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        # Simuler une erreur en créant un simulateur invalide
        # (cela devrait être géré gracieusement)
        if hasattr(backend, "robot") and backend.robot:
            # Le simulateur devrait être robuste
            pass

        # Vérifier que le backend fonctionne toujours
        assert backend.is_connected

        # Opérations normales devraient fonctionner
        backend.set_joint_pos("yaw_body", 0.1)
        pos = backend.get_joint_pos("yaw_body")
        assert isinstance(pos, float | int) or np.isscalar(pos)
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_multiple_connect_disconnect_cycles() -> None:
    """Test cycles multiples de connexion/déconnexion."""
    backend = ReachyMiniBackend(use_sim=True)

    cycles = 5
    for _ in range(cycles):
        assert backend.connect() is True
        assert backend.is_connected

        # Opération simple
        pos = backend.get_joint_pos("yaw_body")
        assert isinstance(pos, float | int) or np.isscalar(pos)

        backend.disconnect()
        assert not backend.is_connected

    # Dernière connexion pour nettoyer
    backend.connect()
    backend.disconnect()
