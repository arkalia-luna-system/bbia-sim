#!/usr/bin/env python3
"""
Tests avancés limites PID et safe_amplitude_limit.
Vérifie clamping avec bornes validées.
"""

import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


@pytest.mark.unit
@pytest.mark.fast
def test_safe_amplitude_limit_applied() -> None:
    """Test que safe_amplitude_limit est appliqué avec bornes validées."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        # Vérifier limite si accessible
        if hasattr(backend, "safe_amplitude_limit"):
            limit = backend.safe_amplitude_limit
            assert isinstance(limit, float | int)
            assert limit > 0.0, "Limite amplitude doit être positive"
            assert limit <= 0.5, "Limite amplitude trop élevée (> 0.5 rad)"

        # Tester avec valeur > limite
        test_joint = "yaw_body"
        large_angle = 1.0  # > 0.3 rad (limite typique)

        # Position doit être clampée
        result = backend.set_joint_pos(test_joint, large_angle)
        assert result is True  # Doit réussir (avec clamping)

        # Vérifier que position est bien clampée
        pos = backend.get_joint_pos(test_joint)
        assert pos is not None
        assert isinstance(pos, float | int)

        # Position doit être dans les limites de sécurité
        if hasattr(backend, "safe_amplitude_limit"):
            limit = backend.safe_amplitude_limit
            assert abs(pos) <= limit, f"Position {pos} dépasse limite {limit}"
        else:
            # Fallback: vérifier raisonnable (< 0.5 rad)
            assert abs(pos) <= 0.5, f"Position {pos} trop élevée"
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_pid_limits_multi_level_clamping() -> None:
    """Test clamping multi-niveaux (hardware + sécurité)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        test_joint = "yaw_body"
        extreme_values = [
            10.0,  # Très grand
            -10.0,  # Très petit
            0.5,  # Juste au-dessus limite sécurité
            -0.5,  # Juste en-dessous limite sécurité
        ]

        for value in extreme_values:
            result = backend.set_joint_pos(test_joint, value)
            assert result is True  # Doit réussir (avec clamping)

            pos = backend.get_joint_pos(test_joint)
            assert pos is not None
            assert isinstance(pos, float | int)

            # Position doit être dans limites raisonnables
            assert abs(pos) <= 3.2, f"Position {pos} dépasse limites hardware (π rad)"
            assert abs(pos) <= 0.5, f"Position {pos} dépasse limite sécurité"
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_stewart_joints_safe_limits() -> None:
    """Test limites sécurité joints stewart."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        stewart_joints = [
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "stewart_4",
            "stewart_5",
            "stewart_6",
        ]

        for joint in stewart_joints:
            # Joints stewart ont limites plus restrictives
            # Tester avec valeur raisonnable
            test_angle = 0.2  # Dans limites stewart (-0.5 à 0.5 rad)

            result = backend.set_joint_pos(joint, test_angle)
            # Peut réussir ou échouer selon implémentation

            if result:
                pos = backend.get_joint_pos(joint)
                if pos is not None:
                    # Vérifier dans limites stewart
                    assert abs(pos) <= 0.5, f"Position {pos} dépasse limites stewart"
    finally:
        backend.disconnect()
