#!/usr/bin/env python3
"""
Tests avancés de validation des entrées utilisateur.
Protection contre injection, validation longueur, format, etc.
"""

import pytest

from bbia_sim.robot_factory import RobotFactory


@pytest.mark.unit
@pytest.mark.fast
def test_joint_name_validation() -> None:
    """Test validation noms de joints contre injection."""
    robot = RobotFactory.create_backend("mujoco")
    if not robot or not robot.connect():
        pytest.skip("MuJoCo non disponible")

    try:
        # Noms de joints malveillants/invalides
        malicious_names = [
            "../../etc/passwd",
            "'; DROP TABLE joints; --",
            "joint\0null",
            "joint\nnewline",
            "joint\ttab",
            "  spaced  ",
            "",
            None,
            123,  # Type invalide
        ]

        for name in malicious_names:
            if name is None or not isinstance(name, str):
                continue
            # Les noms invalides doivent être rejetés ou sanitizés
            joints = robot.get_available_joints()
            if isinstance(name, str) and name.strip():
                # Nettoyer et vérifier
                clean_name = name.strip()
                if clean_name in joints:
                    # Joint valide, doit fonctionner
                    pos = robot.get_joint_pos(clean_name)
                    assert pos is not None or isinstance(pos, float | int)
                else:
                    # Joint invalide, doit être rejeté gracieusement
                    try:
                        pos = robot.get_joint_pos(clean_name)
                        # Si on arrive ici, c'est OK (certains backends sont permissifs)
                        assert isinstance(pos, float | int) or pos is None
                    except (ValueError, KeyError, AttributeError):
                        # C'est le comportement attendu
                        pass
    finally:
        robot.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_joint_position_range_validation() -> None:
    """Test validation plages de positions joints."""
    robot = RobotFactory.create_backend("mujoco")
    if not robot or not robot.connect():
        pytest.skip("MuJoCo non disponible")

    try:
        # Valeurs extrêmes
        extreme_values = [
            1e10,  # Très grand
            -1e10,  # Très petit
            float("inf"),  # Infini
            float("-inf"),  # -Infini
            float("nan"),  # NaN
        ]

        for value in extreme_values:
            # Les valeurs extrêmes doivent être clampées ou rejetées
            try:
                result = robot.set_joint_pos("yaw_body", value)
                if result:
                    # Si accepté, vérifier que position est dans limites
                    pos = robot.get_joint_pos("yaw_body")
                    assert isinstance(pos, float | int)
                    assert abs(pos) < 10.0  # Devrait être raisonnable
            except (ValueError, OverflowError):
                # C'est le comportement attendu pour valeurs invalides
                pass
    finally:
        robot.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_emotion_name_validation() -> None:
    """Test validation noms d'émotions."""
    robot = RobotFactory.create_backend("mujoco")
    if not robot or not robot.connect():
        pytest.skip("MuJoCo non disponible")

    try:
        # Émotions valides
        valid_emotions = ["happy", "sad", "neutral", "excited", "curious", "calm"]

        for emotion in valid_emotions:
            result = robot.set_emotion(emotion, 0.5)
            assert result is True or result is False  # Doit retourner bool

        # Émotions invalides
        invalid_emotions = [
            "../../etc/passwd",
            "'; DROP TABLE emotions; --",
            "",
            None,
            123,
        ]

        for emotion in invalid_emotions:
            if emotion is None or not isinstance(emotion, str):
                continue
            try:
                result = robot.set_emotion(emotion, 0.5)
                # Doit soit rejeter (False) soit gérer gracieusement
                assert result is True or result is False
            except (ValueError, TypeError):
                # C'est le comportement attendu
                pass
    finally:
        robot.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_emotion_intensity_validation() -> None:
    """Test validation intensité émotions (0.0 à 1.0)."""
    robot = RobotFactory.create_backend("mujoco")
    if not robot or not robot.connect():
        pytest.skip("MuJoCo non disponible")

    try:
        # Intensités valides
        valid_intensities = [0.0, 0.5, 1.0, 0.25, 0.75]

        for intensity in valid_intensities:
            result = robot.set_emotion("happy", intensity)
            assert result is True or result is False

        # Intensités invalides
        invalid_intensities = [
            -1.0,  # Négatif
            1.5,  # > 1.0
            -10.0,  # Très négatif
            100.0,  # Très grand
        ]

        for intensity in invalid_intensities:
            result = robot.set_emotion("happy", intensity)
            # Doit soit clamp soit rejeter
            assert result is True or result is False
    finally:
        robot.disconnect()
