"""Tests pour la validation des articulations."""

import pytest

from src.bbia_sim.sim.joints import (
    REACHY_JOINTS,
    get_available_joints,
    get_joint_range,
    get_joint_spec,
    validate_joint_position,
    validate_joint_velocity,
)


class TestJointValidation:
    """Tests pour la validation des articulations."""

    def test_joint_specs_complete(self):
        """Test que toutes les spécifications d'articulations sont complètes."""
        expected_joints = [
            "neck_yaw",
            "right_shoulder_pitch",
            "right_elbow_pitch",
            "right_gripper_joint",
            "left_shoulder_pitch",
            "left_elbow_pitch",
            "left_gripper_joint",
        ]

        available_joints = get_available_joints()

        for joint in expected_joints:
            assert joint in available_joints, f"Articulation {joint} manquante"
            assert joint in REACHY_JOINTS, f"Spécification {joint} manquante"

            spec = get_joint_spec(joint)
            assert spec.min_angle < spec.max_angle, f"Plage invalide pour {joint}"
            assert spec.max_velocity > 0, f"Vitesse max invalide pour {joint}"
            assert spec.gear_ratio > 0, f"Ratio invalide pour {joint}"

        print(f"✅ {len(available_joints)} articulations validées")

    def test_joint_position_validation(self):
        """Test la validation des positions d'articulations."""
        # Test positions valides
        assert validate_joint_position("neck_yaw", 0.0) is True
        assert validate_joint_position("neck_yaw", 1.0) is True
        assert validate_joint_position("neck_yaw", -1.0) is True

        # Test positions invalides
        assert validate_joint_position("neck_yaw", 2.0) is False  # Trop grand
        assert validate_joint_position("neck_yaw", -2.0) is False  # Trop petit

        # Test gripper (plage plus petite)
        assert validate_joint_position("right_gripper_joint", 0.0) is True
        assert validate_joint_position("right_gripper_joint", 0.4) is True
        assert (
            validate_joint_position("right_gripper_joint", 0.6) is False
        )  # Trop grand
        assert (
            validate_joint_position("right_gripper_joint", -0.6) is False
        )  # Trop petit

        print("✅ Validation des positions fonctionne")

    def test_joint_velocity_validation(self):
        """Test la validation des vitesses d'articulations."""
        # Test vitesses valides
        assert validate_joint_velocity("neck_yaw", 1.0) is True
        assert validate_joint_velocity("neck_yaw", -1.0) is True
        assert validate_joint_velocity("neck_yaw", 0.0) is True

        # Test vitesses invalides
        assert validate_joint_velocity("neck_yaw", 3.0) is False  # Trop rapide
        assert validate_joint_velocity("neck_yaw", -3.0) is False  # Trop rapide

        # Test gripper (vitesse max plus faible)
        assert validate_joint_velocity("right_gripper_joint", 0.5) is True
        assert (
            validate_joint_velocity("right_gripper_joint", 1.5) is False
        )  # Trop rapide

        print("✅ Validation des vitesses fonctionne")

    def test_joint_range_retrieval(self):
        """Test la récupération des plages de mouvement."""
        # Test articulation normale
        min_angle, max_angle = get_joint_range("neck_yaw")
        assert min_angle == -1.57
        assert max_angle == 1.57

        # Test gripper (plage plus petite)
        min_angle, max_angle = get_joint_range("right_gripper_joint")
        assert min_angle == -0.5
        assert max_angle == 0.5

        print("✅ Récupération des plages fonctionne")

    def test_unknown_joint_error(self):
        """Test que les articulations inconnues lèvent une erreur."""
        with pytest.raises(ValueError, match="Articulation inconnue"):
            get_joint_spec("unknown_joint")

        with pytest.raises(ValueError, match="Articulation inconnue"):
            validate_joint_position("unknown_joint", 0.0)

        with pytest.raises(ValueError, match="Articulation inconnue"):
            validate_joint_velocity("unknown_joint", 0.0)

        with pytest.raises(ValueError, match="Articulation inconnue"):
            get_joint_range("unknown_joint")

        print("✅ Gestion des erreurs fonctionne")

    def test_joint_spec_consistency(self):
        """Test la cohérence des spécifications d'articulations."""
        for joint_name, spec in REACHY_JOINTS.items():
            # Vérifier la cohérence des noms
            assert spec.name == joint_name, f"Nom incohérent pour {joint_name}"

            # Vérifier les plages raisonnables
            assert (
                -3.14 <= spec.min_angle <= 3.14
            ), f"Angle min invalide pour {joint_name}"
            assert (
                -3.14 <= spec.max_angle <= 3.14
            ), f"Angle max invalide pour {joint_name}"

            # Vérifier les vitesses raisonnables
            assert (
                0.1 <= spec.max_velocity <= 10.0
            ), f"Vitesse max invalide pour {joint_name}"

            # Vérifier les ratios raisonnables
            assert 1 <= spec.gear_ratio <= 1000, f"Ratio invalide pour {joint_name}"

        print("✅ Cohérence des spécifications validée")
