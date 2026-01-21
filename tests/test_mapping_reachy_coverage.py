#!/usr/bin/env python3
"""Tests pour le module mapping_reachy.py - Couverture complète."""

import pytest

from bbia_sim.mapping_reachy import (
    FORBIDDEN_JOINTS,
    GLOBAL_SAFETY_LIMIT,
    JOINTS,
    RECOMMENDED_JOINTS,
    JointInfo,
    ReachyMapping,
    get_joint_info,
    validate_joint_position,
)


class TestJointInfo:
    """Tests pour la dataclass JointInfo."""

    def test_joint_info_creation(self) -> None:
        """Test création d'un JointInfo."""
        info = JointInfo(
            name="test_joint",
            min_limit=-1.0,
            max_limit=1.0,
            safe_amplitude=0.3,
            description="Test joint",
            is_active=True,
        )

        assert info.name == "test_joint"
        assert info.min_limit == -1.0
        assert info.max_limit == 1.0
        assert info.safe_amplitude == 0.3
        assert info.description == "Test joint"
        assert info.is_active is True

    def test_joint_info_defaults(self) -> None:
        """Test valeurs par défaut de JointInfo."""
        info = JointInfo(
            name="test",
            min_limit=-0.5,
            max_limit=0.5,
            safe_amplitude=0.2,
        )

        assert info.description == ""
        assert info.is_active is True


class TestReachyMapping:
    """Tests pour ReachyMapping."""

    def test_joints_dict_exists(self) -> None:
        """Test que JOINTS contient les joints attendus."""
        assert "yaw_body" in ReachyMapping.JOINTS
        assert "stewart_1" in ReachyMapping.JOINTS
        assert "stewart_2" in ReachyMapping.JOINTS
        assert "stewart_3" in ReachyMapping.JOINTS
        assert "stewart_4" in ReachyMapping.JOINTS
        assert "stewart_5" in ReachyMapping.JOINTS
        assert "stewart_6" in ReachyMapping.JOINTS

    def test_forbidden_joints_exist(self) -> None:
        """Test que FORBIDDEN_JOINTS contient les joints passifs."""
        assert "passive_1" in ReachyMapping.FORBIDDEN_JOINTS
        assert "passive_2" in ReachyMapping.FORBIDDEN_JOINTS

    def test_recommended_joints_contain_yaw_body(self) -> None:
        """Test que yaw_body est dans RECOMMENDED_JOINTS."""
        assert "yaw_body" in ReachyMapping.RECOMMENDED_JOINTS

    def test_get_joint_info_valid(self) -> None:
        """Test get_joint_info avec joint valide."""
        info = ReachyMapping.get_joint_info("yaw_body")

        assert info.name == "yaw_body"
        assert info.min_limit < 0
        assert info.max_limit > 0

    def test_get_joint_info_forbidden_raises(self) -> None:
        """Test get_joint_info avec joint interdit."""
        with pytest.raises(ValueError, match="Joint interdit"):
            ReachyMapping.get_joint_info("passive_1")

    def test_get_joint_info_unknown_raises(self) -> None:
        """Test get_joint_info avec joint inconnu."""
        with pytest.raises(ValueError, match="Joint inconnu"):
            ReachyMapping.get_joint_info("unknown_joint")

    def test_validate_position_valid(self) -> None:
        """Test validate_position avec position valide."""
        is_valid, clamped = ReachyMapping.validate_position("yaw_body", 0.1)

        assert is_valid is True
        assert clamped == 0.1

    def test_validate_position_clamps_to_safe(self) -> None:
        """Test validate_position clampe aux limites de sécurité."""
        is_valid, clamped = ReachyMapping.validate_position("yaw_body", 1.0)

        assert is_valid is True
        assert clamped <= ReachyMapping.JOINTS["yaw_body"].safe_amplitude

    def test_validate_position_clamps_negative(self) -> None:
        """Test validate_position clampe les valeurs négatives."""
        is_valid, clamped = ReachyMapping.validate_position("yaw_body", -1.0)

        assert is_valid is True
        assert clamped >= -ReachyMapping.JOINTS["yaw_body"].safe_amplitude

    def test_validate_position_forbidden_returns_false(self) -> None:
        """Test validate_position avec joint interdit."""
        is_valid, clamped = ReachyMapping.validate_position("passive_1", 0.1)

        assert is_valid is False
        assert clamped == 0.0

    def test_validate_position_unknown_returns_false(self) -> None:
        """Test validate_position avec joint inconnu."""
        is_valid, clamped = ReachyMapping.validate_position("unknown", 0.1)

        assert is_valid is False
        assert clamped == 0.0

    def test_get_all_joints(self) -> None:
        """Test get_all_joints retourne tous les joints."""
        all_joints = ReachyMapping.get_all_joints()

        assert isinstance(all_joints, set)
        assert "yaw_body" in all_joints
        assert len(all_joints) == len(ReachyMapping.JOINTS)

    def test_get_recommended_joints_returns_copy(self) -> None:
        """Test get_recommended_joints retourne une copie."""
        recommended1 = ReachyMapping.get_recommended_joints()
        recommended2 = ReachyMapping.get_recommended_joints()

        # Modifier l'un ne doit pas affecter l'autre
        recommended1.add("test")
        assert "test" not in recommended2

    def test_get_forbidden_joints_returns_copy(self) -> None:
        """Test get_forbidden_joints retourne une copie."""
        forbidden1 = ReachyMapping.get_forbidden_joints()
        forbidden2 = ReachyMapping.get_forbidden_joints()

        # Modifier l'un ne doit pas affecter l'autre
        forbidden1.add("test")
        assert "test" not in forbidden2

    def test_is_joint_safe_valid(self) -> None:
        """Test is_joint_safe avec joint valide."""
        assert ReachyMapping.is_joint_safe("yaw_body") is True

    def test_is_joint_safe_forbidden(self) -> None:
        """Test is_joint_safe avec joint interdit."""
        assert ReachyMapping.is_joint_safe("passive_1") is False

    def test_is_joint_safe_unknown(self) -> None:
        """Test is_joint_safe avec joint inconnu."""
        assert ReachyMapping.is_joint_safe("unknown") is False


class TestModuleFunctions:
    """Tests pour les fonctions utilitaires du module."""

    def test_validate_joint_position_function(self) -> None:
        """Test la fonction validate_joint_position."""
        is_valid, clamped = validate_joint_position("yaw_body", 0.2)

        assert is_valid is True
        assert abs(clamped - 0.2) < 0.01

    def test_get_joint_info_function(self) -> None:
        """Test la fonction get_joint_info."""
        info = get_joint_info("stewart_1")

        assert info.name == "stewart_1"
        assert isinstance(info, JointInfo)


class TestModuleConstants:
    """Tests pour les constantes exportées."""

    def test_joints_exported(self) -> None:
        """Test que JOINTS est exporté."""
        assert JOINTS is ReachyMapping.JOINTS

    def test_forbidden_joints_exported(self) -> None:
        """Test que FORBIDDEN_JOINTS est exporté."""
        assert FORBIDDEN_JOINTS is ReachyMapping.FORBIDDEN_JOINTS

    def test_recommended_joints_exported(self) -> None:
        """Test que RECOMMENDED_JOINTS est exporté."""
        assert RECOMMENDED_JOINTS is ReachyMapping.RECOMMENDED_JOINTS

    def test_global_safety_limit_exported(self) -> None:
        """Test que GLOBAL_SAFETY_LIMIT est exporté."""
        assert GLOBAL_SAFETY_LIMIT == 0.3
