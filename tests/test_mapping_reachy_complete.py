#!/usr/bin/env python3
"""
Tests exhaustifs pour mapping_reachy.py
Validation complète de la conformité avec le SDK officiel Reachy Mini
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.mapping_reachy import (
    FORBIDDEN_JOINTS,
    GLOBAL_SAFETY_LIMIT,
    JOINTS,
    RECOMMENDED_JOINTS,
    ReachyMapping,
    get_joint_info,
    validate_joint_position,
)


class TestReachyMappingComplete:
    """Tests exhaustifs pour ReachyMapping."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.mapping = ReachyMapping()

    def test_01_joints_structure(self):
        """Test 1: Structure de base des joints."""
        assert isinstance(JOINTS, dict)
        assert len(JOINTS) == 7  # 6 stewart + 1 yaw_body
        assert "yaw_body" in JOINTS
        for i in range(1, 7):
            assert f"stewart_{i}" in JOINTS

    def test_02_joint_info_structure(self):
        """Test 2: Structure JointInfo correcte."""
        for joint_name, joint_info in JOINTS.items():
            assert joint_info.name == joint_name
            assert isinstance(joint_info.min_limit, float)
            assert isinstance(joint_info.max_limit, float)
            assert isinstance(joint_info.safe_amplitude, float)
            assert isinstance(joint_info.description, str)
            assert isinstance(joint_info.is_active, bool)
            assert joint_info.min_limit < joint_info.max_limit
            assert joint_info.safe_amplitude > 0
            assert joint_info.safe_amplitude <= 0.3

    def test_03_joint_limits_official_sdk(self):
        """Test 3: Limites exactes du SDK officiel (comparaison avec backend)."""
        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()

        for joint_name in ["yaw_body", "stewart_1", "stewart_2", "stewart_3"]:
            if joint_name in JOINTS and joint_name in backend.joint_limits:
                mapping_info = JOINTS[joint_name]
                backend_min, backend_max = backend.joint_limits[joint_name]

                # Vérifier cohérence exacte (tolérance 1e-10 pour flottants)
                assert abs(mapping_info.min_limit - backend_min) < 1e-10, (
                    f"Limite min incohérente pour {joint_name}: "
                    f"mapping={mapping_info.min_limit}, backend={backend_min}"
                )
                assert abs(mapping_info.max_limit - backend_max) < 1e-10, (
                    f"Limite max incohérente pour {joint_name}: "
                    f"mapping={mapping_info.max_limit}, backend={backend_max}"
                )

    def test_04_stewart_joints_ik_warning(self):
        """Test 4: Vérifier que tous les stewart joints ont l'avertissement IK."""
        for i in range(1, 7):
            joint_name = f"stewart_{i}"
            joint_info = JOINTS[joint_name]
            assert (
                "goto_target" in joint_info.description or "IK" in joint_info.description
            ), f"Description stewart_{i} doit mentionner IK/goto_target"

    def test_05_forbidden_joints_complete(self):
        """Test 5: Joints interdits complets."""
        assert isinstance(FORBIDDEN_JOINTS, set)
        # Note: left_antenna et right_antenna sont maintenant optionnelles (commentées)
        # Elles sont animables avec limites sûres (-0.3 à 0.3 rad)
        expected_forbidden = {
            # "left_antenna",   # Optionnel: décommenter pour bloquer
            # "right_antenna",  # Optionnel: décommenter pour bloquer
            "passive_1",
            "passive_2",
            "passive_3",
            "passive_4",
            "passive_5",
            "passive_6",
            "passive_7",
        }
        assert (
            FORBIDDEN_JOINTS == expected_forbidden
        ), f"FORBIDDEN_JOINTS ne correspond pas: {FORBIDDEN_JOINTS} vs {expected_forbidden}"

    def test_06_recommended_joints_correct(self):
        """Test 6: Joints recommandés (seulement yaw_body car stewart nécessitent IK)."""
        assert RECOMMENDED_JOINTS == {"yaw_body"}
        assert "stewart_1" not in RECOMMENDED_JOINTS  # Nécessite IK

    def test_07_get_all_joints(self):
        """Test 7: Méthode get_all_joints()."""
        all_joints = self.mapping.get_all_joints()
        assert isinstance(all_joints, set)
        assert all_joints == set(JOINTS.keys())
        assert len(all_joints) == 7

    def test_08_get_recommended_joints(self):
        """Test 8: Méthode get_recommended_joints()."""
        recommended = self.mapping.get_recommended_joints()
        assert isinstance(recommended, set)
        assert recommended == RECOMMENDED_JOINTS
        assert recommended == {"yaw_body"}

    def test_09_get_forbidden_joints(self):
        """Test 9: Méthode get_forbidden_joints()."""
        forbidden = self.mapping.get_forbidden_joints()
        assert isinstance(forbidden, set)
        assert forbidden == FORBIDDEN_JOINTS

    def test_10_get_joint_info_valid(self):
        """Test 10: get_joint_info() avec joint valide."""
        joint_info = self.mapping.get_joint_info("yaw_body")
        assert joint_info.name == "yaw_body"
        assert joint_info.min_limit == -2.792526803190975
        assert joint_info.max_limit == 2.792526803190879

    def test_11_get_joint_info_forbidden(self):
        """Test 11: get_joint_info() avec joint interdit."""
        # Utiliser passive_1 qui est réellement interdit
        # (left_antenna n'est plus interdit par défaut - optionnel)
        with pytest.raises(ValueError, match="Joint interdit"):
            ReachyMapping.get_joint_info("passive_1")

    def test_12_get_joint_info_unknown(self):
        """Test 12: get_joint_info() avec joint inconnu."""
        with pytest.raises(ValueError, match="Joint inconnu"):
            self.mapping.get_joint_info("unknown_joint")

    def test_13_is_joint_safe_valid(self):
        """Test 13: is_joint_safe() avec joint valide."""
        assert self.mapping.is_joint_safe("yaw_body") is True
        assert self.mapping.is_joint_safe("stewart_1") is True

    def test_14_is_joint_safe_forbidden(self):
        """Test 14: is_joint_safe() avec joint interdit."""
        # Note: Antennes maintenant optionnelles (commentées dans FORBIDDEN_JOINTS)
        # Si commentées, is_joint_safe peut retourner True (selon mapping)
        # Vérifier que les joints passifs sont toujours False
        assert self.mapping.is_joint_safe("passive_1") is False

    def test_15_is_joint_safe_unknown(self):
        """Test 15: is_joint_safe() avec joint inconnu."""
        assert self.mapping.is_joint_safe("unknown_joint") is False

    def test_16_validate_position_within_limits(self):
        """Test 16: validate_position() position dans limites hardware."""
        # Position dans limites hardware mais > safe_amplitude
        is_valid, clamped = self.mapping.validate_position("yaw_body", 0.2)
        assert is_valid is True
        assert clamped == 0.2  # Dans safe_amplitude (0.3)

    def test_17_validate_position_exceeds_safe_amplitude(self):
        """Test 17: validate_position() position > safe_amplitude (clampée)."""
        # Position > safe_amplitude (0.3)
        is_valid, clamped = self.mapping.validate_position("yaw_body", 0.5)
        assert is_valid is True
        assert clamped == 0.3  # Clampée à safe_amplitude

    def test_18_validate_position_negative_safe_amplitude(self):
        """Test 18: validate_position() position négative > safe_amplitude."""
        # Position négative < -safe_amplitude
        is_valid, clamped = self.mapping.validate_position("yaw_body", -0.5)
        assert is_valid is True
        assert clamped == -0.3  # Clampée à -safe_amplitude

    def test_19_validate_position_stewart_clamp(self):
        """Test 19: validate_position() avec stewart joint (safe_amplitude=0.2)."""
        # Position > safe_amplitude (0.2 pour stewart)
        is_valid, clamped = self.mapping.validate_position("stewart_1", 0.5)
        assert is_valid is True
        assert clamped == 0.2  # Clampée à safe_amplitude stewart

    def test_20_validate_position_hardware_limit_vs_safe(self):
        """Test 20: validate_position() priorité hardware puis safe_amplitude."""
        # Position dans limites hardware mais > safe_amplitude
        # Le clamp doit respecter les deux: d'abord hardware, puis safe
        is_valid, clamped = self.mapping.validate_position("yaw_body", 1.5)
        assert is_valid is True
        # D'abord clamp hardware: 1.5 est dans [-2.79, 2.79] → reste 1.5
        # Puis clamp safe: 1.5 > 0.3 → clamp à 0.3
        assert clamped == 0.3

    def test_21_validate_position_forbidden_joint(self):
        """Test 21: validate_position() avec joint interdit."""
        is_valid, clamped = self.mapping.validate_position("left_antenna", 0.1)
        assert is_valid is False
        assert clamped == 0.0

    def test_22_validate_position_unknown_joint(self):
        """Test 22: validate_position() avec joint inconnu."""
        is_valid, clamped = self.mapping.validate_position("unknown_joint", 0.1)
        assert is_valid is False
        assert clamped == 0.0

    def test_23_validate_position_edge_cases(self):
        """Test 23: validate_position() cas limites."""
        # Position exactement à safe_amplitude
        is_valid, clamped = self.mapping.validate_position("yaw_body", 0.3)
        assert is_valid is True
        assert clamped == 0.3

        # Position exactement à -safe_amplitude
        is_valid, clamped = self.mapping.validate_position("yaw_body", -0.3)
        assert is_valid is True
        assert clamped == -0.3

        # Position zéro
        is_valid, clamped = self.mapping.validate_position("yaw_body", 0.0)
        assert is_valid is True
        assert clamped == 0.0

    def test_24_utility_functions(self):
        """Test 24: Fonctions utilitaires (validate_joint_position, get_joint_info)."""
        # Test validate_joint_position
        is_valid, clamped = validate_joint_position("yaw_body", 0.2)
        assert is_valid is True
        assert clamped == 0.2

        # Test get_joint_info
        joint_info = get_joint_info("yaw_body")
        assert joint_info.name == "yaw_body"

    def test_25_global_safety_limit(self):
        """Test 25: Limite de sécurité globale."""
        assert GLOBAL_SAFETY_LIMIT == 0.3
        assert isinstance(GLOBAL_SAFETY_LIMIT, float)

    def test_26_constants_exports(self):
        """Test 26: Export des constantes pour compatibilité."""
        from bbia_sim.mapping_reachy import (
            FORBIDDEN_JOINTS,
            GLOBAL_SAFETY_LIMIT,
            JOINTS,
            RECOMMENDED_JOINTS,
        )

        assert JOINTS == ReachyMapping.JOINTS
        assert FORBIDDEN_JOINTS == ReachyMapping.FORBIDDEN_JOINTS
        assert RECOMMENDED_JOINTS == ReachyMapping.RECOMMENDED_JOINTS
        assert GLOBAL_SAFETY_LIMIT == ReachyMapping.GLOBAL_SAFETY_LIMIT

    def test_27_comprehensive_clamp_behavior(self):
        """Test 27: Comportement complet du clamp (double étapes)."""
        # Cas 1: Position < -safe_amplitude mais dans limites hardware
        is_valid, clamped = self.mapping.validate_position("yaw_body", -0.5)
        assert is_valid is True
        assert clamped == -0.3  # Clampée à -safe_amplitude

        # Cas 2: Position > hardware max (impossible pour yaw_body qui a grandes limites)
        # Testons avec un stewart qui a limites plus petites
        is_valid, clamped = self.mapping.validate_position("stewart_4", -1.5)  # < min_limit (-1.40)
        assert is_valid is True
        # D'abord clamp hardware: -1.5 → -1.396 (min_limit)
        # Puis clamp safe: -1.396 < -0.2 → clamp à -0.2
        assert clamped == -0.2

    def test_28_class_vs_instance_methods(self):
        """Test 28: Méthodes de classe fonctionnent sans instance."""
        # Toutes les méthodes sont @classmethod, donc fonctionnent sans instance
        joints = ReachyMapping.get_all_joints()
        assert isinstance(joints, set)
        assert len(joints) == 7

        # Instance fonctionne aussi
        mapping = ReachyMapping()
        joints2 = mapping.get_all_joints()
        assert joints == joints2


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
