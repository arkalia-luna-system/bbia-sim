#!/usr/bin/env python3
"""
Tests unitaires pour validation limites sécurité et PID
Créé suite audit BBIA → Reachy Integration
"""

import pytest

from bbia_sim.mapping_reachy import ReachyMapping


class TestSafetyLimitsPID:
    """Tests validation limites sécurité et gains PID."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_global_safety_limit_value(self):
        """Test que GLOBAL_SAFETY_LIMIT est à 0.3 rad (aligné SDK)."""
        assert ReachyMapping.GLOBAL_SAFETY_LIMIT == 0.3

    @pytest.mark.unit
    @pytest.mark.fast
    def test_validate_position_clamping(self):
        """Test validation et clamping des positions joints."""
        # Test joint valide avec position dans limites
        is_valid, pos = ReachyMapping.validate_position("yaw_body", 0.2)
        assert is_valid is True
        assert 0.0 <= abs(pos) <= 0.3  # Dans limite sécurité

        # Test joint valide avec position hors limite sécurité
        is_valid, pos = ReachyMapping.validate_position("yaw_body", 1.0)
        assert is_valid is True
        assert abs(pos) <= 0.3  # Clampé à limite sécurité

        # Test joint interdit
        is_valid, pos = ReachyMapping.validate_position("left_antenna", 0.5)
        assert is_valid is False
        assert pos == 0.0

    @pytest.mark.unit
    @pytest.mark.fast
    def test_forbidden_joints_protection(self):
        """Test que les joints interdits sont bien protégés."""
        forbidden = ReachyMapping.get_forbidden_joints()
        assert "left_antenna" in forbidden
        assert "right_antenna" in forbidden

        # Tous les joints interdits doivent être rejetés
        for joint in forbidden:
            is_valid, pos = ReachyMapping.validate_position(joint, 0.5)
            assert is_valid is False
            assert pos == 0.0

    @pytest.mark.unit
    @pytest.mark.fast
    def test_joint_limits_hardware_compliance(self):
        """Test que les limites hardware sont conformes SDK."""
        yaw_info = ReachyMapping.get_joint_info("yaw_body")
        # Limite yaw_body: rotation complète ~±2.79 rad (SDK officiel)
        assert yaw_info.min_limit < -2.7
        assert yaw_info.max_limit > 2.7

        # Limite sécurité doit être plus restrictive
        assert abs(yaw_info.safe_amplitude) <= 0.3

    @pytest.mark.unit
    @pytest.mark.fast
    def test_stewart_joints_recommendation(self):
        """Test que les joints stewart ne sont pas recommandés pour contrôle direct."""
        recommended = ReachyMapping.get_recommended_joints()
        assert "yaw_body" in recommended
        # Les joints stewart ne doivent PAS être dans recommandés
        for i in range(1, 7):
            assert f"stewart_{i}" not in recommended
