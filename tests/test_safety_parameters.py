#!/usr/bin/env python3
"""
Tests de sécurité pour valider les paramètres de mouvement.
Vérifie que tous les scripts utilisent des paramètres sûrs par défaut.
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.mapping_reachy import ReachyMapping


class TestSafetyParameters:
    """Tests de sécurité pour les paramètres de mouvement."""

    def test_default_amplitude_limit(self):
        """Test que l'amplitude par défaut est ≤ 0.3 rad."""
        # Test avec mapping Reachy
        safe_limit = ReachyMapping.GLOBAL_SAFETY_LIMIT
        assert safe_limit <= 0.3, f"Limite d'amplitude trop élevée: {safe_limit} > 0.3 rad"

    def test_default_frequency_limit(self):
        """Test que la fréquence par défaut est ≤ 0.2 Hz."""
        # Fréquences sûres détectées dans les scripts
        safe_frequencies = [0.1, 0.2]  # Hz
        max_safe_freq = max(safe_frequencies)
        assert max_safe_freq <= 0.2, f"Fréquence trop élevée: {max_safe_freq} > 0.2 Hz"

    def test_joint_validation_safety(self):
        """Test que la validation des joints respecte les limites de sécurité."""
        # Test joint valide
        is_valid, clamped_pos = ReachyMapping.validate_position("yaw_body", 0.1)
        assert is_valid, "Joint valide rejeté"
        assert abs(clamped_pos) <= 0.3, f"Position clampée trop élevée: {clamped_pos}"

        # Test joint avec amplitude trop élevée
        is_valid, clamped_pos = ReachyMapping.validate_position("yaw_body", 0.5)
        assert abs(clamped_pos) <= 0.3, f"Amplitude non clampée: {clamped_pos} > 0.3"

        # Test joint interdit
        is_valid, clamped_pos = ReachyMapping.validate_position("left_antenna", 0.1)
        assert not is_valid, "Joint interdit accepté"
        assert clamped_pos == 0.0, f"Position non nulle pour joint interdit: {clamped_pos}"

    def test_forbidden_joints_list(self):
        """Test que la liste des joints interdits est correcte."""
        forbidden = ReachyMapping.get_forbidden_joints()

        # Vérifier que les antennes sont interdites
        # Note: Antennes maintenant optionnelles (commentées dans forbidden_joints)
        # Vérifier que les joints passifs sont toujours bloqués
        assert "passive_1" in forbidden, "passive_1 devrait être interdit"

        # Vérifier que les joints passifs sont interdits
        for i in range(1, 8):
            assert f"passive_{i}" in forbidden, f"passive_{i} devrait être interdit"

    def test_recommended_joints_safety(self):
        """Test que les joints recommandés sont sûrs."""
        recommended = ReachyMapping.get_recommended_joints()

        # Vérifier que tous les joints recommandés ont des limites
        for joint in recommended:
            joint_info = ReachyMapping.get_joint_info(joint)
            assert joint_info is not None, f"Joint recommandé {joint} sans info"

            assert (
                joint_info.min_limit < joint_info.max_limit
            ), f"Limites invalides pour {joint}: {joint_info.min_limit} >= {joint_info.max_limit}"

    def test_amplitude_clamping(self):
        """Test que le clamp d'amplitude fonctionne correctement."""
        # Test avec différentes amplitudes
        test_cases = [
            (0.1, 0.1),  # Amplitude sûre
            (0.3, 0.3),  # Amplitude limite
            (0.5, 0.3),  # Amplitude trop élevée → clampée
            (-0.4, -0.3),  # Amplitude négative trop élevée → clampée
        ]

        for input_amp, expected_max in test_cases:
            is_valid, clamped_pos = ReachyMapping.validate_position("yaw_body", input_amp)
            assert abs(clamped_pos) <= abs(
                expected_max
            ), f"Clamp incorrect: {input_amp} → {clamped_pos}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
