#!/usr/bin/env python3
"""
Tests unitaires pour GlobalConfig
Validation de la configuration globale BBIA
"""

import importlib
import os

import pytest

from bbia_sim.global_config import GlobalConfig


class TestGlobalConfig:
    """Tests pour la configuration globale."""

    def setup_method(self):
        """Configuration avant chaque test."""
        # Réinitialiser le seed
        GlobalConfig.initialize_seed()

    def test_global_seed_default(self):
        """Test seed global par défaut."""
        assert GlobalConfig.GLOBAL_SEED == 42

    def test_global_seed_from_env(self):
        """Test seed global depuis variable d'environnement."""
        os.environ["BBIA_SEED"] = "123"
        # Recharger le module pour appliquer le nouveau seed
        import bbia_sim.global_config

        importlib.reload(bbia_sim.global_config)

        # Le seed devrait être 123
        assert bbia_sim.global_config.GlobalConfig.GLOBAL_SEED == 123

        # Restaurer
        os.environ["BBIA_SEED"] = "42"
        importlib.reload(bbia_sim.global_config)

    def test_safe_amplitude_limit(self):
        """Test limite amplitude sûre."""
        assert GlobalConfig.SAFE_AMPLITUDE_LIMIT == 0.3

    def test_max_step_time(self):
        """Test temps maximum pour un step."""
        assert GlobalConfig.MAX_STEP_TIME == 0.1

    def test_max_duration(self):
        """Test durée maximale."""
        assert GlobalConfig.MAX_DURATION == 60

    def test_forbidden_joints(self):
        """Test joints interdits."""
        assert isinstance(GlobalConfig.FORBIDDEN_JOINTS, set)
        # Note: Antennes maintenant optionnelles (commentées dans forbidden_joints)
        # Elles peuvent être dans forbidden_joints si décommentées, sinon animables
        assert "passive_1" in GlobalConfig.FORBIDDEN_JOINTS

    def test_safe_joints(self):
        """Test joints sûrs."""
        assert isinstance(GlobalConfig.SAFE_JOINTS, set)
        assert "yaw_body" in GlobalConfig.SAFE_JOINTS

    def test_valid_emotions(self):
        """Test émotions valides."""
        assert isinstance(GlobalConfig.VALID_EMOTIONS, set)
        assert "happy" in GlobalConfig.VALID_EMOTIONS
        assert "sad" in GlobalConfig.VALID_EMOTIONS
        assert "neutral" in GlobalConfig.VALID_EMOTIONS
        assert "curious" in GlobalConfig.VALID_EMOTIONS

    def test_valid_behaviors(self):
        """Test comportements valides."""
        assert isinstance(GlobalConfig.VALID_BEHAVIORS, set)
        assert "wake_up" in GlobalConfig.VALID_BEHAVIORS
        assert "greeting" in GlobalConfig.VALID_BEHAVIORS
        assert "emotional_response" in GlobalConfig.VALID_BEHAVIORS

    def test_initialize_seed(self):
        """Test initialisation seed."""
        GlobalConfig.initialize_seed()
        # Vérifier que le seed est bien initialisé

        # Le seed devrait être 42
        assert GlobalConfig.GLOBAL_SEED == 42

    def test_validate_joint_allowed(self):
        """Test validation joint autorisé."""
        result = GlobalConfig.validate_joint("yaw_body")
        assert result is True

    def test_validate_joint_forbidden(self):
        """Test validation joint interdit."""
        # Utiliser passive_1 qui est réellement dans FORBIDDEN_JOINTS
        # (left_antenna n'est plus interdit par défaut)
        result = GlobalConfig.validate_joint("passive_1")
        assert (
            result is False
        ), f"passive_1 devrait être interdit (FORBIDDEN_JOINTS: {GlobalConfig.FORBIDDEN_JOINTS})"

    def test_clamp_amplitude_within_limits(self):
        """Test clamp amplitude dans les limites."""
        result = GlobalConfig.clamp_amplitude(0.1)
        assert result == 0.1

    def test_clamp_amplitude_above_limit(self):
        """Test clamp amplitude au-dessus de la limite."""
        result = GlobalConfig.clamp_amplitude(0.5)
        assert result == 0.3  # Limité à SAFE_AMPLITUDE_LIMIT

    def test_clamp_amplitude_below_limit(self):
        """Test clamp amplitude en-dessous de la limite."""
        result = GlobalConfig.clamp_amplitude(-0.5)
        assert result == -0.3  # Limité à -SAFE_AMPLITUDE_LIMIT

    def test_validate_emotion_valid(self):
        """Test validation émotion valide."""
        result = GlobalConfig.validate_emotion("happy")
        assert result is True

    def test_validate_emotion_invalid(self):
        """Test validation émotion invalide."""
        result = GlobalConfig.validate_emotion("invalid_emotion")
        assert result is False

    def test_validate_behavior_valid(self):
        """Test validation comportement valide."""
        result = GlobalConfig.validate_behavior("wake_up")
        assert result is True

    def test_validate_behavior_invalid(self):
        """Test validation comportement invalide."""
        result = GlobalConfig.validate_behavior("invalid_behavior")
        assert result is False

    def test_get_safe_joint(self):
        """Test récupération joint sûr."""
        result = GlobalConfig.get_safe_joint()
        assert result == "yaw_body"

    def test_get_config_summary(self):
        """Test récupération résumé configuration."""
        summary = GlobalConfig.get_config_summary()

        assert isinstance(summary, dict)
        assert "global_seed" in summary
        assert "safe_amplitude_limit" in summary
        assert "max_step_time" in summary
        assert "max_duration" in summary
        assert "forbidden_joints_count" in summary
        assert "safe_joints_count" in summary
        assert "valid_emotions_count" in summary
        assert "valid_behaviors_count" in summary

        assert summary["global_seed"] == 42
        assert summary["safe_amplitude_limit"] == 0.3
        assert summary["max_step_time"] == 0.1
        assert summary["max_duration"] == 60
        assert summary["forbidden_joints_count"] > 0
        assert summary["safe_joints_count"] > 0
        assert summary["valid_emotions_count"] > 0
        assert summary["valid_behaviors_count"] > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
