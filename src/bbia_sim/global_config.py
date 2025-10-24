#!/usr/bin/env python3
"""
Configuration globale pour déterminisme et sécurité
SEED global, limites sûres, failsafe
"""

import logging
import os
import random
from typing import Any

logger = logging.getLogger(__name__)


class GlobalConfig:
    """Configuration globale pour BBIA."""

    # SEED global pour déterminisme
    GLOBAL_SEED = int(os.environ.get("BBIA_SEED", 42))

    # Limites sûres centralisées
    SAFE_AMPLITUDE_LIMIT = 0.3  # rad
    MAX_STEP_TIME = 0.1  # s
    MAX_DURATION = 60  # s

    # Joints interdits
    FORBIDDEN_JOINTS = {
        "left_antenna",
        "right_antenna",
        "passive_1",
        "passive_2",
        "passive_3",
        "passive_4",
        "passive_5",
        "passive_6",
        "passive_7",
    }

    # Joints sûrs recommandés
    SAFE_JOINTS = {"yaw_body"}

    # Émotions valides
    VALID_EMOTIONS = {
        "neutral",
        "happy",
        "sad",
        "angry",
        "surprised",
        "confused",
        "determined",
        "nostalgic",
        "proud",
    }

    # Comportements valides
    VALID_BEHAVIORS = {
        "wake_up",
        "greeting",
        "emotional_response",
        "vision_tracking",
        "conversation",
        "antenna_animation",
        "hide",
    }

    @classmethod
    def initialize_seed(cls):
        """Initialise le seed global."""
        random.seed(cls.GLOBAL_SEED)
        logger.info(f"Seed global initialisé: {cls.GLOBAL_SEED}")

    @classmethod
    def validate_joint(cls, joint_name: str) -> bool:
        """Valide qu'un joint est autorisé."""
        if joint_name in cls.FORBIDDEN_JOINTS:
            logger.warning(f"Joint interdit: {joint_name}")
            return False
        return True

    @classmethod
    def clamp_amplitude(cls, amplitude: float) -> float:
        """Clamp une amplitude dans les limites sûres."""
        clamped = max(
            -cls.SAFE_AMPLITUDE_LIMIT, min(cls.SAFE_AMPLITUDE_LIMIT, amplitude)
        )
        if clamped != amplitude:
            logger.warning(f"Amplitude clampée: {amplitude} → {clamped}")
        return clamped

    @classmethod
    def validate_emotion(cls, emotion: str) -> bool:
        """Valide qu'une émotion est supportée."""
        if emotion not in cls.VALID_EMOTIONS:
            logger.error(f"Émotion invalide: {emotion}")
            return False
        return True

    @classmethod
    def validate_behavior(cls, behavior: str) -> bool:
        """Valide qu'un comportement est supporté."""
        if behavior not in cls.VALID_BEHAVIORS:
            logger.error(f"Comportement invalide: {behavior}")
            return False
        return True

    @classmethod
    def get_safe_joint(cls) -> str:
        """Retourne un joint sûr par défaut."""
        return "yaw_body"

    @classmethod
    def get_config_summary(cls) -> dict[str, Any]:
        """Retourne un résumé de la configuration."""
        return {
            "global_seed": cls.GLOBAL_SEED,
            "safe_amplitude_limit": cls.SAFE_AMPLITUDE_LIMIT,
            "max_step_time": cls.MAX_STEP_TIME,
            "max_duration": cls.MAX_DURATION,
            "forbidden_joints_count": len(cls.FORBIDDEN_JOINTS),
            "safe_joints_count": len(cls.SAFE_JOINTS),
            "valid_emotions_count": len(cls.VALID_EMOTIONS),
            "valid_behaviors_count": len(cls.VALID_BEHAVIORS),
        }


# Initialisation automatique
GlobalConfig.initialize_seed()
