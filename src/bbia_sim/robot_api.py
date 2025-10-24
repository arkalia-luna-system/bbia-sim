#!/usr/bin/env python3
"""
RobotAPI - Interface unifiée Sim/Robot
Backend unique pour MuJoCo et Reachy réel
"""

import logging
from abc import ABC, abstractmethod
from typing import Any, Optional

logger = logging.getLogger(__name__)


class RobotAPI(ABC):
    """Interface unifiée pour contrôler le robot (Sim ou Réel)."""

    def __init__(self):
        self.is_connected = False
        self.current_emotion = "neutral"
        self.emotion_intensity = 0.5
        self.joint_limits = {}
        self.forbidden_joints = {
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
        self.safe_amplitude_limit = 0.3  # rad

    @abstractmethod
    def connect(self) -> bool:
        """Connecte au robot/simulateur."""
        pass

    @abstractmethod
    def disconnect(self) -> bool:
        """Déconnecte du robot/simulateur."""
        pass

    @abstractmethod
    def get_available_joints(self) -> list[str]:
        """Retourne la liste des joints disponibles."""
        pass

    @abstractmethod
    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        """Définit la position d'un joint."""
        pass

    @abstractmethod
    def get_joint_pos(self, joint_name: str) -> Optional[float]:
        """Récupère la position actuelle d'un joint."""
        pass

    @abstractmethod
    def step(self) -> bool:
        """Effectue un pas de simulation."""
        pass

    def set_emotion(self, emotion: str, intensity: float = 0.5) -> bool:
        """Définit l'émotion du robot."""
        if not self.is_connected:
            logger.error("Robot non connecté")
            return False

        # Validation de l'émotion
        valid_emotions = {
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
        if emotion not in valid_emotions:
            logger.error(f"Émotion invalide: {emotion}")
            return False

        # Validation de l'intensité
        if not 0.0 <= intensity <= 1.0:
            logger.warning(f"Intensité hors limites, clamp: {intensity}")
            intensity = max(0.0, min(1.0, intensity))

        self.current_emotion = emotion
        self.emotion_intensity = intensity
        logger.info(f"Émotion définie: {emotion} (intensité: {intensity})")
        return True

    def look_at(self, target_x: float, target_y: float, target_z: float = 0.0) -> bool:
        """Fait regarder le robot vers une cible."""
        if not self.is_connected:
            logger.error("Robot non connecté")
            return False

        # Mapping cible → angle de rotation (yaw_body)
        angle = target_x * 0.5  # Amplitude max 0.5 rad

        # Clamp dans les limites sûres
        angle = max(-self.safe_amplitude_limit, min(self.safe_amplitude_limit, angle))

        return self.set_joint_pos("yaw_body", angle)

    def run_behavior(self, behavior_name: str, duration: float = 5.0, **kwargs) -> bool:
        """Exécute un comportement."""
        if not self.is_connected:
            logger.error("Robot non connecté")
            return False

        valid_behaviors = {
            "wake_up",
            "greeting",
            "emotional_response",
            "vision_tracking",
            "conversation",
            "antenna_animation",
            "hide",
        }
        if behavior_name not in valid_behaviors:
            logger.error(f"Comportement invalide: {behavior_name}")
            return False

        logger.info(f"Exécution comportement: {behavior_name} (durée: {duration}s)")

        # Implémentation basique - à étendre selon le backend
        if behavior_name == "wake_up":
            return self._execute_wake_up(duration)
        elif behavior_name == "greeting":
            return self._execute_greeting(duration)
        else:
            logger.warning(f"Comportement {behavior_name} non implémenté")
            return False

    def _execute_wake_up(self, duration: float) -> bool:
        """Exécute le comportement de réveil."""
        import math
        import time

        steps = int(duration * 10)  # 10 Hz
        for step in range(steps):
            t = step / steps
            angle = 0.3 * (1 - math.cos(math.pi * t))  # Mouvement de réveil
            self.set_joint_pos("yaw_body", angle)
            self.step()
            time.sleep(0.1)

        return True

    def _execute_greeting(self, duration: float) -> bool:
        """Exécute le comportement de salutation."""
        import math
        import time

        steps = int(duration * 10)  # 10 Hz
        for step in range(steps):
            t = step / steps
            angle = 0.2 * math.sin(4 * math.pi * t)  # Mouvement de salutation
            self.set_joint_pos("yaw_body", angle)
            self.step()
            time.sleep(0.1)

        return True

    def clamp_joint_position(self, joint_name: str, position: float) -> float:
        """Clamp une position de joint dans les limites sûres."""
        if joint_name in self.forbidden_joints:
            logger.warning(f"Joint interdit: {joint_name}")
            return 0.0

        # Limites générales
        position = max(
            -self.safe_amplitude_limit, min(self.safe_amplitude_limit, position)
        )

        # Limites spécifiques par joint
        if joint_name in self.joint_limits:
            min_limit, max_limit = self.joint_limits[joint_name]
            position = max(min_limit, min(max_limit, position))

        return position

    def get_status(self) -> dict[str, Any]:
        """Retourne le statut du robot."""
        return {
            "connected": self.is_connected,
            "current_emotion": self.current_emotion,
            "emotion_intensity": self.emotion_intensity,
            "available_joints": len(self.get_available_joints()),
            "forbidden_joints": len(self.forbidden_joints),
            "safe_amplitude_limit": self.safe_amplitude_limit,
        }
