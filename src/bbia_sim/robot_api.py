#!/usr/bin/env python3
"""RobotAPI - Interface unifiée Sim/Robot
Backend unique pour MuJoCo et Reachy réel
"""

import logging
from abc import ABC, abstractmethod
from typing import Any

from .utils.types import RobotStatus

logger = logging.getLogger(__name__)


class RobotAPI(ABC):
    """Interface unifiée pour contrôler le robot (Sim ou Réel)."""

    def __init__(self) -> None:
        self.is_connected = False
        self.current_emotion = "neutral"
        self.emotion_intensity = 0.5
        self.joint_limits: dict[str, tuple[float, float]] = {}
        # Utiliser GlobalConfig pour cohérence (antennes maintenant animables)
        from .global_config import GlobalConfig

        self.forbidden_joints = set(GlobalConfig.FORBIDDEN_JOINTS)
        self.safe_amplitude_limit = 0.3  # rad

    @abstractmethod
    def connect(self) -> bool:
        """Connecte au robot/simulateur."""

    @abstractmethod
    def disconnect(self) -> bool:
        """Déconnecte du robot/simulateur."""

    @abstractmethod
    def get_available_joints(self) -> list[str]:
        """Retourne la liste des joints disponibles."""

    @abstractmethod
    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        """Définit la position d'un joint."""

    def _validate_joint_pos(
        self,
        joint_name: str,
        position: float,
    ) -> tuple[bool, float]:
        """Valide et clamp la position d'un joint."""
        # Vérifier si le joint est interdit
        if joint_name in self.forbidden_joints:
            logger.error("Joint interdit: %s", joint_name)
            return False, position

        # Clamp l'amplitude pour la sécurité
        clamped_position = max(
            -self.safe_amplitude_limit,
            min(self.safe_amplitude_limit, position),
        )

        if clamped_position != position:
            logger.warning(
                f"Position clampée: {position:.3f} → {clamped_position:.3f} rad",
            )

        return True, clamped_position

    @abstractmethod
    def get_joint_pos(self, joint_name: str) -> float | None:
        """Récupère la position actuelle d'un joint."""

    @abstractmethod
    def step(self) -> bool:
        """Effectue un pas de simulation."""

    def goto_target(
        self,
        head: Any = None,
        antennas: Any = None,
        duration: float = 0.5,
        method: str = "minjerk",
        body_yaw: float | None = None,
    ) -> None:
        """Va vers une cible spécifique avec technique d'interpolation.

        NOTE: Cette méthode est optionnelle dans RobotAPI. Les backends avancés
        (comme ReachyMiniBackend) l'implémentent avec IK complète. Les backends
        simples (comme MuJoCoBackend) peuvent avoir une implémentation simplifiée.

        Args:
            head: Matrice 4x4 ou HeadPose représentant la pose de la tête (ou None)
            antennas: Angles des antennes en radians [right, left] (ou None)
            duration: Durée du mouvement en secondes (doit être > 0)
            method: Technique d'interpolation ("minjerk", "linear", etc.)
            body_yaw: Angle yaw du corps en radians (None = garder position actuelle)

        Raises:
            NotImplementedError: Si le backend ne supporte pas goto_target
            ValueError: Si duration <= 0
        """
        raise NotImplementedError(
            f"{self.__class__.__name__} ne supporte pas goto_target(). "
            "Utilisez set_joint_pos() pour contrôler les joints individuellement."
        )

    @abstractmethod
    def emergency_stop(self) -> bool:
        """Arrêt d'urgence hardware.

        Arrête immédiatement tous les moteurs et désactive le contrôle.
        Conforme aux specs sécurité robotique.
        """

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
            "excited",  # Ajouté pour compatibilité démos
            "curious",  # Ajouté pour compatibilité démos
            "calm",  # Ajouté pour compatibilité démos
        }
        if emotion not in valid_emotions:
            logger.error("Émotion invalide: %s", emotion)
            return False

        # Validation de l'intensité
        if not 0.0 <= intensity <= 1.0:
            logger.warning("Intensité hors limites, clamp: %s", intensity)
            intensity = max(0.0, min(1.0, intensity))

        self.current_emotion = emotion
        self.emotion_intensity = intensity
        logger.info("Émotion définie: %s (intensité: %s)", emotion, intensity)
        return True

    def look_at(self, target_x: float, target_y: float, target_z: float = 0.0) -> bool:
        """Fait regarder le robot vers une cible (méthode générique).

        NOTE EXPERT: Cette méthode est un wrapper générique. Les backends avancés
        (comme ReachyMiniBackend) implémentent look_at_world() qui est plus précise
        avec calcul IK complet pour la tête.

        Args:
            target_x: Position X de la cible (mètres)
            target_y: Position Y de la cible (mètres)
            target_z: Position Z de la cible (mètres, défaut: 0.0)

        Returns:
            True si commande envoyée, False sinon

        """
        if not self.is_connected:
            logger.error("Robot non connecté")
            return False

        # Vérifier si le backend a look_at_world (plus précis)
        if hasattr(self, "look_at_world"):
            try:
                # Utiliser la méthode SDK avancée si disponible
                self.look_at_world(
                    target_x,
                    target_y,
                    target_z,
                    duration=1.0,
                    perform_movement=True,
                )
                return True
            except Exception as e:
                logger.warning("Erreur look_at_world, fallback générique: %s", e)

        # Fallback: Mapping cible → angle de rotation (yaw_body) simplifié
        # CORRECTION EXPERTE: Validation coordonnées avant utilisation
        # Limites recommandées SDK: -2.0 ≤ x,y ≤ 2.0 mètres, 0.0 ≤ z ≤ 1.5 mètres
        if (
            abs(target_x) > 2.0
            or abs(target_y) > 2.0
            or target_z < 0.0
            or target_z > 1.5
        ):
            logger.warning(
                f"Coordonnées ({target_x}, {target_y}, {target_z}) hors limites recommandées "
                "SDK (-2.0 ≤ x,y ≤ 2.0, 0.0 ≤ z ≤ 1.5). Clampage appliqué.",
            )
            target_x = max(-2.0, min(2.0, target_x))
            target_y = max(-2.0, min(2.0, target_y))
            target_z = max(0.0, min(1.5, target_z))

        # Mapping simplifié: rotation corps vers cible
        angle = target_x * 0.5  # Amplitude max 0.5 rad

        # Clamp dans les limites sûres
        angle = max(-self.safe_amplitude_limit, min(self.safe_amplitude_limit, angle))

        return self.set_joint_pos("yaw_body", angle)

    def run_behavior(
        self,
        behavior_name: str,
        duration: float = 5.0,
        **kwargs: Any,
    ) -> bool:
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
            "nod",  # Ajouté pour compatibilité démos
            "goto_sleep",  # Ajouté pour compatibilité démos
            "exploration",  # Comportement d'exploration
            "interaction",  # Comportement d'interaction
            "demo",  # Comportement de démonstration
        }
        if behavior_name not in valid_behaviors:
            logger.error("Comportement invalide: %s", behavior_name)
            return False

        logger.info("Exécution comportement: %s (durée: %ss)", behavior_name, duration)

        # Implémentation basique - à étendre selon le backend
        if behavior_name == "wake_up":
            return self._execute_wake_up(duration)
        if behavior_name == "greeting":
            return self._execute_greeting(duration)
        if behavior_name == "nod":
            return self._execute_nod(duration)
        if behavior_name == "goto_sleep":
            return self._execute_goto_sleep(duration)
        if behavior_name == "exploration":
            return self._execute_exploration(duration)
        if behavior_name == "interaction":
            return self._execute_interaction(duration)
        if behavior_name == "demo":
            return self._execute_demo(duration)
        logger.warning("Comportement %s non implémenté", behavior_name)
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

    def _execute_nod(self, duration: float) -> bool:
        """Exécute un hochement de tête."""
        import math
        import time

        steps = int(duration * 10)  # 10 Hz
        for step in range(steps):
            t = step / steps
            # Mouvement de hochement (pitch)
            pitch = 0.15 * math.sin(2 * math.pi * t)
            if "pitch_head" in self.get_available_joints():
                self.set_joint_pos("pitch_head", pitch)
            self.step()
            time.sleep(0.1)

        return True

    def _execute_goto_sleep(self, duration: float) -> bool:
        """Exécute le comportement d'endormissement."""
        import time

        steps = int(duration * 10)  # 10 Hz
        for step in range(steps):
            t = step / steps
            # Mouvement lent vers position neutre
            angle = 0.2 * (1 - t)  # Retour progressif à 0
            self.set_joint_pos("yaw_body", angle)
            self.step()
            time.sleep(0.1)

        return True

    def _execute_exploration(self, duration: float) -> bool:
        """Exécute le comportement d'exploration."""
        import math
        import time

        steps = int(duration * 10)  # 10 Hz
        for step in range(steps):
            t = step / steps
            # Mouvement d'exploration (balayage)
            angle = 0.3 * math.sin(2 * math.pi * t / 2)  # Balayage lent
            self.set_joint_pos("yaw_body", angle)
            self.step()
            time.sleep(0.1)

        return True

    def _execute_interaction(self, duration: float) -> bool:
        """Exécute le comportement d'interaction."""
        import math
        import time

        steps = int(duration * 10)  # 10 Hz
        for step in range(steps):
            t = step / steps
            # Mouvement d'interaction (petits mouvements)
            angle = 0.15 * math.sin(6 * math.pi * t)  # Mouvements rapides
            self.set_joint_pos("yaw_body", angle)
            self.step()
            time.sleep(0.1)

        return True

    def _execute_demo(self, duration: float) -> bool:
        """Exécute le comportement de démonstration."""
        import math
        import time

        steps = int(duration * 10)  # 10 Hz
        for step in range(steps):
            t = step / steps
            # Séquence de démonstration (combinaison de mouvements)
            if t < 0.33:
                angle = 0.2 * math.sin(4 * math.pi * t)
            elif t < 0.66:
                angle = 0.3 * (1 - math.cos(math.pi * (t - 0.33) * 3))
            else:
                angle = 0.2 * math.sin(4 * math.pi * t)
            self.set_joint_pos("yaw_body", angle)
            self.step()
            time.sleep(0.1)

        return True

    def clamp_joint_position(self, joint_name: str, position: float) -> float:
        """Clamp une position de joint dans les limites sûres."""
        if joint_name in self.forbidden_joints:
            logger.warning("Joint interdit: %s", joint_name)
            return 0.0

        # Limites générales
        position = max(
            -self.safe_amplitude_limit,
            min(self.safe_amplitude_limit, position),
        )

        # Limites spécifiques par joint
        if joint_name in self.joint_limits:
            min_limit, max_limit = self.joint_limits[joint_name]
            position = max(min_limit, min(max_limit, position))

        return position

    def get_status(self) -> RobotStatus:
        """Retourne le statut du robot.

        Returns:
            RobotStatus: Dictionnaire typé contenant le statut du robot
        """
        return {
            "connected": self.is_connected,
            "current_emotion": self.current_emotion,
            "emotion_intensity": self.emotion_intensity,
            "available_joints": len(self.get_available_joints()),
            "forbidden_joints": len(self.forbidden_joints),
            "safe_amplitude_limit": self.safe_amplitude_limit,
        }

    # NOTE EXPERT: RobotFactory a été déplacé dans robot_factory.py pour éviter duplication
    # ✅ MIGRATION TERMINÉE (Décembre 2025): Tous les imports utilisent maintenant robot_factory.py
    # Le mécanisme __getattr__ permet compatibilité ascendante pour code existant (déprécié)
    # Import déplacé en bas du fichier pour éviter circular import


# Import tardif pour compatibilité avec code existant (évite circular import)
# DÉPRÉCIÉ: Utiliser `from bbia_sim.robot_factory import RobotFactory` directement
def __getattr__(name: str) -> Any:
    """Import tardif pour RobotFactory depuis robot_factory (déprécié).

    Utiliser `from bbia_sim.robot_factory import RobotFactory` dans le nouveau code.
    """
    if name == "RobotFactory":
        import warnings

        warnings.warn(
            "Import de RobotFactory depuis robot_api est déprécié. "
            "Utiliser 'from bbia_sim.robot_factory import RobotFactory' directement.",
            DeprecationWarning,
            stacklevel=2,
        )
        from .robot_factory import RobotFactory

        return RobotFactory
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
