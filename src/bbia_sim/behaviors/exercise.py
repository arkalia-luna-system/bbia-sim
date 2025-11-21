#!/usr/bin/env python3
"""BBIA Exercise Behavior - Guide exercices physiques.

Ce comportement guide l'utilisateur dans des exercices physiques
avec des mouvements démonstratifs, un comptage de répétitions,
et des encouragements.
"""

import logging
import time
from typing import TYPE_CHECKING, Any

from .base import BBIABehavior

if TYPE_CHECKING:
    from ..robot_api import RobotAPI

logger = logging.getLogger(__name__)

# Import SDK officiel pour create_head_pose
try:
    from reachy_mini.utils import create_head_pose

    REACHY_MINI_UTILS_AVAILABLE = True
except ImportError:
    REACHY_MINI_UTILS_AVAILABLE = False
    create_head_pose = None


class ExerciseBehavior(BBIABehavior):
    """Comportement de guide exercices physiques."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le comportement exercise.

        Args:
            robot_api: Interface robotique pour contrôler le robot
        """
        super().__init__(
            name="exercise",
            description="Guide exercices physiques avec mouvements démonstratifs",
            robot_api=robot_api,
        )
        self.exercises = {
            "head_rotation": self._exercise_head_rotation,
            "neck_stretch": self._exercise_neck_stretch,
            "shoulder_roll": self._exercise_shoulder_roll,
        }
        self.current_exercise: str | None = None
        self.repetitions = 5

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si le comportement peut être exécuté
        """
        if not self.robot_api:
            logger.warning("ExerciseBehavior: robot_api non disponible")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement exercise.

        Args:
            context: Contexte d'exécution
                - 'exercise': Nom de l'exercice (head_rotation, neck_stretch, etc.)
                - 'repetitions': Nombre de répétitions (optionnel, défaut 5)

        Returns:
            True si l'exécution a réussi
        """
        if not self.robot_api:
            return False

        exercise_name = context.get("exercise", "head_rotation")
        self.repetitions = context.get("repetitions", 5)

        if exercise_name not in self.exercises:
            logger.warning("Exercice '%s' non trouvé", exercise_name)
            return False

        self.is_active = True
        self.current_exercise = exercise_name

        try:
            exercise_func = self.exercises[exercise_name]
            exercise_func()
            return True
        except Exception as e:
            logger.exception("Erreur lors de l'exercice: %s", e)
            return False
        finally:
            self.is_active = False

    def _exercise_head_rotation(self) -> None:
        """Exercice de rotation de la tête."""
        intro = "Faisons des rotations de tête. Suivez mes mouvements."
        self._speak_with_movement(intro, emotion="excited")

        for i in range(self.repetitions):
            if not self.is_active:
                break

            # Rotation droite
            self._speak_with_movement(
                f"Répétition {i+1}: Tournez la tête à droite",
                movement={"yaw": 0.3, "pitch": 0.0},
            )
            time.sleep(2.0)

            # Retour centre
            self._speak_with_movement(
                "Retour au centre",
                movement={"yaw": 0.0, "pitch": 0.0},
            )
            time.sleep(1.0)

            # Rotation gauche
            self._speak_with_movement(
                "Tournez la tête à gauche",
                movement={"yaw": -0.3, "pitch": 0.0},
            )
            time.sleep(2.0)

            # Retour centre
            self._speak_with_movement(
                "Retour au centre",
                movement={"yaw": 0.0, "pitch": 0.0},
            )
            time.sleep(1.0)

        self._encourage()

    def _exercise_neck_stretch(self) -> None:
        """Exercice d'étirement du cou."""
        intro = "Faisons des étirements du cou. Suivez mes mouvements."
        self._speak_with_movement(intro, emotion="excited")

        for i in range(self.repetitions):
            if not self.is_active:
                break

            # Tête en haut
            self._speak_with_movement(
                f"Répétition {i+1}: Regardez vers le haut",
                movement={"yaw": 0.0, "pitch": 0.2},
            )
            time.sleep(2.0)

            # Retour centre
            self._speak_with_movement(
                "Retour au centre",
                movement={"yaw": 0.0, "pitch": 0.0},
            )
            time.sleep(1.0)

            # Tête en bas
            self._speak_with_movement(
                "Regardez vers le bas",
                movement={"yaw": 0.0, "pitch": -0.2},
            )
            time.sleep(2.0)

            # Retour centre
            self._speak_with_movement(
                "Retour au centre",
                movement={"yaw": 0.0, "pitch": 0.0},
            )
            time.sleep(1.0)

        self._encourage()

    def _exercise_shoulder_roll(self) -> None:
        """Exercice de rotation des épaules (simulé avec mouvement tête)."""
        intro = "Faisons des rotations d'épaules. Suivez mes mouvements."
        self._speak_with_movement(intro, emotion="excited")

        for i in range(self.repetitions):
            if not self.is_active:
                break

            # Mouvement circulaire (simulé)
            self._speak_with_movement(
                f"Répétition {i+1}: Rotation complète",
                movement={"yaw": 0.2, "pitch": 0.1},
            )
            time.sleep(1.0)

            self._speak_with_movement(
                "Continuez",
                movement={"yaw": -0.2, "pitch": -0.1},
            )
            time.sleep(1.0)

            self._speak_with_movement(
                "Retour au centre",
                movement={"yaw": 0.0, "pitch": 0.0},
            )
            time.sleep(1.0)

        self._encourage()

    def _speak_with_movement(
        self,
        text: str,
        emotion: str = "neutral",
        movement: dict[str, float] | None = None,
    ) -> None:
        """Parle avec mouvement démonstratif.

        Args:
            text: Texte à dire
            emotion: Émotion à exprimer
            movement: Mouvement tête (yaw, pitch)
        """
        if not self.robot_api:
            return

        # Appliquer émotion
        try:
            from ..bbia_emotions import BBIAEmotions

            emotions_module = BBIAEmotions()
            emotions_module.set_emotion(emotion, intensity=0.6)
        except ImportError:
            pass

        # Appliquer mouvement
        if movement and REACHY_MINI_UTILS_AVAILABLE and create_head_pose:
            pose = create_head_pose(
                yaw=movement.get("yaw", 0.0),
                pitch=movement.get("pitch", 0.0),
                degrees=False,
            )
            self.robot_api.goto_target(head=pose, duration=1.0)

        # Parler
        try:
            from ..bbia_voice import dire_texte

            dire_texte(text, robot_api=self.robot_api)
        except ImportError:
            logger.info("[EXERCISE] %s", text)

    def _encourage(self) -> None:
        """Encourage l'utilisateur."""
        messages = [
            "Excellent travail !",
            "Très bien !",
            "Bravo !",
            "Continuez comme ça !",
        ]
        import random

        message = random.choice(messages)  # nosec B311
        self._speak_with_movement(message, emotion="happy")

    def stop(self) -> None:
        """Arrête le comportement exercise."""
        self.is_active = False
        self.current_exercise = None
        logger.info("ExerciseBehavior arrêté")
