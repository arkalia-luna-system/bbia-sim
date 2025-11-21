#!/usr/bin/env python3
"""BBIA Meditation Behavior - Guide méditation avec mouvements lents.

Ce comportement guide l'utilisateur dans une séance de méditation
avec des mouvements lents et fluides, une voix calme, et une
respiration synchronisée.
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


class MeditationBehavior(BBIABehavior):
    """Comportement de guide méditation."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le comportement meditation.

        Args:
            robot_api: Interface robotique pour contrôler le robot
        """
        super().__init__(
            name="meditation",
            description="Guide méditation avec mouvements lents et voix calme",
            robot_api=robot_api,
        )
        self.duration_minutes = 5  # Durée par défaut
        self.current_phase = 0

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si le comportement peut être exécuté
        """
        if not self.robot_api:
            logger.warning("MeditationBehavior: robot_api non disponible")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement meditation.

        Args:
            context: Contexte d'exécution
                - 'duration': Durée en minutes (optionnel, défaut 5)

        Returns:
            True si l'exécution a réussi
        """
        if not self.robot_api:
            return False

        self.duration_minutes = context.get("duration", 5)
        self.is_active = True
        self.current_phase = 0

        try:
            self._guide_meditation()
            return True
        except Exception as e:
            logger.exception("Erreur lors de la méditation: %s", e)
            return False
        finally:
            self.is_active = False

    def _guide_meditation(self) -> None:
        """Guide la séance de méditation."""
        phases = [
            {
                "text": "Bienvenue dans cette séance de méditation. Asseyez-vous confortablement.",
                "emotion": "calm",
                "movement": {"yaw": 0.0, "pitch": 0.0},
                "duration": 3.0,
            },
            {
                "text": "Fermez les yeux et respirez profondément.",
                "emotion": "calm",
                "movement": {"yaw": 0.0, "pitch": 0.05},
                "duration": 4.0,
            },
            {
                "text": "Inspirez lentement... un, deux, trois...",
                "emotion": "calm",
                "movement": {"yaw": 0.0, "pitch": 0.1},
                "duration": 3.0,
            },
            {
                "text": "Expirez doucement... un, deux, trois...",
                "emotion": "calm",
                "movement": {"yaw": 0.0, "pitch": 0.0},
                "duration": 3.0,
            },
            {
                "text": "Répétez cette respiration plusieurs fois.",
                "emotion": "calm",
                "movement": {"yaw": 0.0, "pitch": -0.05},
                "duration": 5.0,
            },
            {
                "text": "Prenez votre temps, détendez-vous complètement.",
                "emotion": "calm",
                "movement": {"yaw": 0.0, "pitch": 0.0},
                "duration": 4.0,
            },
            {
                "text": "Quand vous êtes prêt, ouvrez doucement les yeux.",
                "emotion": "calm",
                "movement": {"yaw": 0.0, "pitch": 0.05},
                "duration": 3.0,
            },
        ]

        for i, phase in enumerate(phases):
            if not self.is_active:
                break

            self.current_phase = i
            self._execute_phase(phase)

    def _execute_phase(self, phase: dict[str, Any]) -> None:
        """Exécute une phase de méditation.

        Args:
            phase: Dictionnaire avec 'text', 'emotion', 'movement', 'duration'
        """
        if not self.robot_api:
            return

        # Appliquer émotion calme
        try:
            from ..bbia_emotions import BBIAEmotions

            emotions_module = BBIAEmotions()
            emotions_module.set_emotion(phase.get("emotion", "calm"), intensity=0.4)
        except ImportError:
            pass

        # Mouvement lent et fluide
        movement = phase.get("movement", {})
        if REACHY_MINI_UTILS_AVAILABLE and create_head_pose:
            pose = create_head_pose(
                yaw=movement.get("yaw", 0.0),
                pitch=movement.get("pitch", 0.0),
                degrees=False,
            )
            # Mouvement très lent (duration plus longue)
            self.robot_api.goto_target(head=pose, duration=3.0)

        # Parler avec voix calme
        text = phase.get("text", "")
        try:
            from ..bbia_voice import dire_texte

            dire_texte(text, robot_api=self.robot_api)
        except ImportError:
            logger.info("[MEDITATION] %s", text)

        # Attendre durée phase
        duration = phase.get("duration", 3.0)
        time.sleep(duration)

    def stop(self) -> None:
        """Arrête le comportement meditation."""
        self.is_active = False
        self.current_phase = 0
        logger.info("MeditationBehavior arrêté")
