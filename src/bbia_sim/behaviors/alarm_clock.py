#!/usr/bin/env python3
"""BBIA Alarm Clock Behavior - Réveil intelligent avec interactions.

Ce comportement permet à BBIA de fonctionner comme un réveil intelligent
avec séquence de réveil progressive, détection si l'utilisateur se réveille,
et mode snooze.
"""

import logging
import time
from datetime import datetime
from datetime import time as dt_time
from typing import TYPE_CHECKING, Any

from .base import BBIABehavior

if TYPE_CHECKING:
    from bbia_sim.robot_api import RobotAPI

logger = logging.getLogger(__name__)

# Import SDK officiel pour create_head_pose
try:
    from reachy_mini.utils import create_head_pose

    REACHY_MINI_UTILS_AVAILABLE = True
except ImportError:
    REACHY_MINI_UTILS_AVAILABLE = False
    create_head_pose = None


class AlarmClockBehavior(BBIABehavior):
    """Comportement de réveil intelligent."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le comportement alarm_clock.

        Args:
            robot_api: Interface robotique pour contrôler le robot

        """
        super().__init__(
            name="alarm_clock",
            description="Réveil intelligent avec séquence progressive",
            robot_api=robot_api,
        )
        self.alarm_time: dt_time | None = None
        self.snooze_minutes = 5
        self.is_ringing = False
        self.snooze_count = 0
        self.max_snooze = 3

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si le comportement peut être exécuté

        """
        if not self.robot_api:
            logger.warning("AlarmClockBehavior: robot_api non disponible")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement alarm_clock.

        Args:
            context: Contexte d'exécution
                - 'hour': Heure de réveil (0-23)
                - 'minute': Minute de réveil (0-59)
                - 'snooze_minutes': Minutes de snooze (optionnel, défaut 5)

        Returns:
            True si l'exécution a réussi

        """
        if not self.robot_api:
            return False

        hour = context.get("hour", 7)
        minute = context.get("minute", 0)
        self.snooze_minutes = context.get("snooze_minutes", 5)

        self.alarm_time = dt_time(hour, minute)
        self.is_active = True
        self.is_ringing = False
        self.snooze_count = 0

        try:
            self._wait_for_alarm()
            return True
        except Exception:
            logger.exception("Erreur lors du réveil")
            return False
        finally:
            self.is_active = False

    def _wait_for_alarm(self) -> None:
        """Attend jusqu'à l'heure du réveil."""
        if not self.alarm_time:
            return

        while self.is_active:
            now = datetime.now().time()
            if now >= self.alarm_time:
                self._ring_alarm()
                break
            time.sleep(10)  # Vérifier toutes les 10 secondes

    def _ring_alarm(self) -> None:
        """Fait sonner le réveil avec séquence progressive."""
        self.is_ringing = True

        # Séquence progressive
        phases = [
            {
                "text": "Bonjour ! Il est temps de se réveiller.",
                "emotion": "happy",
                "movement": {"yaw": 0.0, "pitch": 0.1},
                "intensity": 0.5,
            },
            {
                "text": "Réveillez-vous doucement...",
                "emotion": "calm",
                "movement": {"yaw": 0.1, "pitch": 0.1},
                "intensity": 0.6,
            },
            {
                "text": "Il est l'heure ! Réveillez-vous !",
                "emotion": "excited",
                "movement": {"yaw": -0.1, "pitch": 0.15},
                "intensity": 0.8,
            },
        ]

        phase_index = 0
        while self.is_ringing and self.is_active:
            if phase_index < len(phases):
                phase = phases[phase_index]
                self._execute_alarm_phase(phase)
                phase_index += 1
            else:
                # Répéter dernière phase
                self._execute_alarm_phase(phases[-1])

            # Vérifier si utilisateur s'est réveillé (simulation)
            # Dans version réelle, utiliser vision pour détecter mouvement
            time.sleep(3.0)

    def _execute_alarm_phase(self, phase: dict[str, Any]) -> None:
        """Exécute une phase du réveil.

        Args:
            phase: Dictionnaire avec 'text', 'emotion', 'movement', 'intensity'

        """
        if not self.robot_api:
            return

        # Appliquer émotion
        try:
            from bbia_sim.bbia_emotions import BBIAEmotions

            emotions_module = BBIAEmotions()
            emotions_module.set_emotion(
                phase.get("emotion", "neutral"),
                intensity=phase.get("intensity", 0.7),
            )
        except ImportError:
            pass

        # Mouvement
        movement = phase.get("movement", {})
        if REACHY_MINI_UTILS_AVAILABLE and create_head_pose:
            pose = create_head_pose(
                yaw=movement.get("yaw", 0.0),
                pitch=movement.get("pitch", 0.0),
                degrees=False,
            )
            self.robot_api.goto_target(head=pose, duration=1.0)

        # Parler
        text = phase.get("text", "")
        try:
            from bbia_sim.bbia_voice import dire_texte

            dire_texte(text, robot_api=self.robot_api)
        except ImportError:
            logger.info("[ALARM] %s", text)

    def snooze(self) -> bool:
        """Active le mode snooze.

        Returns:
            True si snooze activé, False si max atteint

        """
        if self.snooze_count >= self.max_snooze:
            logger.warning("Nombre maximum de snooze atteint")
            return False

        self.snooze_count += 1
        self.is_ringing = False

        # Calculer nouvelle heure
        if self.alarm_time:
            # Ajouter minutes de snooze
            alarm_datetime = datetime.combine(datetime.now().date(), self.alarm_time)
            snooze_datetime = alarm_datetime.replace(
                minute=alarm_datetime.minute + self.snooze_minutes,
            )
            self.alarm_time = snooze_datetime.time()

        logger.info(
            "Snooze activé (%d/%d). Nouveau réveil à %s",
            self.snooze_count,
            self.max_snooze,
            self.alarm_time,
        )

        # Message
        try:
            from bbia_sim.bbia_voice import dire_texte

            dire_texte(
                f"D'accord, je vous réveillerai dans {self.snooze_minutes} minutes.",
                robot_api=self.robot_api,
            )
        except ImportError:
            pass

        # Attendre nouveau réveil
        self._wait_for_alarm()
        return True

    def stop(self) -> None:
        """Arrête le comportement alarm_clock."""
        self.is_active = False
        self.is_ringing = False
        self.alarm_time = None
        logger.info("AlarmClockBehavior arrêté")
