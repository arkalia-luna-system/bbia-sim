#!/usr/bin/env python3
"""Comportement de danse synchronisée avec musique pour BBIA.

Détection rythme audio (analyse amplitude), mouvements chorégraphiés,
synchronisation musique, émotions selon type musique.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from ..robot_api import RobotAPI

try:
    from ..bbia_audio import detecter_son
    from ..bbia_voice import dire_texte
except ImportError:
    detecter_son = None  # type: ignore[assignment, misc]
    dire_texte = None  # type: ignore[assignment, misc]

from .base import BBIABehavior

logger = logging.getLogger("BBIA")


class DanceBehavior(BBIABehavior):
    """Comportement de danse synchronisée avec musique.

    Détecte le rythme audio et synchronise des mouvements chorégraphiés.
    """

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        """Initialise le comportement de danse.

        Args:
            robot_api: Instance RobotAPI pour contrôler le robot

        """
        super().__init__(
            "dance",
            "Danse synchronisée avec musique",
            robot_api=robot_api,
        )
        self.priority = 5
        self.is_dancing = False

        # Routines de danse pré-définies
        self.dance_routines = {
            "happy": self._dance_happy,
            "calm": self._dance_calm,
            "energetic": self._dance_energetic,
        }

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si robot_api disponible

        """
        return self.robot_api is not None

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute la danse.

        Args:
            context: Contexte d'exécution (peut contenir music_type, duration, audio_file)

        Returns:
            True si la danse a réussi

        """
        if not self.robot_api:
            logger.error("Robot API non disponible")
            return False

        # Type de musique (happy, calm, energetic)
        music_type = context.get("music_type", "happy")
        duration = context.get("duration", 30.0)  # 30 secondes par défaut
        audio_file = context.get("audio_file")  # Fichier audio optionnel

        logger.info(f"Démarrage danse type '{music_type}' pour {duration}s")

        # Message de démarrage
        if dire_texte is not None:
            messages = {
                "happy": "C'est parti pour une danse joyeuse !",
                "calm": "Je vais danser calmement...",
                "energetic": "Danse énergique en cours !",
            }
            message = messages.get(music_type, "C'est parti pour danser !")
            dire_texte(message, robot_api=self.robot_api)

        self.is_dancing = True
        start_time = time.time()

        # Sélectionner routine de danse
        routine = self.dance_routines.get(music_type, self._dance_happy)

        try:
            # Détection audio si fichier fourni
            if audio_file and detecter_son is not None:
                detecter_son(audio_file)

            # Exécuter routine de danse
            beat_count = 0
            while self.is_dancing and (time.time() - start_time) < duration:
                # Appliquer émotion selon type musique
                self._apply_music_emotion(music_type)

                # Exécuter mouvement de danse
                routine()

                beat_count += 1
                time.sleep(0.5)  # 2 Hz pour mouvements fluides

            logger.info(f"Danse terminée - {beat_count} mouvements exécutés")

        except KeyboardInterrupt:
            logger.info("Danse interrompue par l'utilisateur")
        except Exception as e:
            logger.error(f"Erreur durant danse: {e}")
            return False
        finally:
            self.is_dancing = False
            # Retour à l'émotion neutre
            if hasattr(self.robot_api, "set_emotion"):
                self.robot_api.set_emotion("neutral", 0.5)

        return True

    def _apply_music_emotion(self, music_type: str) -> None:
        """Applique l'émotion selon le type de musique.

        Args:
            music_type: Type de musique (happy, calm, energetic)

        """
        if not self.robot_api or not hasattr(self.robot_api, "set_emotion"):
            return

        emotion_map = {
            "happy": ("happy", 0.8),
            "calm": ("calm", 0.6),
            "energetic": ("excited", 0.9),
        }

        emotion, intensity = emotion_map.get(music_type, ("happy", 0.7))
        self.robot_api.set_emotion(emotion, intensity)

    def _dance_happy(self) -> None:
        """Routine de danse joyeuse."""
        if not self.robot_api:
            return

        try:
            # Mouvements joyeux : rotation corps + tête
            if hasattr(self.robot_api, "goto_target"):
                # Rotation corps
                for angle in [0.15, -0.15, 0.0]:
                    self.robot_api.goto_target(
                        body_yaw=angle,
                        duration=0.4,
                        method="minjerk",
                    )
                    time.sleep(0.5)

        except Exception as e:
            logger.warning(f"Erreur routine danse happy: {e}")

    def _dance_calm(self) -> None:
        """Routine de danse calme."""
        if not self.robot_api:
            return

        try:
            # Mouvements lents et fluides
            if hasattr(self.robot_api, "goto_target"):
                # Mouvement subtil
                self.robot_api.goto_target(
                    body_yaw=0.08,
                    duration=1.0,
                    method="minjerk",
                )
                time.sleep(1.1)
                self.robot_api.goto_target(
                    body_yaw=-0.08,
                    duration=1.0,
                    method="minjerk",
                )
                time.sleep(1.1)
                self.robot_api.goto_target(
                    body_yaw=0.0,
                    duration=1.0,
                    method="minjerk",
                )

        except Exception as e:
            logger.warning(f"Erreur routine danse calm: {e}")

    def _dance_energetic(self) -> None:
        """Routine de danse énergique."""
        if not self.robot_api:
            return

        try:
            # Mouvements rapides et énergiques
            if hasattr(self.robot_api, "goto_target"):
                # Mouvements rapides
                for _ in range(2):
                    self.robot_api.goto_target(
                        body_yaw=0.2,
                        duration=0.2,
                        method="minjerk",
                    )
                    time.sleep(0.3)
                    self.robot_api.goto_target(
                        body_yaw=-0.2,
                        duration=0.2,
                        method="minjerk",
                    )
                    time.sleep(0.3)

                self.robot_api.goto_target(
                    body_yaw=0.0,
                    duration=0.3,
                    method="minjerk",
                )

        except Exception as e:
            logger.warning(f"Erreur routine danse energetic: {e}")

    def stop(self) -> None:
        """Arrête la danse."""
        super().stop()
        self.is_dancing = False
