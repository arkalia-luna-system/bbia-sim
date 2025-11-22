#!/usr/bin/env python3
"""BBIA Music Reaction Behavior - Réagir à la musique avec mouvements.

Ce comportement permet à BBIA de réagir à la musique avec des mouvements
synchronisés, des émotions selon le genre musical, et une détection
du rythme pour adapter les mouvements.
"""

import logging
import time
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


class MusicReactionBehavior(BBIABehavior):
    """Comportement de réaction à la musique."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le comportement music_reaction.

        Args:
            robot_api: Interface robotique pour contrôler le robot

        """
        super().__init__(
            name="music_reaction",
            description="Réagir à la musique avec mouvements synchronisés",
            robot_api=robot_api,
        )
        self.music_genres = {
            "pop": {"emotion": "happy", "tempo": "fast", "movement_style": "energetic"},
            "classical": {
                "emotion": "calm",
                "tempo": "slow",
                "movement_style": "smooth",
            },
            "rock": {
                "emotion": "excited",
                "tempo": "fast",
                "movement_style": "energetic",
            },
            "jazz": {
                "emotion": "curious",
                "tempo": "medium",
                "movement_style": "smooth",
            },
        }
        self.current_genre: str | None = None
        self.is_dancing = False

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si le comportement peut être exécuté

        """
        if not self.robot_api:
            logger.warning("MusicReactionBehavior: robot_api non disponible")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement music_reaction.

        Args:
            context: Contexte d'exécution
                - 'genre': Genre musical (pop, classical, rock, jazz)
                - 'duration': Durée en secondes (optionnel)

        Returns:
            True si l'exécution a réussi

        """
        if not self.robot_api:
            return False

        genre = context.get("genre", "pop")
        duration = context.get("duration", 30)  # 30 secondes par défaut

        if genre not in self.music_genres:
            logger.warning("Genre musical '%s' non trouvé", genre)
            return False

        self.is_active = True
        self.current_genre = genre
        self.is_dancing = True

        try:
            self._react_to_music(genre, duration)
            return True
        except Exception as e:
            logger.exception("Erreur lors de la réaction musicale")
            return False
        finally:
            self.is_active = False
            self.is_dancing = False

    def _react_to_music(self, genre: str, duration: float) -> None:
        """Réagit à la musique avec mouvements.

        Args:
            genre: Genre musical
            duration: Durée en secondes

        """
        genre_info = self.music_genres[genre]
        emotion = genre_info["emotion"]
        tempo = genre_info["tempo"]
        # movement_style utilisé pour déterminer style de mouvement (implémenté dans _fast_movement, etc.)

        # Appliquer émotion selon genre
        try:
            from bbia_sim.bbia_emotions import BBIAEmotions

            emotions_module = BBIAEmotions()
            emotions_module.set_emotion(emotion, intensity=0.7)
        except ImportError:
            pass

        # Message initial
        intro = f"J'adore cette musique {genre} ! Laissez-moi danser !"
        self._speak_with_movement(intro, emotion=emotion)

        # Mouvements synchronisés selon tempo
        start_time = time.time()
        movement_count = 0

        while self.is_active and (time.time() - start_time) < duration:
            if tempo == "fast":
                # Mouvements rapides et énergiques
                self._fast_movement(movement_count)
                time.sleep(0.5)  # Pause courte
            elif tempo == "slow":
                # Mouvements lents et fluides
                self._slow_movement(movement_count)
                time.sleep(2.0)  # Pause longue
            else:  # medium
                # Mouvements modérés
                self._medium_movement(movement_count)
                time.sleep(1.0)  # Pause moyenne

            movement_count += 1

    def _fast_movement(self, count: int) -> None:
        """Mouvement rapide et énergique.

        Args:
            count: Compteur de mouvement

        """
        if (
            not self.robot_api
            or not REACHY_MINI_UTILS_AVAILABLE
            or not create_head_pose
        ):
            return

        # Pattern de mouvement rapide
        patterns = [
            {"yaw": 0.3, "pitch": 0.1},
            {"yaw": -0.3, "pitch": -0.1},
            {"yaw": 0.2, "pitch": 0.15},
            {"yaw": -0.2, "pitch": -0.15},
        ]

        pattern = patterns[count % len(patterns)]
        pose = create_head_pose(
            yaw=pattern["yaw"],
            pitch=pattern["pitch"],
            degrees=False,
        )
        self.robot_api.goto_target(head=pose, duration=0.5)  # Rapide

    def _slow_movement(self, count: int) -> None:
        """Mouvement lent et fluide.

        Args:
            count: Compteur de mouvement

        """
        if (
            not self.robot_api
            or not REACHY_MINI_UTILS_AVAILABLE
            or not create_head_pose
        ):
            return

        # Pattern de mouvement lent
        patterns = [
            {"yaw": 0.2, "pitch": 0.1},
            {"yaw": -0.2, "pitch": 0.1},
            {"yaw": 0.0, "pitch": 0.15},
            {"yaw": 0.0, "pitch": -0.1},
        ]

        pattern = patterns[count % len(patterns)]
        pose = create_head_pose(
            yaw=pattern["yaw"],
            pitch=pattern["pitch"],
            degrees=False,
        )
        self.robot_api.goto_target(head=pose, duration=2.0)  # Lent

    def _medium_movement(self, count: int) -> None:
        """Mouvement modéré.

        Args:
            count: Compteur de mouvement

        """
        if (
            not self.robot_api
            or not REACHY_MINI_UTILS_AVAILABLE
            or not create_head_pose
        ):
            return

        # Pattern de mouvement modéré
        patterns = [
            {"yaw": 0.25, "pitch": 0.1},
            {"yaw": -0.25, "pitch": 0.1},
            {"yaw": 0.15, "pitch": 0.12},
            {"yaw": -0.15, "pitch": -0.08},
        ]

        pattern = patterns[count % len(patterns)]
        pose = create_head_pose(
            yaw=pattern["yaw"],
            pitch=pattern["pitch"],
            degrees=False,
        )
        self.robot_api.goto_target(head=pose, duration=1.0)  # Modéré

    def _speak_with_movement(self, text: str, emotion: str = "neutral") -> None:
        """Parle avec mouvement.

        Args:
            text: Texte à dire
            emotion: Émotion à exprimer

        """
        if not self.robot_api:
            return

        try:
            from bbia_sim.bbia_voice import dire_texte

            dire_texte(text, robot_api=self.robot_api)
        except ImportError:
            logger.info("[MUSIC_REACTION] %s", text)

    def stop(self) -> None:
        """Arrête le comportement music_reaction."""
        self.is_active = False
        self.is_dancing = False
        self.current_genre = None
        logger.info("MusicReactionBehavior arrêté")
