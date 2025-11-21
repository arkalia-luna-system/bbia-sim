#!/usr/bin/env python3
"""BBIA News Reader Behavior - Lecture actualités avec réactions.

Ce comportement permet à BBIA de lire les actualités avec des réactions
émotionnelles selon le contenu, et un résumé des nouvelles.
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


class NewsReaderBehavior(BBIABehavior):
    """Comportement de lecture d'actualités."""

    def __init__(self, robot_api: "RobotAPI | None" = None) -> None:
        """Initialise le comportement news_reader.

        Args:
            robot_api: Interface robotique pour contrôler le robot
        """
        super().__init__(
            name="news_reader",
            description="Lecture actualités avec réactions émotionnelles",
            robot_api=robot_api,
        )
        self.news_items: list[dict[str, Any]] = []

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si le comportement peut être exécuté
        """
        if not self.robot_api:
            logger.warning("NewsReaderBehavior: robot_api non disponible")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le comportement news_reader.

        Args:
            context: Contexte d'exécution
                - 'news_items': Liste d'actualités (optionnel, sinon récupération)
                - 'max_items': Nombre max d'actualités à lire (optionnel, défaut 5)

        Returns:
            True si l'exécution a réussi
        """
        if not self.robot_api:
            return False

        self.news_items = context.get("news_items", [])
        max_items = context.get("max_items", 5)

        # Si pas d'actualités, simuler (dans version réelle, utiliser RSS/API)
        if not self.news_items:
            self.news_items = self._simulate_news(max_items)

        self.is_active = True

        try:
            self._read_news()
            return True
        except Exception as e:
            logger.exception("Erreur lors de la lecture d'actualités: %s", e)
            return False
        finally:
            self.is_active = False

    def _simulate_news(self, max_items: int) -> list[dict[str, Any]]:
        """Simule des actualités (à remplacer par vraie API/RSS).

        Args:
            max_items: Nombre d'actualités

        Returns:
            Liste d'actualités simulées
        """
        # Simulation - dans version réelle, utiliser RSS feed ou API
        news = [
            {
                "title": "Découverte scientifique majeure",
                "content": "Des chercheurs ont fait une découverte importante.",
                "emotion": "excited",
            },
            {
                "title": "Nouvelles technologies",
                "content": "Les nouvelles technologies continuent d'évoluer.",
                "emotion": "curious",
            },
            {
                "title": "Événement culturel",
                "content": "Un grand événement culturel se prépare.",
                "emotion": "happy",
            },
        ]
        return news[:max_items]

    def _read_news(self) -> None:
        """Lit les actualités avec réactions."""
        if not self.news_items:
            return

        # Introduction
        intro = f"Voici les actualités du jour. J'ai {len(self.news_items)} nouvelles à vous présenter."
        self._speak_with_movement(intro, emotion="neutral")

        time.sleep(1.0)

        # Lire chaque actualité
        for i, news in enumerate(self.news_items):
            if not self.is_active:
                break

            title = news.get("title", "")
            content = news.get("content", "")
            emotion = news.get("emotion", "neutral")

            # Titre
            title_text = f"Actualité {i+1}: {title}"
            self._speak_with_movement(
                title_text,
                emotion=emotion,
                movement={"yaw": 0.1, "pitch": 0.1},
            )

            time.sleep(1.0)

            # Contenu
            self._speak_with_movement(
                content,
                emotion=emotion,
                movement={"yaw": 0.0, "pitch": 0.0},
            )

            time.sleep(2.0)

        # Résumé
        summary = "Voilà pour les actualités du jour. Passez une bonne journée !"
        self._speak_with_movement(summary, emotion="happy")

    def _speak_with_movement(
        self,
        text: str,
        emotion: str = "neutral",
        movement: dict[str, float] | None = None,
    ) -> None:
        """Parle avec mouvement expressif.

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
            self.robot_api.goto_target(head=pose, duration=1.5)

        # Parler
        try:
            from ..bbia_voice import dire_texte

            dire_texte(text, robot_api=self.robot_api)
        except ImportError:
            logger.info("[NEWS] %s", text)

    def stop(self) -> None:
        """Arrête le comportement news_reader."""
        self.is_active = False
        self.news_items = []
        logger.info("NewsReaderBehavior arrêté")
