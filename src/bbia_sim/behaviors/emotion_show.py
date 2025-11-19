#!/usr/bin/env python3
"""Comportement de démonstration des émotions BBIA.

Parcourt toutes les 12 émotions BBIA avec transitions fluides
et explications vocales.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from ..robot_api import RobotAPI

try:
    from ..bbia_emotions import BBIAEmotions
    from ..bbia_voice import dire_texte
except ImportError:
    BBIAEmotions = None  # type: ignore[assignment, misc]
    dire_texte = None  # type: ignore[assignment, misc]

from .base import BBIABehavior

logger = logging.getLogger("BBIA")

# Liste des 12 émotions BBIA
BBIA_EMOTIONS = [
    "neutral",
    "happy",
    "sad",
    "angry",
    "curious",
    "excited",
    "surprised",
    "fearful",
    "confused",
    "determined",
    "nostalgic",
    "proud",
]

# Durées adaptatives selon émotion (en secondes)
EMOTION_DURATIONS: dict[str, float] = {
    "neutral": 2.0,
    "happy": 3.0,
    "sad": 3.0,
    "angry": 2.5,
    "curious": 2.5,
    "excited": 3.5,
    "surprised": 2.0,
    "fearful": 2.5,
    "confused": 2.5,
    "determined": 3.0,
    "nostalgic": 3.0,
    "proud": 3.0,
}

# Explications vocales pour chaque émotion
EMOTION_EXPLANATIONS: dict[str, str] = {
    "neutral": "Maintenant je suis neutre, dans un état de repos.",
    "happy": "Maintenant je suis heureux, plein de joie !",
    "sad": "Maintenant je suis triste, un peu mélancolique.",
    "angry": "Maintenant je suis en colère, frustré.",
    "curious": "Maintenant je suis curieux, intrigué.",
    "excited": "Maintenant je suis excité, plein d'énergie !",
    "surprised": "Maintenant je suis surpris, étonné !",
    "fearful": "Maintenant je suis craintif, inquiet.",
    "confused": "Maintenant je suis confus, perplexe.",
    "determined": "Maintenant je suis déterminé, résolu.",
    "nostalgic": "Maintenant je suis nostalgique, mélancolique.",
    "proud": "Maintenant je suis fier, satisfait.",
}


class EmotionShowBehavior(BBIABehavior):
    """Comportement de démonstration des émotions BBIA.

    Parcourt toutes les 12 émotions avec transitions fluides
    et explications vocales.
    """

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        """Initialise le comportement de démonstration d'émotions.

        Args:
            robot_api: Instance RobotAPI pour contrôler le robot

        """
        super().__init__(
            "emotion_show",
            "Démonstration des 12 émotions BBIA",
            robot_api=robot_api,
        )
        self.priority = 5

        # Initialiser émotions si disponible
        if BBIAEmotions:
            self.emotions = BBIAEmotions()
        else:
            self.emotions = None

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si robot_api disponible

        """
        return self.robot_api is not None

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute la démonstration des émotions.

        Args:
            context: Contexte d'exécution (peut contenir emotions_list pour émotions spécifiques)

        Returns:
            True si la démonstration a réussi

        """
        if not self.robot_api:
            logger.error("Robot API non disponible")
            return False

        # Liste d'émotions à démontrer (par défaut toutes)
        emotions_to_show = context.get("emotions_list", BBIA_EMOTIONS)

        logger.info(f"Démarrage démonstration de {len(emotions_to_show)} émotions")

        try:
            for emotion in emotions_to_show:
                if emotion not in BBIA_EMOTIONS:
                    logger.warning(f"Émotion inconnue ignorée: {emotion}")
                    continue

                logger.info(f"Affichage émotion: {emotion}")

                # Appliquer l'émotion avec transition fluide
                self._apply_emotion(emotion)

                # Attendre un peu avant l'explication vocale
                time.sleep(0.5)

                # Explication vocale
                if dire_texte:
                    explanation = EMOTION_EXPLANATIONS.get(
                        emotion,
                        f"Maintenant je suis {emotion}.",
                    )
                    dire_texte(explanation, robot_api=self.robot_api)
                    logger.info(f"Explication vocale: {explanation}")

                # Maintenir l'émotion selon durée adaptative
                duration = EMOTION_DURATIONS.get(emotion, 2.5)
                time.sleep(duration)

                # Transition fluide vers émotion suivante
                time.sleep(0.3)

            logger.info("Démonstration des émotions terminée")

            # Retour à l'émotion neutre
            self._apply_emotion("neutral")
            time.sleep(1.0)

        except KeyboardInterrupt:
            logger.info("Démonstration interrompue par l'utilisateur")
        except Exception as e:
            logger.error(f"Erreur durant démonstration émotions: {e}")
            return False

        return True

    def _apply_emotion(self, emotion: str) -> None:
        """Applique une émotion avec transition fluide.

        Args:
            emotion: Nom de l'émotion à appliquer

        """
        if not self.robot_api or not hasattr(self.robot_api, "set_emotion"):
            return

        try:
            # Mapper vers émotions SDK si nécessaire
            sdk_emotions = {"happy", "sad", "neutral", "excited", "curious", "calm"}

            if emotion in sdk_emotions:
                sdk_emotion = emotion
                intensity = 0.8
            else:
                # Mapper les autres émotions BBIA vers SDK
                emotion_map = {
                    "angry": "excited",
                    "surprised": "curious",
                    "fearful": "sad",
                    "confused": "curious",
                    "determined": "neutral",
                    "nostalgic": "sad",
                    "proud": "happy",
                }
                sdk_emotion = emotion_map.get(emotion, "neutral")
                intensity = 0.7

            self.robot_api.set_emotion(sdk_emotion, intensity)
            logger.debug(f"Émotion appliquée: {sdk_emotion} (intensité: {intensity})")

            # Mouvements expressifs selon émotion
            self._apply_expressive_movement(emotion)

        except Exception as e:
            logger.warning(f"Erreur application émotion {emotion}: {e}")

    def _apply_expressive_movement(self, emotion: str) -> None:
        """Applique un mouvement expressif selon l'émotion.

        Args:
            emotion: Nom de l'émotion

        """
        if not self.robot_api:
            return

        try:
            # Mouvements selon émotion (utiliser goto_target pour fluidité)
            movements: dict[str, float] = {
                "happy": 0.12,
                "excited": 0.15,
                "curious": 0.10,
                "sad": -0.08,
                "angry": 0.0,
                "surprised": 0.12,
                "fearful": -0.10,
                "confused": 0.05,
                "determined": 0.0,
                "nostalgic": -0.05,
                "proud": 0.10,
                "neutral": 0.0,
            }

            yaw = movements.get(emotion, 0.0)

            if yaw != 0.0 and hasattr(self.robot_api, "goto_target"):
                self.robot_api.goto_target(
                    body_yaw=yaw,
                    duration=0.6,
                    method="minjerk",
                )
                time.sleep(0.7)
                self.robot_api.goto_target(
                    body_yaw=0.0,
                    duration=0.6,
                    method="minjerk",
                )

        except Exception as e:
            logger.warning(f"Erreur mouvement expressif {emotion}: {e}")
