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
    from bbia_sim.robot_api import RobotAPI

try:
    from bbia_sim.bbia_emotions import BBIAEmotions
    from bbia_sim.bbia_voice import dire_texte
except ImportError:
    BBIAEmotions = None  # type: ignore[assignment, misc]
    dire_texte = None  # type: ignore[assignment, misc]

from bbia_sim.behaviors.base import BBIABehavior

logger = logging.getLogger("BBIA")

# Liste des 12 émotions BBIA (frozenset pour recherches O(1) et immuable)
BBIA_EMOTIONS_LIST = [
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
BBIA_EMOTIONS = frozenset(BBIA_EMOTIONS_LIST)

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

# Mapping des émotions BBIA vers émotions SDK avec intensités pré-calculées
# Format: (sdk_emotion, intensity)
SDK_EMOTIONS = frozenset({"happy", "sad", "neutral", "excited", "curious", "calm"})

EMOTION_SDK_MAP: dict[str, tuple[str, float]] = {
    # Émotions directes SDK (intensité 0.8)
    "happy": ("happy", 0.8),
    "sad": ("sad", 0.8),
    "neutral": ("neutral", 0.8),
    "excited": ("excited", 0.8),
    "curious": ("curious", 0.8),
    # Émotions mappées (intensité 0.7)
    "angry": ("excited", 0.7),
    "surprised": ("curious", 0.7),
    "fearful": ("sad", 0.7),
    "confused": ("curious", 0.7),
    "determined": ("neutral", 0.7),
    "nostalgic": ("sad", 0.7),
    "proud": ("happy", 0.7),
}

# Mouvements expressifs selon émotion (yaw en radians)
EXPRESSIVE_MOVEMENTS: dict[str, float] = {
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

# Constantes pour les durées et transitions
PRE_VOICE_DELAY = 0.5
TRANSITION_DELAY = 0.3
RETURN_TO_NEUTRAL_DELAY = 1.0
MOVEMENT_DURATION = 0.6
MOVEMENT_WAIT = 0.7
SLEEP_CHECK_INTERVAL = 0.1


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
        if BBIAEmotions is not None:
            self.emotions = BBIAEmotions()
        else:
            self.emotions = None

        # Cache des capacités du robot pour éviter les appels répétés à hasattr
        self._has_set_emotion: bool | None = None
        self._has_goto_target: bool | None = None
        self._cancelled = False

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si robot_api disponible

        """
        return self.robot_api is not None

    def cancel(self) -> None:
        """Annule l'exécution en cours."""
        self._cancelled = True
        logger.info("Annulation de la démonstration demandée")

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

        # Réinitialiser le flag d'annulation
        self._cancelled = False

        # Initialiser le cache des capacités du robot
        self._has_set_emotion = hasattr(self.robot_api, "set_emotion")
        self._has_goto_target = hasattr(self.robot_api, "goto_target")

        # Liste d'émotions à démontrer (par défaut toutes)
        emotions_to_show = context.get("emotions_list", BBIA_EMOTIONS_LIST)

        # Filtrer et valider les émotions en une seule passe (O(n) avec set lookup O(1))
        valid_emotions = [
            emotion for emotion in emotions_to_show if emotion in BBIA_EMOTIONS
        ]

        if not valid_emotions:
            logger.warning("Aucune émotion valide à démontrer")
            return False

        # Avertir sur les émotions invalides (optimisé avec set difference)
        if isinstance(emotions_to_show, list | tuple):
            invalid_emotions = set(emotions_to_show) - BBIA_EMOTIONS
            if invalid_emotions:
                logger.warning("Émotions inconnues ignorées: %s", invalid_emotions)

        logger.info("Démarrage démonstration de %s émotions", len(valid_emotions))

        try:
            for emotion in valid_emotions:
                # Vérifier si annulation demandée
                if self._cancelled:
                    logger.info("Démonstration annulée")
                    break

                logger.info("Affichage émotion: %s", emotion)

                # Appliquer l'émotion avec transition fluide
                if not self._apply_emotion(emotion):
                    logger.warning(
                        f"Échec application émotion {emotion}, passage à la suivante"
                    )
                    continue

                # Attendre un peu avant l'explication vocale
                if self._sleep_with_cancel(PRE_VOICE_DELAY):
                    break

                # Explication vocale (pré-charger l'explication pour éviter lookup répété)
                if dire_texte is not None:
                    explanation = EMOTION_EXPLANATIONS.get(
                        emotion,
                        f"Maintenant je suis {emotion}.",
                    )
                    try:
                        dire_texte(explanation, robot_api=self.robot_api)
                        logger.info("Explication vocale: %s", explanation)
                    except Exception as e:
                        logger.warning(
                            "Erreur explication vocale pour %s: %s", emotion, e
                        )

                # Maintenir l'émotion selon durée adaptative (pré-charger la durée)
                duration = EMOTION_DURATIONS.get(emotion, 2.5)
                if self._sleep_with_cancel(duration):
                    break

                # Transition fluide vers émotion suivante
                if self._sleep_with_cancel(TRANSITION_DELAY):
                    break

            if not self._cancelled:
                logger.info("Démonstration des émotions terminée")
                # Retour à l'émotion neutre
                self._apply_emotion("neutral")
                self._sleep_with_cancel(RETURN_TO_NEUTRAL_DELAY)

        except KeyboardInterrupt:
            logger.info("Démonstration interrompue par l'utilisateur")
            self._cancelled = True
        except Exception as e:
            logger.exception("Erreur durant démonstration émotions: %s", e)
            return False

        return True

    def _sleep_with_cancel(self, duration: float) -> bool:
        """Fait une pause avec vérification d'annulation.

        Args:
            duration: Durée en secondes

        Returns:
            True si annulé, False sinon

        """
        if self._cancelled:
            return True

        # Pour les durées courtes, sleep direct (moins de overhead)
        if duration <= SLEEP_CHECK_INTERVAL:
            time.sleep(duration)
            return self._cancelled

        # Diviser le sleep en petits intervalles pour permettre l'annulation
        # Optimisé: calculer le nombre d'intervalles une seule fois
        num_intervals = max(1, int(duration / SLEEP_CHECK_INTERVAL))
        interval = duration / num_intervals

        for _ in range(num_intervals):
            if self._cancelled:
                return True
            time.sleep(interval)

        return self._cancelled

    def _apply_emotion(self, emotion: str) -> bool:
        """Applique une émotion avec transition fluide.

        Args:
            emotion: Nom de l'émotion à appliquer

        Returns:
            True si l'émotion a été appliquée avec succès, False sinon

        """
        if not self.robot_api:
            return False

        # Utiliser le cache au lieu de hasattr répété
        if self._has_set_emotion is None:
            self._has_set_emotion = hasattr(self.robot_api, "set_emotion")

        if not self._has_set_emotion:
            return False

        try:
            # Utiliser le mapping pré-calculé (évite les if/else répétés)
            sdk_emotion, intensity = EMOTION_SDK_MAP.get(emotion, ("neutral", 0.7))

            self.robot_api.set_emotion(sdk_emotion, intensity)
            logger.debug(
                "Émotion appliquée: %s (intensité: %s)", sdk_emotion, intensity
            )

            # Mouvements expressifs selon émotion
            self._apply_expressive_movement(emotion)

            return True

        except Exception as e:
            logger.warning("Erreur application émotion %s: %s", emotion, e)
            return False

    def _apply_expressive_movement(self, emotion: str) -> None:
        """Applique un mouvement expressif selon l'émotion.

        Args:
            emotion: Nom de l'émotion

        """
        if not self.robot_api:
            return

        # Utiliser le cache au lieu de hasattr répété
        if self._has_goto_target is None:
            self._has_goto_target = hasattr(self.robot_api, "goto_target")

        if not self._has_goto_target:
            return

        try:
            # Utiliser la constante de classe au lieu de recréer le dict
            yaw = EXPRESSIVE_MOVEMENTS.get(emotion, 0.0)

            if yaw != 0.0:
                self.robot_api.goto_target(
                    body_yaw=yaw,
                    duration=MOVEMENT_DURATION,
                    method="minjerk",
                )
                # Utiliser sleep avec annulation pour cohérence
                if self._sleep_with_cancel(MOVEMENT_WAIT):
                    return
                self.robot_api.goto_target(
                    body_yaw=0.0,
                    duration=MOVEMENT_DURATION,
                    method="minjerk",
                )

        except Exception as e:
            logger.warning("Erreur mouvement expressif %s: %s", emotion, e)
