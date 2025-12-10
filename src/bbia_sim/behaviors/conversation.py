#!/usr/bin/env python3
"""Comportement de conversation amélioré pour BBIA.

Conversation naturelle avec LLM, mouvements expressifs selon émotions,
réactions visuelles (hochement tête).
"""

from __future__ import annotations

import logging
import secrets
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from bbia_sim.robot_api import RobotAPI

try:
    from bbia_sim.bbia_huggingface import BBIAHuggingFace
    from bbia_sim.bbia_tools import BBIATools
    from bbia_sim.bbia_vision import BBIAVision
    from bbia_sim.bbia_voice import dire_texte, reconnaitre_parole
except ImportError:
    BBIAHuggingFace = None  # type: ignore[assignment, misc]
    BBIATools = None  # type: ignore[assignment, misc]
    BBIAVision = None  # type: ignore[assignment, misc]
    dire_texte = None  # type: ignore[assignment, misc]
    reconnaitre_parole = None  # type: ignore[assignment, misc]

from bbia_sim.behaviors.base import BBIABehavior

try:
    from bbia_sim.bbia_emotional_sync import BBIAEmotionalSync
except ImportError:
    BBIAEmotionalSync = None  # type: ignore[assignment, misc]

logger = logging.getLogger("BBIA")


class ConversationBehavior(BBIABehavior):
    """Comportement de conversation amélioré.

    Utilise BBIAHuggingFace si disponible pour des réponses intelligentes,
    sinon utilise un système de réponses enrichies.
    """

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        """Initialise le comportement de conversation.

        Args:
            robot_api: Instance RobotAPI pour contrôler le robot

        """
        super().__init__(
            "conversation",
            "Conversation interactive intelligente",
            robot_api=robot_api,
        )
        self.priority = 7

        # Tentative d'importation de BBIAHuggingFace avec outils LLM
        self.hf_chat = None
        try:
            if (
                BBIAHuggingFace is not None
                and BBIATools is not None
                and BBIAVision is not None
            ):
                # Initialiser outils LLM pour function calling
                vision = BBIAVision(robot_api=robot_api) if robot_api else None
                tools = BBIATools(robot_api=robot_api, vision=vision)

                # Créer BBIAHuggingFace avec outils intégrés
                self.hf_chat = BBIAHuggingFace(tools=tools)
                logger.info(
                    (
                        "✅ BBIAHuggingFace avec outils LLM disponible - "
                        "Conversation intelligente + function calling activé"
                    ),
                )
        except (ImportError, Exception) as e:
            logger.info(
                "ℹ️  BBIAHuggingFace non disponible - Mode enrichi activé: %s",
                e,
            )
            self.hf_chat = None

        # Système de réponses enrichies (fallback)
        self.enriched_responses = self._init_enriched_responses()

        # Module de synchronisation fine mouvements émotionnels ↔ parole
        self.emotional_sync = None
        if BBIAEmotionalSync is not None:
            try:
                self.emotional_sync = BBIAEmotionalSync(robot_api=robot_api)
                logger.info("✅ Synchronisation fine mouvements émotionnels activée")
            except (ValueError, AttributeError, RuntimeError) as e:
                logger.warning("⚠️ Synchronisation fine non disponible: %s", e)
                self.emotional_sync = None

    def _init_enriched_responses(self) -> dict[str, list[str]]:
        """Initialise les réponses enrichies avec variété."""
        return {
            "greeting": [
                "Bonjour ! Je suis ravi de vous entendre. "
                "Comment allez-vous aujourd'hui ?",
                "Salut ! Ça fait plaisir de vous voir ! Qu'est-ce qui vous amène ?",
                "Hello ! Je suis BBIA, votre compagnon robotique. "
                "Que puis-je faire pour vous ?",
            ],
            "how_are_you": [
                "Je vais très bien, merci de demander ! Et vous ?",
                "Excellente journée, je suis en pleine forme !",
                "Tout va bien, je suis content d'être là.",
            ],
            "goodbye": [
                "Au revoir ! Ce fut un plaisir de discuter.",
                "À bientôt ! N'hésitez pas à revenir me voir.",
                "Au revoir ! Prenez bien soin de vous.",
            ],
            "thanks": [
                "De rien ! C'est un plaisir de vous aider.",
                "Je vous en prie ! Tout le plaisir est pour moi.",
            ],
            "positive": [
                "C'est super ! Je suis content pour vous.",
                "Fantastique ! Continuez comme ça.",
                "Excellent ! C'est une bonne nouvelle.",
            ],
            "question": [
                "C'est une excellente question ! Laissez-moi réfléchir...",
                "Hmm, intéressant ! Voici ce que je pense :",
                "Bonne question ! Voici comment je vois les choses :",
            ],
            "default": [
                "C'est intéressant ! Pouvez-vous m'en dire plus ?",
                "Je comprends. Continuez, je vous écoute attentivement.",
                "Ah, c'est fascinant ! Racontez-moi davantage.",
            ],
            "not_heard": [
                "Je n'ai pas bien entendu. Pouvez-vous répéter s'il vous plaît ?",
                "Pardon, pouvez-vous répéter ? Je n'ai pas saisi.",
            ],
        }

    def _get_enriched_response(self, category: str) -> str:
        """Récupère une réponse enrichie aléatoire dans une catégorie."""
        responses = self.enriched_responses.get(
            category,
            self.enriched_responses["default"],
        )
        return secrets.choice(responses)  # nosec B311

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si robot_api disponible

        """
        return self.robot_api is not None

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute une conversation intelligente.

        Args:
            context: Contexte d'exécution

        Returns:
            True si la conversation a réussi

        """
        if not self.robot_api:
            logger.error("Robot API non disponible")
            return False

        logger.info("Activation du mode conversation intelligente")

        # Démarrer micro-mouvements pendant écoute
        if self.emotional_sync:
            self.emotional_sync.start_listening_movements()

        # Message d'accueil
        if dire_texte is not None:
            greeting_messages = [
                "Je vous écoute attentivement.",
                "Je suis tout ouïe, parlez-moi.",
                "Dites-moi ce qui vous passe par la tête.",
            ]
            greeting = secrets.choice(greeting_messages)  # nosec B311
            # Utiliser synchronisation fine si disponible
            if self.emotional_sync:
                self.emotional_sync.sync_speak_with_emotion(
                    greeting,
                    emotion="neutral",
                    intensity=0.5,
                    speak_function=dire_texte,
                )
            else:
                dire_texte(greeting, robot_api=self.robot_api)
            logger.info("Synthèse vocale : %s", greeting)

        # Reconnaissance vocale
        texte = None
        if reconnaitre_parole is not None:
            texte = reconnaitre_parole(duree=5, robot_api=self.robot_api)
            logger.info("Texte reconnu : %s", texte)

        # Arrêter micro-mouvements écoute
        if self.emotional_sync:
            self.emotional_sync.stop_listening_movements()

        if texte:
            texte_lower = texte.lower()

            # Utiliser BBIAHuggingFace si disponible
            if self.hf_chat:
                try:
                    # Transition vers réflexion
                    if self.emotional_sync:
                        self.emotional_sync.transition_to_thinking()

                    response = self.hf_chat.chat(texte, enable_tools=True)
                    logger.info("Synthèse vocale (HF) : %s", response)

                    # Appliquer émotion correspondante
                    sentiment_result = self.hf_chat.analyze_sentiment(texte)
                    sentiment_dict: dict[str, Any] = {
                        "sentiment": sentiment_result.get("sentiment", "neutral"),
                        "score": sentiment_result.get("score", 0.5),
                    }

                    # Déterminer émotion
                    emotion_mapping = {
                        "POSITIVE": (
                            "happy",
                            min(sentiment_dict.get("score", 0.5), 0.9),
                        ),
                        "NEGATIVE": ("sad", min(sentiment_dict.get("score", 0.5), 0.7)),
                        "NEUTRAL": ("neutral", 0.5),
                    }
                    sentiment_label = sentiment_dict.get("sentiment", "NEUTRAL")
                    emotion, intensity = emotion_mapping.get(
                        sentiment_label, ("neutral", 0.5)
                    )

                    # Parler avec synchronisation fine
                    if dire_texte is not None:
                        if self.emotional_sync:
                            self.emotional_sync.sync_speak_with_emotion(
                                response,
                                emotion=emotion,
                                intensity=intensity,
                                speak_function=dire_texte,
                            )
                        else:
                            dire_texte(response, robot_api=self.robot_api)
                            self._apply_sentiment_to_robot(sentiment_dict)
                            self._expressive_movement("nod")

                    return True
                except (
                    ValueError,
                    TypeError,
                    KeyError,
                    AttributeError,
                    RuntimeError,
                ) as e:
                    logger.warning("Erreur BBIAHuggingFace, fallback enrichi : %s", e)

            # Système enrichi (fallback)
            response = self._generate_enriched_response(texte_lower)
            logger.info("Synthèse vocale (enrichi) : %s", response)

            # Appliquer émotion basique
            emotion = self._detect_emotion_from_text(texte_lower) or "neutral"

            # Parler avec synchronisation fine si disponible
            if dire_texte is not None:
                if self.emotional_sync:
                    self.emotional_sync.sync_speak_with_emotion(
                        response,
                        emotion=emotion,
                        intensity=0.6,
                        speak_function=dire_texte,
                    )
                else:
                    dire_texte(response, robot_api=self.robot_api)
                    if hasattr(self.robot_api, "set_emotion"):
                        self.robot_api.set_emotion(emotion, 0.6)
                    self._expressive_movement("nod")

        else:
            # Aucun texte entendu
            if dire_texte is not None:
                response = self._get_enriched_response("not_heard")
                dire_texte(response, robot_api=self.robot_api)
            logger.info("Aucun texte reconnu")

        return True

    def _generate_enriched_response(self, texte_lower: str) -> str:
        """Génère une réponse enrichie basée sur le texte reconnu."""
        if any(word in texte_lower for word in ["bonjour", "salut", "hello", "hi"]):
            return self._get_enriched_response("greeting")
        if any(word in texte_lower for word in ["comment", "ça va", "vas-tu"]):
            return self._get_enriched_response("how_are_you")
        if any(word in texte_lower for word in ["au revoir", "bye", "goodbye"]):
            return self._get_enriched_response("goodbye")
        if any(word in texte_lower for word in ["merci", "thanks", "thank you"]):
            return self._get_enriched_response("thanks")
        if any(word in texte_lower for word in ["super", "génial", "excellent"]):
            return self._get_enriched_response("positive")
        if "?" in texte_lower or any(
            word in texte_lower for word in ["qui", "quoi", "comment", "pourquoi"]
        ):
            return self._get_enriched_response("question")

        return self._get_enriched_response("default")

    def _detect_emotion_from_text(self, texte_lower: str) -> str | None:
        """Détecte une émotion basique depuis le texte."""
        if any(word in texte_lower for word in ["excité", "enthousiaste"]):
            return "excited"
        if any(word in texte_lower for word in ["super", "génial", "content"]):
            return "happy"
        if any(word in texte_lower for word in ["triste", "déçu"]):
            return "sad"
        if any(word in texte_lower for word in ["curieux", "intrigué"]):
            return "curious"
        if any(word in texte_lower for word in ["calme", "serein"]):
            return "calm"

        return "neutral"

    def _apply_sentiment_to_robot(self, sentiment: dict[str, Any]) -> None:
        """Applique le sentiment au robot."""
        if not self.robot_api or not hasattr(self.robot_api, "set_emotion"):
            return

        try:
            sentiment_label = sentiment.get("sentiment", "NEUTRAL")
            sentiment_score = sentiment.get("score", 0.5)

            emotion_mapping = {
                "POSITIVE": ("happy", min(sentiment_score, 0.9)),
                "NEGATIVE": ("sad", min(sentiment_score, 0.7)),
                "NEUTRAL": ("neutral", 0.5),
            }

            emotion, intensity = emotion_mapping.get(sentiment_label, ("neutral", 0.5))
            self.robot_api.set_emotion(emotion, intensity)
            logger.info("Émotion appliquée via sentiment : %s", emotion)

        except Exception as e:
            logger.warning("Erreur application sentiment : %s", e)

    def _expressive_movement(self, movement_type: str) -> None:
        """Applique un mouvement expressif.

        Args:
            movement_type: Type de mouvement (nod, shake)

        """
        if not self.robot_api:
            return

        try:
            if movement_type == "nod" and hasattr(self.robot_api, "goto_target"):
                # Hochement de tête
                self.robot_api.goto_target(
                    body_yaw=0.1,
                    duration=0.3,
                    method="minjerk",
                )
                import time

                time.sleep(0.4)
                self.robot_api.goto_target(
                    body_yaw=-0.1,
                    duration=0.3,
                    method="minjerk",
                )
                time.sleep(0.4)
                self.robot_api.goto_target(
                    body_yaw=0.0,
                    duration=0.3,
                    method="minjerk",
                )

        except Exception as e:
            logger.warning("Erreur mouvement expressif: %s", e)
