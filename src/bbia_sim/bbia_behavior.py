#!/usr/bin/env python3

"""BBIA Behavior Manager - Module de gestion des comportements.

Module de gestion des comportements pour Reachy Mini Wireless.

Comportements personnalisés, réactions automatiques, intégration avec les émotions.
"""

from __future__ import annotations

import logging
import secrets
import threading
import time
from pathlib import Path
from queue import Queue
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .robot_api import RobotAPI

try:
    from .bbia_emotions import BBIAEmotions
    from .bbia_vision import BBIAVision
    from .bbia_voice import dire_texte, reconnaitre_parole
except ImportError:
    # Pour les tests directs
    pass

# Import SDK officiel pour create_head_pose (optionnel)
try:
    from reachy_mini.utils import create_head_pose

    REACHY_MINI_UTILS_AVAILABLE = True
except ImportError:
    REACHY_MINI_UTILS_AVAILABLE = False
    create_head_pose = None

# Création du dossier log si besoin (préférence utilisateur)
Path("log").mkdir(parents=True, exist_ok=True)
# Logger BBIA strictement comme le test minimal

logger = logging.getLogger("BBIA")
# Supprime tous les handlers existants
for h in list(logger.handlers):
    logger.removeHandler(h)
handler = logging.FileHandler("log/bbia.log", mode="a", encoding="utf-8")
formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)
logger.setLevel(logging.INFO)
logger.info("--- Initialisation du logger BBIA ---")


class BBIABehavior:
    """Comportement de base pour BBIA."""

    def __init__(
        self,
        name: str,
        description: str,
        robot_api: RobotAPI | None = None,
    ) -> None:
        """Initialise un comportement.

        Args:
            name: Nom du comportement
            description: Description du comportement
            robot_api: Instance RobotAPI pour contrôler le robot (optionnel)

        """
        self.name = name
        self.description = description
        self.is_active = False
        self.priority = 1  # 1-10, 10 étant le plus prioritaire
        self.robot_api = robot_api  # Référence au robot pour contrôler les mouvements

    def can_execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        """Vérifie si le comportement peut être exécuté."""
        logger.info("Vérification d'exécution du comportement : %s", self.name)
        return True

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        """Exécute le comportement."""
        logger.info("Exécution du comportement : %s", self.name)
        return True

    def stop(self) -> None:
        """Arrête le comportement."""
        logger.info("Arrêt du comportement : %s", self.name)
        self.is_active = False


class WakeUpBehavior(BBIABehavior):
    """Comportement de réveil de BBIA conforme au SDK Reachy Mini officiel."""

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        super().__init__(
            "wake_up",
            "Séquence de réveil complète de BBIA",
            robot_api=robot_api,
        )
        self.priority = 10

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        logger.info("Début de la séquence de réveil BBIA")

        # Si robot_api disponible, utiliser run_behavior du SDK officiel
        if self.robot_api and hasattr(self.robot_api, "run_behavior"):
            try:
                logger.info("Exécution wake_up via SDK officiel")
                success = self.robot_api.run_behavior("wake_up", duration=5.0)
                if success:
                    logger.info("Réveil via SDK réussi")
                    # AMÉLIORATION INTELLIGENCE: Messages de réveil variés, personnels et expressifs
                    wake_messages = [
                        "Bonjour Athalia ! Je suis là, prêt à interagir avec vous.",
                        "Salut ! BBIA est réveillé et prêt à discuter !",
                        "Coucou Athalia ! Content de me réveiller à vos côtés.",
                        "Bonjour ! Je suis BBIA, votre robot compagnon. Comment allez-vous ?",
                        "Salut Athalia ! Je me sens bien et énergisé aujourd'hui !",
                        "Hello ! Je suis prêt pour une nouvelle journée avec vous !",
                        "Me voilà ! Je suis là pour vous accompagner, Athalia.",
                        "Bonjour ! Je me réveille avec enthousiasme pour passer du temps ensemble.",
                    ]
                    wake_message = secrets.choice(wake_messages)
                    # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
                    dire_texte(wake_message, robot_api=self.robot_api)
                    logger.info("Synthèse vocale : %s", wake_message)
                    return True
            except Exception as e:
                logger.error(f"Erreur wake_up SDK: {e}")

        # Fallback: Séquence manuelle conforme SDK (utilise create_head_pose et yaw_body)
        logger.info(
            "Étape : Mouvement de tête vers position neutre puis légèrement relevée",
        )
        if self.robot_api and hasattr(self.robot_api, "set_emotion"):
            self.robot_api.set_emotion("neutral", 0.3)
            time.sleep(0.5)
            self.robot_api.set_emotion(
                "happy",
                0.5,
            )  # Conforme SDK: pitch=0.1 * intensity

        time.sleep(1)

        logger.info("Étape : Rotation corps subtile avec interpolation fluide (réveil)")
        # OPTIMISATION EXPERT: Utiliser goto_target au lieu de set_joint_pos répétés
        # pour mouvement fluide avec interpolation automatique (minjerk)
        if self.robot_api:
            try:
                if (
                    hasattr(self.robot_api, "goto_target")
                    and REACHY_MINI_UTILS_AVAILABLE
                ):
                    # Mouvement fluide avec interpolation SDK (meilleure performance)
                    self.robot_api.goto_target(
                        body_yaw=0.15,
                        duration=0.8,
                        method="minjerk",
                    )
                    time.sleep(1.0)
                    self.robot_api.goto_target(
                        body_yaw=-0.15,
                        duration=0.8,
                        method="minjerk",
                    )
                    time.sleep(1.0)
                    self.robot_api.goto_target(
                        body_yaw=0.0,
                        duration=0.8,
                        method="minjerk",
                    )
                elif hasattr(self.robot_api, "set_joint_pos"):
                    # Fallback: Rotation subtile conforme limites (max 0.3 rad safe)
                    self.robot_api.set_joint_pos("yaw_body", 0.15)
                    time.sleep(0.5)
                    self.robot_api.set_joint_pos("yaw_body", -0.15)
                    time.sleep(0.5)
                    self.robot_api.set_joint_pos("yaw_body", 0.0)
            except Exception as e:
                logger.warning(f"Erreur mouvement corps réveil (continuation): {e}")

        time.sleep(1)

        logger.info("Étape : Première parole d'éveil")
        # AMÉLIORATION INTELLIGENCE: Messages de réveil plus variés, personnels et expressifs
        wake_messages = [
            "Bonjour Athalia ! Je suis là, prêt à interagir avec vous.",
            "Salut ! BBIA est réveillé et prêt à discuter !",
            "Coucou Athalia ! Content de me réveiller à vos côtés.",
            "Bonjour ! Je suis BBIA, votre robot compagnon. Comment allez-vous ?",
            "Salut Athalia ! Je me sens bien et énergisé aujourd'hui !",
            "Hello ! Je suis prêt pour une nouvelle journée avec vous !",
            "Me voilà ! Je suis là pour vous accompagner, Athalia.",
            "Bonjour ! Je me réveille avec enthousiasme pour passer du temps ensemble.",
        ]
        wake_message = secrets.choice(wake_messages)
        # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
        dire_texte(wake_message, robot_api=self.robot_api)
        logger.info(f"Synthèse vocale : {wake_message}")

        logger.info("Fin de la séquence de réveil BBIA")
        return True


class GreetingBehavior(BBIABehavior):
    """Comportement de salutation conforme au SDK Reachy Mini officiel."""

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        super().__init__("greeting", "Salutation personnalisée", robot_api=robot_api)
        # AMÉLIORATION INTELLIGENCE: Salutations plus variées et naturelles
        # Mélange de formel et décontracté selon le contexte
        self.greetings = [
            "Bonjour ! Comment allez-vous aujourd'hui ?",
            "Salut ! Ravi de vous retrouver !",
            "Hello ! Belle journée, non ?",
            "Bonjour ! Je suis BBIA, enchanté de vous voir !",
            "Coucou ! Ça va bien ?",
            "Salut ! Qu'est-ce qui vous amène ?",
            "Bonjour ! Heureux de vous revoir !",
            "Hey ! Ça fait plaisir de vous voir !",
            "Bonjour ! Quoi de neuf ?",
            "Salut ! Prêt pour discuter ?",
        ]

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        greeting = secrets.choice(self.greetings)
        logger.info(f"Salutation choisie : {greeting}")

        # Appliquer émotion "happy" conforme SDK (pitch=0.1 * intensity)
        if self.robot_api and hasattr(self.robot_api, "set_emotion"):
            self.robot_api.set_emotion("happy", 0.6)

        # OPTIMISATION EXPERT: Utiliser goto_target pour mouvement fluide avec interpolation
        # au lieu de set_target_head_pose répétés (évite mouvements saccadés)
        if self.robot_api and (REACHY_MINI_UTILS_AVAILABLE and create_head_pose):
            try:
                # Méthode 1 (recommandée): goto_target avec interpolation fluide (minjerk)
                if hasattr(self.robot_api, "goto_target"):
                    pose = create_head_pose(pitch=0.08, yaw=0.0, degrees=False)
                    self.robot_api.goto_target(
                        head=pose,
                        duration=0.8,
                        method="minjerk",
                    )
                    time.sleep(0.9)
                    pose_neutral = create_head_pose(pitch=0.0, yaw=0.0, degrees=False)
                    self.robot_api.goto_target(
                        head=pose_neutral,
                        duration=0.8,
                        method="minjerk",
                    )
                # Méthode 2 (fallback): set_target_head_pose direct
                elif hasattr(self.robot_api, "set_target_head_pose"):
                    pose = create_head_pose(pitch=0.08, yaw=0.0, degrees=False)
                    self.robot_api.set_target_head_pose(pose)
                    time.sleep(0.8)
                    pose_neutral = create_head_pose(pitch=0.0, yaw=0.0, degrees=False)
                    self.robot_api.set_target_head_pose(pose_neutral)
            except Exception as e:
                logger.warning(f"Erreur pose tête salutation (fallback): {e}")
                # Fallback final: rotation corps subtile
                if hasattr(self.robot_api, "set_joint_pos"):
                    try:
                        self.robot_api.set_joint_pos("yaw_body", 0.12)
                        time.sleep(0.5)
                        self.robot_api.set_joint_pos("yaw_body", 0.0)
                    except Exception as e2:
                        logger.error(f"Erreur fallback rotation corps: {e2}")
        elif self.robot_api and hasattr(self.robot_api, "set_joint_pos"):
            # Fallback: rotation corps subtile si SDK non disponible
            try:
                self.robot_api.set_joint_pos("yaw_body", 0.12)
                time.sleep(0.5)
                self.robot_api.set_joint_pos("yaw_body", 0.0)
            except Exception as e:
                logger.error(f"Erreur fallback rotation corps: {e}")

        # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
        dire_texte(greeting, robot_api=self.robot_api)
        logger.info(f"Synthèse vocale : {greeting}")
        return True


class EmotionalResponseBehavior(BBIABehavior):
    """Comportement de réponse émotionnelle conforme au SDK Reachy Mini officiel."""

    def __init__(
        self, emotions: BBIAEmotions, robot_api: RobotAPI | None = None
    ) -> None:
        super().__init__(
            "emotional_response",
            "Réponse émotionnelle automatique",
            robot_api=robot_api,
        )
        self.emotions = emotions
        self.priority = 8

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        stimulus = context.get("stimulus", "")
        logger.info(f"Stimulus reçu pour réponse émotionnelle : {stimulus}")
        if stimulus:
            emotion = self.emotions.emotional_response(stimulus)
            logger.info(f"Réponse émotionnelle générée : {emotion}")

            # Appliquer l'émotion au robot via SDK officiel
            sdk_emotion = emotion
            if self.robot_api and hasattr(self.robot_api, "set_emotion"):
                # Mapper vers les 6 émotions SDK officiel si nécessaire
                sdk_emotions = {"happy", "sad", "neutral", "excited", "curious", "calm"}
                if emotion in sdk_emotions:
                    sdk_emotion = emotion
                    self.robot_api.set_emotion(emotion, 0.7)
                else:
                    # Mapper les autres émotions BBIA vers les émotions SDK
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
                    self.robot_api.set_emotion(sdk_emotion, 0.7)

            # AMÉLIORATION INTELLIGENCE: Commentaires vocaux variés selon l'émotion
            # Pour rendre les réactions plus expressives et moins "robotiques"
            emotion_comments = {
                "happy": [
                    "Ça me fait plaisir !",
                    "C'est super !",
                    "Oh, c'est gentil !",
                    "Je suis content !",
                ],
                "excited": [
                    "Wow, c'est excitant !",
                    "Quelle bonne nouvelle !",
                    "Fantastique !",
                    "C'est génial !",
                ],
                "curious": [
                    "Hmm, c'est intéressant...",
                    "Intriguant !",
                    "Je me demande...",
                    "Cela m'intrigue.",
                ],
                "sad": [
                    "Oh, je comprends...",
                    "C'est dommage.",
                    "Je compatis.",
                    "Ça m'attriste un peu.",
                ],
                "calm": [
                    "Je comprends.",
                    "C'est bien ainsi.",
                    "D'accord.",
                    "Tout va bien.",
                ],
                "neutral": [
                    "Je vois.",
                    "D'accord.",
                    "Compris.",
                ],
            }

            # Sélectionner un commentaire varié selon l'émotion SDK appliquée
            comments = emotion_comments.get(sdk_emotion, emotion_comments["neutral"])
            if comments:
                comment = secrets.choice(comments)
                # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
                dire_texte(comment, robot_api=self.robot_api)
                logger.info(f"Synthèse vocale (émotion) : {comment}")

            return True
        logger.info("Aucun stimulus fourni pour la réponse émotionnelle")
        return False


class VisionTrackingBehavior(BBIABehavior):
    """Comportement de suivi visuel conforme au SDK Reachy Mini officiel."""

    def __init__(self, vision: BBIAVision, robot_api: RobotAPI | None = None) -> None:
        super().__init__(
            "vision_tracking",
            "Suivi visuel d'objets",
            robot_api=robot_api,
        )
        self.vision = vision
        self.priority = 6

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        logger.info("Activation du suivi visuel")

        result = self.vision.scan_environment()
        logger.info(f"Résultat du scan environnement : {result}")

        if result["objects"]:
            first_object = result["objects"][0]
            object_name = first_object.get("name", "quelque chose")
            logger.info(f"Suivi de l'objet : {object_name}")
            self.vision.track_object(first_object["name"])

            # AMÉLIORATION INTELLIGENCE: Commentaires vocaux variés lors de détection
            detection_comments = [
                f"Je vois {object_name} !",
                f"Oh, il y a {object_name} là-bas !",
                f"Je regarde {object_name}.",
                f"{object_name} m'intrigue !",
                f"Intéressant, je vois {object_name}.",
            ]
            comment = secrets.choice(detection_comments)
            # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
            dire_texte(comment, robot_api=self.robot_api)
            logger.info(f"Synthèse vocale (vision) : {comment}")

            # OPTIMISATION EXPERT: Utiliser look_at_world/look_at_image avec gestion d'erreur robuste
            if self.robot_api:
                try:
                    # Méthode 1 (préférée): look_at_world si position 3D disponible
                    if hasattr(self.robot_api, "look_at_world"):
                        pos = first_object.get("position", {})
                        if pos:
                            x = float(pos.get("x", 0.2))
                            y = float(pos.get("y", 0.1))
                            z = float(pos.get("z", 0.0))
                            # Validation des coordonnées (éviter valeurs extrêmes)
                            if (
                                -2.0 <= x <= 2.0
                                and -2.0 <= y <= 2.0
                                and -1.0 <= z <= 1.0
                            ):
                                self.robot_api.look_at_world(
                                    x,
                                    y,
                                    z,
                                    duration=1.0,
                                    perform_movement=True,
                                )
                                logger.info(
                                    f"Look_at_world vers position 3D: ({x:.2f}, {y:.2f}, {z:.2f})",
                                )
                            else:
                                logger.warning(
                                    f"Coordonnées 3D hors limites: ({x}, {y}, {z})",
                                )
                                raise ValueError("Coordonnées hors limites")
                        else:
                            raise ValueError("Pas de position 3D disponible")
                    # Méthode 2: look_at_image si position 2D disponible
                    elif hasattr(self.robot_api, "look_at_image"):
                        bbox = first_object.get("bbox", {})
                        if bbox:
                            u = int(bbox.get("center_x", 320))
                            v = int(bbox.get("center_y", 240))
                            # Validation coordonnées image (éviter hors cadre)
                            if 0 <= u <= 640 and 0 <= v <= 480:
                                self.robot_api.look_at_image(u, v, duration=1.0)
                                logger.info(f"Look_at_image vers pixel: ({u}, {v})")
                            else:
                                logger.warning(
                                    f"Coordonnées image hors limites: ({u}, {v})",
                                )
                                raise ValueError("Coordonnées image hors limites")
                        else:
                            raise ValueError("Pas de bbox disponible")
                    # Méthode 3 (fallback): émotion curious (regard explorateur)
                    elif hasattr(self.robot_api, "set_emotion"):
                        self.robot_api.set_emotion("curious", 0.6)
                        logger.info("Fallback: émotion curious appliquée")
                except Exception as e:
                    logger.error(f"Erreur suivi visuel (fallback émotion): {e}")
                    # Fallback final: émotion curious
                    if hasattr(self.robot_api, "set_emotion"):
                        try:
                            self.robot_api.set_emotion("curious", 0.6)
                        except Exception as e2:
                            logger.error(f"Erreur même avec fallback émotion: {e2}")

            return True

        # Aucun objet détecté - appliquer émotion "curious" et commentaire varié
        if self.robot_api and hasattr(self.robot_api, "set_emotion"):
            self.robot_api.set_emotion("curious", 0.6)

        # AMÉLIORATION INTELLIGENCE: Messages variés quand aucun objet détecté
        no_object_comments = [
            "Je ne vois rien d'intéressant pour l'instant.",
            "Rien de nouveau dans mon champ de vision.",
            "Je cherche, mais je ne vois rien de particulier.",
            "Mon environnement semble vide pour le moment.",
            "Aucun objet détecté autour de moi.",
        ]
        comment = secrets.choice(no_object_comments)
        # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
        dire_texte(comment, robot_api=self.robot_api)
        logger.info(f"Synthèse vocale (vision, aucun objet) : {comment}")
        logger.info("Aucun objet détecté pour le suivi visuel")
        return False


class ConversationBehavior(BBIABehavior):
    """Comportement de conversation intelligent conforme au SDK Reachy Mini officiel.

    Utilise BBIAHuggingFace si disponible pour des réponses intelligentes avec analyse
    de sentiment, sinon utilise un système de réponses enrichies avec plus de variété
    et de personnalité.
    """

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        super().__init__(
            "conversation",
            "Conversation interactive intelligente",
            robot_api=robot_api,
        )
        self.priority = 7

        # Tentative d'importation de BBIAHuggingFace (optionnel)
        self.hf_chat = None
        try:
            from .bbia_huggingface import BBIAHuggingFace

            self.hf_chat = BBIAHuggingFace()
            logger.info(
                "✅ BBIAHuggingFace disponible - Conversation intelligente activée",
            )
        except (ImportError, Exception) as e:
            logger.info(f"ℹ️  BBIAHuggingFace non disponible - Mode enrichi activé: {e}")
            self.hf_chat = None

        # Système de réponses enrichies (fallback)
        self.enriched_responses = self._init_enriched_responses()

    def _init_enriched_responses(self) -> dict[str, list[str]]:
        """Initialise les réponses enrichies avec variété."""
        return {
            "greeting": [
                "Bonjour ! Je suis ravi de vous entendre. Comment allez-vous aujourd'hui ?",
                "Salut ! Ça fait plaisir de vous voir ! Qu'est-ce qui vous amène ?",
                "Hello ! Je suis BBIA, votre compagnon robotique. Que puis-je faire pour vous ?",
                "Bonjour ! Prêt pour une belle conversation ? Dites-moi ce qui vous passe par la tête.",
                "Salut ! Content de vous revoir. Comment s'est passée votre journée ?",
                "Bonjour ! Je suis tout ouïe. Qu'est-ce que vous aimeriez partager ?",
            ],
            "how_are_you": [
                "Je vais très bien, merci de demander ! Et vous ?",
                "Excellente journée, je suis en pleine forme !",
                "Tout va bien, je suis content d'être là.",
                "Parfaitement ! Merci beaucoup pour votre attention.",
            ],
            "goodbye": [
                "Au revoir ! Ce fut un plaisir de discuter.",
                "À bientôt ! N'hésitez pas à revenir me voir.",
                "Au revoir ! Prenez bien soin de vous.",
                "À très bientôt ! J'ai adoré notre conversation.",
            ],
            "thanks": [
                "De rien ! C'est un plaisir de vous aider.",
                "Je vous en prie ! Tout le plaisir est pour moi.",
                "Pas de souci ! N'hésitez pas si vous avez d'autres questions.",
                "Avec plaisir ! Je suis toujours là pour vous.",
            ],
            "positive": [
                "C'est super ! Je suis content pour vous.",
                "Fantastique ! Continuez comme ça, vous faites du bon travail.",
                "Excellent ! C'est une bonne nouvelle.",
                "Génial ! Je partage votre enthousiasme.",
            ],
            "question": [
                "C'est une excellente question ! Laissez-moi réfléchir...",
                "Hmm, intéressant ! Voici ce que je pense :",
                "Bonne question ! Voici comment je vois les choses :",
                "Je comprends votre interrogation. Voici mon avis :",
            ],
            "default": [
                "C'est intéressant ! Pouvez-vous m'en dire plus ? J'aimerais mieux comprendre votre point de vue.",
                "Je comprends. Continuez, je vous écoute attentivement. N'hésitez pas à développer.",
                "Ah, c'est fascinant ! Racontez-moi davantage, ça m'intrigue.",
                "Je vois. C'est un sujet qui mérite réflexion. Que pensez-vous vous-même de ça ?",
                "Hmm, intriguant ! Qu'est-ce qui vous a amené à penser à ça ?",
                "Intéressant ! Je suis curieux d'en apprendre plus. Partagez-moi vos réflexions.",
                "Je comprends votre perspective. Voulez-vous approfondir ce sujet ensemble ?",
            ],
            "not_heard": [
                "Je n'ai pas bien entendu. Pouvez-vous répéter s'il vous plaît ?",
                "Pardon, pouvez-vous répéter ? Je n'ai pas saisi.",
                "Je n'ai rien entendu. Pouvez-vous parler un peu plus fort ?",
                "Désolé, je n'ai pas compris. Pouvez-vous reformuler ?",
                "Excusez-moi, je n'ai pas capté. Pourriez-vous répéter ?",
                "Je n'ai pas bien saisi. Peux-tu redire ?",
            ],
        }

    def _get_enriched_response(self, category: str) -> str:
        """Récupère une réponse enrichie aléatoire dans une catégorie."""

        responses = self.enriched_responses.get(
            category,
            self.enriched_responses["default"],
        )
        return secrets.choice(responses)  # nosec B311

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        """Exécute une conversation intelligente avec reconnaissance vocale."""
        logger.info("Activation du mode conversation intelligente")

        # Message d'accueil avec variété
        greeting_messages = [
            "Je vous écoute attentivement.",
            "Je suis tout ouïe, parlez-moi.",
            "Dites-moi ce qui vous passe par la tête.",
            "Je suis là pour vous, que souhaitez-vous me dire ?",
        ]

        greeting = secrets.choice(greeting_messages)  # nosec B311
        # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
        dire_texte(greeting, robot_api=self.robot_api)
        logger.info(f"Synthèse vocale : {greeting}")

        # OPTIMISATION SDK: Reconnaissance vocale via robot.media.microphone si disponible
        # Bénéfice: 4 microphones directionnels avec annulation de bruit automatique
        texte = reconnaitre_parole(duree=5, robot_api=self.robot_api)
        logger.info(f"Texte reconnu : {texte}")

        if texte:
            texte_lower = texte.lower()

            # Utiliser BBIAHuggingFace si disponible (intelligence avancée)
            if self.hf_chat:
                try:
                    response = self.hf_chat.chat(texte)
                    # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
                    dire_texte(response, robot_api=self.robot_api)
                    logger.info(f"Synthèse vocale (HF) : {response}")

                    # Appliquer émotion correspondante au robot si disponible
                    sentiment = self.hf_chat.analyze_sentiment(texte)
                    self._apply_sentiment_to_robot(sentiment)

                    return True
                except Exception as e:
                    logger.warning(f"Erreur BBIAHuggingFace, fallback enrichi : {e}")
                    # Fallback vers système enrichi

            # Système enrichi (fallback ou si HF non disponible)
            response = self._generate_enriched_response(texte_lower)
            # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
            dire_texte(response, robot_api=self.robot_api)
            logger.info(f"Synthèse vocale (enrichi) : {response}")

            # Appliquer émotion basique si robot_api disponible
            emotion = self._detect_emotion_from_text(texte_lower)
            if emotion and self.robot_api:
                try:
                    self.robot_api.set_emotion(emotion, 0.6)
                    logger.info(f"Émotion appliquée au robot : {emotion}")
                except Exception as e:
                    logger.warning(f"Erreur application émotion : {e}")

        else:
            # Aucun texte entendu
            response = self._get_enriched_response("not_heard")
            # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
            dire_texte(response, robot_api=self.robot_api)
            logger.info(f"Synthèse vocale : {response}")

        return True

    def _generate_enriched_response(self, texte_lower: str) -> str:
        """Génère une réponse enrichie basée sur le texte reconnu."""
        # Salutations
        if any(
            word in texte_lower for word in ["bonjour", "salut", "hello", "hi", "hey"]
        ):
            return self._get_enriched_response("greeting")

        # Comment allez-vous
        if any(
            word in texte_lower
            for word in ["comment", "comment ça va", "ça va", "vas-tu"]
        ):
            return self._get_enriched_response("how_are_you")

        # Au revoir
        if any(
            word in texte_lower
            for word in ["au revoir", "bye", "goodbye", "à bientôt", "adieu"]
        ):
            return self._get_enriched_response("goodbye")

        # Remerciements
        if any(
            word in texte_lower
            for word in ["merci", "thanks", "thank you", "merci beaucoup"]
        ):
            return self._get_enriched_response("thanks")

        # Positif
        if any(
            word in texte_lower
            for word in ["super", "génial", "excellent", "cool", "content", "heureux"]
        ):
            return self._get_enriched_response("positive")

        # Questions
        if "?" in texte_lower or any(
            word in texte_lower
            for word in ["qui", "quoi", "comment", "pourquoi", "où", "quand"]
        ):
            return self._get_enriched_response("question")

        # Défaut enrichi
        return self._get_enriched_response("default")

    def _detect_emotion_from_text(self, texte_lower: str) -> str | None:
        """Détecte une émotion basique depuis le texte."""
        # Mapping simple texte → émotion SDK officielle
        # IMPORTANT: Vérifier "excited" AVANT "happy" pour éviter conflit (excité peut être dans les deux)
        if any(
            word in texte_lower for word in ["excité", "enthousiaste", "euphorique"]
        ):
            return "excited"
        elif any(
            word in texte_lower for word in ["super", "génial", "content", "heureux"]
        ):
            return "happy"
        elif any(word in texte_lower for word in ["triste", "déçu", "malheureux"]):
            return "sad"
        elif any(
            word in texte_lower for word in ["curieux", "intrigué", "intéressant"]
        ):
            return "curious"
        elif any(word in texte_lower for word in ["calme", "serein", "détendu"]):
            return "calm"

        return "neutral"

    def _apply_sentiment_to_robot(self, sentiment: dict[str, Any]) -> None:
        """Applique le sentiment au robot via robot_api si disponible."""
        if not self.robot_api:
            return

        try:
            sentiment_label = sentiment.get("sentiment", "NEUTRAL")
            sentiment_score = sentiment.get("score", 0.5)

            # Mapping sentiment → émotion SDK officielle
            emotion_mapping = {
                "POSITIVE": ("happy", min(sentiment_score, 0.9)),
                "NEGATIVE": ("sad", min(sentiment_score, 0.7)),
                "NEUTRAL": ("neutral", 0.5),
            }

            emotion, intensity = emotion_mapping.get(sentiment_label, ("neutral", 0.5))

            if hasattr(self.robot_api, "set_emotion"):
                self.robot_api.set_emotion(emotion, intensity)
                logger.info(
                    f"Émotion appliquée via sentiment : {emotion} (intensité: {intensity:.2f})",
                )
        except Exception as e:
            logger.warning(f"Erreur application sentiment au robot : {e}")


class AntennaAnimationBehavior(BBIABehavior):
    """Comportement d'animation expressivité (alternative aux antennes).

    ✅ IMPORTANT: Les antennes sont maintenant ANIMABLES dans le modèle officiel (range=[-0.300, 0.300]).
    Ce comportement utilise les antennes avec limites de sécurité (-0.3 à 0.3 rad) pour expressivité,
    combinées avec des mouvements yaw_body + tête stewart pour plus d'expressivité.
    """

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        super().__init__(
            "antenna_animation",
            "Animation expressive selon l'émotion (antennes protégées)",
            robot_api=robot_api,
        )
        self.priority = 5

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        emotion = context.get("emotion", "neutral")
        logger.info(f"Animation expressive pour l'émotion : {emotion}")

        # Note: Les antennes sont interdites pour sécurité hardware.
        # Utiliser émotions SDK + mouvements tête/corps pour simuler l'expressivité
        if self.robot_api:
            # Appliquer l'émotion via SDK officiel
            if hasattr(self.robot_api, "set_emotion"):
                sdk_emotions = {"happy", "sad", "neutral", "excited", "curious", "calm"}
                if emotion in sdk_emotions:
                    self.robot_api.set_emotion(emotion, 0.8)
                else:
                    # Mapper vers SDK
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
                    self.robot_api.set_emotion(sdk_emotion, 0.8)

            # OPTIMISATION EXPERT: Utiliser goto_target pour mouvements fluides expressifs
            # au lieu de set_joint_pos répétés (meilleure fluidité et performance)
            movements = {
                "happy": 0.12,
                "excited": 0.15,
                "curious": 0.10,
                "sad": -0.08,
                "angry": 0.0,
                "neutral": 0.0,
            }
            yaw = movements.get(emotion, 0.0)
            if yaw != 0.0:
                try:
                    # Méthode 1 (préférée): goto_target avec interpolation fluide
                    if hasattr(self.robot_api, "goto_target"):
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
                    # Méthode 2 (fallback): set_joint_pos direct
                    elif hasattr(self.robot_api, "set_joint_pos"):
                        self.robot_api.set_joint_pos("yaw_body", yaw)
                        time.sleep(0.6)
                        self.robot_api.set_joint_pos("yaw_body", 0.0)
                except Exception as e:
                    logger.warning(f"Erreur mouvement expressif {emotion}: {e}")

        logger.info(f"Animation expressive appliquée pour : {emotion}")
        time.sleep(1)
        return True


class HideBehavior(BBIABehavior):
    """Comportement 'se cacher' (hide) conforme au SDK Reachy Mini officiel.

    Simule le repli en baissant la tête (pitch négatif) et rotation corps vers l'arrière.
    Les antennes ne sont PAS contrôlées (protégées pour sécurité hardware).
    """

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        super().__init__(
            "hide",
            "Se cacher : tête baissée, corps replié (antennes non contrôlées)",
            robot_api=robot_api,
        )
        self.priority = 9

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        print("🙈 [BBIA] Séquence 'se cacher'...")
        logger.info("Début de la séquence 'se cacher'")

        # Utiliser émotion "sad" conforme SDK (pitch=-0.1 * intensity)
        print("🤖 Tête qui s'abaisse lentement...")
        logger.info("Étape : Tête qui s'abaisse lentement")
        if self.robot_api and hasattr(self.robot_api, "set_emotion"):
            # Émotion sad avec intensité forte = pitch négatif (tête baissée)
            self.robot_api.set_emotion("sad", 0.9)  # Conforme SDK: pitch=-0.09 rad

        # Note: Les antennes ne sont pas contrôlées (protégées pour sécurité hardware)
        print("📡 Antennes qui se replient devant le visage...")
        logger.info("Étape : Antennes (simulation - non contrôlées pour sécurité)")

        print("👁️ Yeux qui se ferment (ou s'éteignent)...")
        logger.info("Étape : Yeux qui se ferment")

        # OPTIMISATION EXPERT: Utiliser goto_target pour mouvement combiné tête+corps fluide
        # au lieu de mouvements séparés (meilleure synchronisation)
        if self.robot_api:
            try:
                if (
                    REACHY_MINI_UTILS_AVAILABLE
                    and create_head_pose
                    and hasattr(self.robot_api, "goto_target")
                ):
                    # Mouvement combiné tête baissée + corps replié avec interpolation
                    pose_down = create_head_pose(pitch=-0.15, yaw=0.0, degrees=False)
                    self.robot_api.goto_target(
                        head=pose_down,
                        body_yaw=-0.2,  # Conforme limites safe
                        duration=1.2,  # Durée pour mouvement fluide
                        method="minjerk",
                    )
                else:
                    # Fallback: mouvements séparés
                    # Rotation corps vers l'arrière (repli)
                    if hasattr(self.robot_api, "set_joint_pos"):
                        self.robot_api.set_joint_pos(
                            "yaw_body",
                            -0.2,
                        )  # Conforme limites safe

                    time.sleep(1.5)

                    # Baisser encore plus la tête si possible
                    if REACHY_MINI_UTILS_AVAILABLE and create_head_pose:
                        if hasattr(self.robot_api, "set_target_head_pose"):
                            # Pose tête baissée (conforme SDK sad: pitch=-0.1, ici plus fort)
                            pose = create_head_pose(pitch=-0.15, yaw=0.0, degrees=False)
                            self.robot_api.set_target_head_pose(pose)
            except Exception as e:
                logger.warning(f"Erreur mouvement hide (continuation): {e}")

        time.sleep(1.0)

        print("💤 BBIA se cache et devient silencieux.")
        logger.info("Étape : BBIA se cache et devient silencieux")
        print("(BBIA attend discrètement...)")
        # AMÉLIORATION INTELLIGENCE: Messages de cache plus variés et expressifs
        hide_messages = [
            "Je me cache... Chut !",
            "Oups, je me fais discret...",
            "Je me replie de façon discret, c'est normal.",
            "Je me fais tout petit et discret...",
            "Je me cache un instant...",
        ]
        hide_message = secrets.choice(hide_messages)
        # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
        dire_texte(hide_message, robot_api=self.robot_api)
        logger.info(f"Synthèse vocale : {hide_message}")
        time.sleep(1)
        logger.info("Fin de la séquence 'se cacher'")
        return True


class BBIABehaviorManager:
    """Gestionnaire de comportements pour BBIA."""

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        """Initialise le gestionnaire de comportements.

        Args:
            robot_api: Instance RobotAPI pour contrôler le robot (optionnel).
                      Si None, les comportements fonctionneront en mode simulation/logs uniquement.

        """
        self.behaviors: dict[str, BBIABehavior] = {}
        self.active_behaviors: list[str] = []
        self.behavior_queue: Queue[tuple[str, dict[str, Any]]] = Queue()
        self.is_running = False
        self.worker_thread: threading.Thread | None = None

        # RobotAPI pour contrôler le robot physique
        self.robot_api = robot_api

        # OPTIMISATION PERFORMANCE: Bibliothèque de mouvements enregistrés pour réutilisation
        self.saved_moves: dict[str, Any] = {}

        # Initialiser les modules BBIA
        self.emotions = BBIAEmotions()
        self.vision = BBIAVision(
            robot_api=robot_api,
        )  # Passer robot_api pour camera SDK

        # Enregistrer les comportements par défaut
        self._register_default_behaviors()

    def _register_default_behaviors(self) -> None:
        """Enregistre les comportements par défaut."""
        self.register_behavior(WakeUpBehavior(robot_api=self.robot_api))
        self.register_behavior(GreetingBehavior(robot_api=self.robot_api))
        self.register_behavior(
            EmotionalResponseBehavior(self.emotions, robot_api=self.robot_api),
        )
        self.register_behavior(
            VisionTrackingBehavior(self.vision, robot_api=self.robot_api),
        )
        self.register_behavior(ConversationBehavior(robot_api=self.robot_api))
        self.register_behavior(AntennaAnimationBehavior(robot_api=self.robot_api))
        self.register_behavior(HideBehavior(robot_api=self.robot_api))

    def register_behavior(self, behavior: BBIABehavior) -> None:
        """Enregistre un nouveau comportement."""
        self.behaviors[behavior.name] = behavior

    def execute_behavior(
        self,
        behavior_name: str,
        context: dict[str, Any] | None = None,
    ) -> bool:
        """Exécute un comportement spécifique."""
        if behavior_name not in self.behaviors:
            return False

        behavior = self.behaviors[behavior_name]
        context = context or {}

        if behavior.can_execute(context):
            return behavior.execute(context)
        else:
            return False

    def add_to_queue(
        self,
        behavior_name: str,
        context: dict[str, Any] | None = None,
    ) -> None:
        """Ajoute un comportement à la queue d'exécution."""
        self.behavior_queue.put((behavior_name, context or {}))

    def start_behavior_worker(self) -> None:
        """Démarre le worker de comportements."""
        if self.is_running:
            return

        self.is_running = True
        thread = threading.Thread(target=self._behavior_worker, daemon=True)
        self.worker_thread = thread
        thread.start()

    def stop_behavior_worker(self) -> None:
        """Arrête le worker de comportements."""
        self.is_running = False
        if self.worker_thread:
            # Ajouter un timeout pour éviter blocage infini
            self.worker_thread.join(timeout=5.0)
            if self.worker_thread.is_alive():
                logger.warning(
                    "Worker thread n'a pas pu être arrêté dans les 5 secondes",
                )

    def _behavior_worker(self) -> None:
        """Worker qui traite la queue de comportements."""
        while self.is_running:
            try:
                if not self.behavior_queue.empty():
                    behavior_name, context = self.behavior_queue.get(timeout=1)
                    self.execute_behavior(behavior_name, context)
                else:
                    time.sleep(0.1)
            except Exception as e:
                logger.error(f"Erreur dans le worker de comportements : {e}")
                time.sleep(0.1)  # Éviter une boucle infinie en cas d'erreur

    def get_available_behaviors(self) -> list[dict[str, Any]]:
        """Retourne la liste des comportements disponibles."""
        return [
            {
                "name": behavior.name,
                "description": behavior.description,
                "priority": behavior.priority,
                "is_active": behavior.is_active,
            }
            for behavior in self.behaviors.values()
        ]

    def get_behavior_stats(self) -> dict[str, Any]:
        """Retourne les statistiques des comportements."""
        return {
            "total_behaviors": len(self.behaviors),
            "active_behaviors": len(self.active_behaviors),
            "queue_size": self.behavior_queue.qsize(),
            "worker_running": self.is_running,
            "available_behaviors": self.get_available_behaviors(),
            "saved_moves_count": len(self.saved_moves),
        }

    # ===== MÉTHODES ENREGISTREMENT/REPLAY POUR PERFORMANCES =====

    def record_behavior_movement(
        self,
        behavior_name: str,
        duration: float = 3.0,
    ) -> object | None:
        """Enregistre un mouvement de comportement pour réutilisation (performance).

        Args:
            behavior_name: Nom du comportement à enregistrer
            duration: Durée d'enregistrement en secondes

        Returns:
            Move enregistré ou None si erreur

        """
        if not self.robot_api or not hasattr(self.robot_api, "start_recording"):
            logger.warning(
                "Enregistrement non disponible (robot_api ou start_recording manquant)",
            )
            return None

        try:
            logger.info(
                f"🎬 Enregistrement mouvement comportement '{behavior_name}' ({duration}s)...",
            )
            self.robot_api.start_recording()

            # Exécuter le comportement pendant l'enregistrement
            context = {"duration": duration}
            self.execute_behavior(behavior_name, context)

            # Arrêter et récupérer le mouvement
            if not hasattr(self.robot_api, "stop_recording"):
                logger.warning("stop_recording non disponible")
                return None
            # Type narrowing: hasattr garantit que stop_recording existe
            move: object | None = self.robot_api.stop_recording()  # type: ignore[attr-defined]
            if move:
                self.saved_moves[behavior_name] = move
                logger.info(
                    f"✅ Mouvement '{behavior_name}' enregistré et sauvegardé "
                    f"({len(move) if isinstance(move, list) else 'N/A'} frames)",
                )
                return move
            else:
                logger.warning(f"⚠️  Aucun mouvement enregistré pour '{behavior_name}'")
                return None
        except Exception as e:
            logger.error(
                f"❌ Erreur enregistrement comportement '{behavior_name}': {e}",
            )
            return None

    def play_saved_behavior(self, behavior_name: str, use_async: bool = True) -> bool:
        """Rejoue un comportement enregistré (meilleure performance que réexécution).

        Args:
            behavior_name: Nom du comportement à rejouer
            use_async: Utiliser async_play_move (non bloquant) ou play_move (bloquant)

        Returns:
            True si rejoué avec succès

        """
        if behavior_name not in self.saved_moves:
            logger.warning(f"⚠️  Aucun mouvement sauvegardé pour '{behavior_name}'")
            return False

        if not self.robot_api:
            logger.warning("Robot API non disponible")
            return False

        try:
            move = self.saved_moves[behavior_name]

            # OPTIMISATION PERFORMANCE: Utiliser async_play_move si disponible (non bloquant)
            if use_async and hasattr(self.robot_api, "async_play_move"):
                self.robot_api.async_play_move(move, play_frequency=100.0)
                logger.info(
                    f"▶️  Rejoué '{behavior_name}' (async, non bloquant) - "
                    f"{len(move) if isinstance(move, list) else 'N/A'} frames",
                )
                return True
            elif hasattr(self.robot_api, "play_move"):
                self.robot_api.play_move(move, play_frequency=100.0)
                logger.info(
                    f"▶️  Rejoué '{behavior_name}' (sync, bloquant) - "
                    f"{len(move) if isinstance(move, list) else 'N/A'} frames",
                )
                return True
            else:
                logger.warning("play_move non disponible dans robot_api")
                return False
        except Exception as e:
            logger.error(f"❌ Erreur rejouant '{behavior_name}': {e}")
            return False

    def clear_saved_moves(self) -> None:
        """Efface tous les mouvements enregistrés."""
        self.saved_moves.clear()
        logger.info("🗑️  Tous les mouvements enregistrés ont été effacés")


def main() -> None:
    """Test du module BBIA Behavior Manager."""
    # Créer l'instance
    manager = BBIABehaviorManager()

    # Test 1 : Comportement de réveil
    manager.execute_behavior("wake_up")

    # Test 2 : Salutation
    manager.execute_behavior("greeting")

    # Test 3 : Animation des antennes
    context = {"emotion": "happy"}
    manager.execute_behavior("antenna_animation", context)

    # Test 4 : Réponse émotionnelle
    context = {"stimulus": "compliment"}
    manager.execute_behavior("emotional_response", context)

    # Test 5 : Suivi visuel
    manager.execute_behavior("vision_tracking")

    # Test 6 : Statistiques
    manager.get_behavior_stats()


if __name__ == "__main__":
    main()
    # Flush et fermeture explicite du handler pour garantir l'écriture du fichier de log
    for h in logger.handlers:
        h.flush()
        h.close()
