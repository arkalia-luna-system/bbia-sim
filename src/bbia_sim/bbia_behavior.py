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
from uuid import UUID, uuid4

if TYPE_CHECKING:
    from .bbia_emotions import BBIAEmotions
    from .bbia_vision import BBIAVision
    from .robot_api import RobotAPI

# Imports lourds (vision/voice/SDK) : jamais au chargement du module.
create_head_pose: Any = None
REACHY_MINI_UTILS_AVAILABLE = False
_SDK_UTILS_LOAD_ATTEMPTED = False
_BBIAEmotions: Any = None
_BBIAVision: Any = None
_get_bbia_vision_singleton: Any = None
_dire_texte: Any = None
_reconnaitre_parole: Any = None
_DEPS_LOAD_ATTEMPTED = False


def _ensure_behavior_deps() -> None:
    """Charge émotions/vision/voix à la demande uniquement."""
    global _BBIAEmotions, _BBIAVision, _get_bbia_vision_singleton
    global _dire_texte, _reconnaitre_parole, _DEPS_LOAD_ATTEMPTED
    if _DEPS_LOAD_ATTEMPTED:
        return
    _DEPS_LOAD_ATTEMPTED = True
    try:
        from .bbia_emotions import BBIAEmotions as _E

        _BBIAEmotions = _E
    except ImportError:
        _BBIAEmotions = None
    try:
        from .bbia_vision import BBIAVision as _V
        from .bbia_vision import get_bbia_vision_singleton as _G

        _BBIAVision = _V
        _get_bbia_vision_singleton = _G
    except ImportError:
        _BBIAVision = None
        _get_bbia_vision_singleton = None
    try:
        from .bbia_voice import dire_texte as _DT
        from .bbia_voice import reconnaitre_parole as _RP

        _dire_texte = _DT
        _reconnaitre_parole = _RP
    except ImportError:
        _dire_texte = None
        _reconnaitre_parole = None


def _dire_texte_safe(*args: Any, **kwargs: Any) -> Any:
    _ensure_behavior_deps()
    fn = _dire_texte
    if fn is None:
        return None
    return fn(*args, **kwargs)



def _reconnaitre_parole_safe(*args: Any, **kwargs: Any) -> Any:
    _ensure_behavior_deps()
    fn = _reconnaitre_parole
    if fn is None:
        return None
    return fn(*args, **kwargs)



def _ensure_create_head_pose() -> bool:
    """Charge create_head_pose à la demande uniquement."""
    global create_head_pose, REACHY_MINI_UTILS_AVAILABLE, _SDK_UTILS_LOAD_ATTEMPTED
    if _SDK_UTILS_LOAD_ATTEMPTED:
        return REACHY_MINI_UTILS_AVAILABLE
    _SDK_UTILS_LOAD_ATTEMPTED = True
    try:
        from reachy_mini.utils import (  # type: ignore[import-untyped]
            create_head_pose as _chp,
        )

        create_head_pose = _chp
        REACHY_MINI_UTILS_AVAILABLE = True
    except ImportError:
        create_head_pose = None
        REACHY_MINI_UTILS_AVAILABLE = False
    return REACHY_MINI_UTILS_AVAILABLE

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

    def _get_wake_messages(self) -> list[str]:
        """Retourne la liste des messages de réveil."""
        return [
            "Bonjour ! Je suis là, prêt à interagir avec vous.",
            "Salut ! BBIA est réveillé et prêt à discuter !",
            "Coucou ! Content de me réveiller à vos côtés.",
            "Bonjour ! Je suis BBIA, votre robot compagnon. " "Comment allez-vous ?",
            "Salut ! Je me sens bien et énergisé aujourd'hui !",
            "Hello ! Je suis prêt pour une nouvelle journée avec vous !",
            "Me voilà ! Je suis là pour vous accompagner.",
            "Bonjour ! Je me réveille avec enthousiasme "
            "pour passer du temps ensemble.",
        ]

    def _try_sdk_wake_up(self) -> bool:
        """Tente le réveil via SDK officiel."""
        if not (self.robot_api and hasattr(self.robot_api, "run_behavior")):
            return False

        try:
            logger.info("Exécution wake_up via SDK officiel")
            success = self.robot_api.run_behavior("wake_up", duration=5.0)
            if success:
                logger.info("Réveil via SDK réussi")
                wake_message = secrets.choice(self._get_wake_messages())
                try:
                    _dire_texte_safe(wake_message, robot_api=self.robot_api)
                    logger.info("Synthèse vocale : %s", wake_message)
                except (RuntimeError, Exception) as e:
                    logger.warning("Synthèse vocale non disponible: %s", e)
                return True
        except (AttributeError, RuntimeError, ValueError):
            logger.exception("Erreur wake_up SDK")
        except Exception:
            logger.exception("Erreur inattendue wake_up SDK")
        return False

    def _perform_manual_wake_sequence(self) -> None:
        """Effectue la séquence de réveil manuelle."""
        logger.info(
            "Étape : Mouvement de tête vers position neutre puis légèrement relevée",
        )
        if self.robot_api and hasattr(self.robot_api, "set_emotion"):
            self.robot_api.set_emotion("neutral", 0.3)
            time.sleep(0.5)
            self.robot_api.set_emotion("happy", 0.5)

        time.sleep(1)
        self._perform_body_rotation()
        time.sleep(1)

    def _perform_body_rotation(self) -> None:
        """Effectue la rotation du corps pour le réveil."""
        logger.info("Étape : Rotation corps subtile avec interpolation fluide (réveil)")
        if not self.robot_api:
            return

        try:
            if hasattr(self.robot_api, "goto_target") and _ensure_create_head_pose():
                self.robot_api.goto_target(
                    body_yaw=0.15, duration=0.8, method="minjerk"
                )
                time.sleep(1.0)
                self.robot_api.goto_target(
                    body_yaw=-0.15, duration=0.8, method="minjerk"
                )
                time.sleep(1.0)
                self.robot_api.goto_target(body_yaw=0.0, duration=0.8, method="minjerk")
            elif hasattr(self.robot_api, "set_joint_pos"):
                self.robot_api.set_joint_pos("yaw_body", 0.15)
                time.sleep(0.5)
                self.robot_api.set_joint_pos("yaw_body", -0.15)
                time.sleep(0.5)
                self.robot_api.set_joint_pos("yaw_body", 0.0)
        except (AttributeError, RuntimeError, ValueError) as e:
            logger.warning("Erreur mouvement corps réveil (continuation): %s", e)
        except (TypeError, KeyError, IndexError) as e:
            logger.warning("Erreur inattendue mouvement corps réveil: %s", e)

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        logger.info("Début de la séquence de réveil BBIA")

        if self._try_sdk_wake_up():
            return True

        self._perform_manual_wake_sequence()

        logger.info("Étape : Première parole d'éveil")
        wake_message = secrets.choice(self._get_wake_messages())
        _dire_texte_safe(wake_message, robot_api=self.robot_api)
        logger.info("Synthèse vocale : %s", wake_message)

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
        logger.info("Salutation choisie : %s", greeting)

        # Appliquer émotion "happy" conforme SDK (pitch=0.1 * intensity)
        if self.robot_api and hasattr(self.robot_api, "set_emotion"):
            self.robot_api.set_emotion("happy", 0.6)

        # OPTIMISATION EXPERT: Utiliser goto_target pour mouvement fluide
        # avec interpolation au lieu de set_target_head_pose répétés
        # (évite mouvements saccadés)
        if self.robot_api and (_ensure_create_head_pose() and create_head_pose):
            try:
                # Méthode 1 (recommandée): goto_target avec interpolation
                # fluide (minjerk)
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
            except (AttributeError, RuntimeError, ValueError) as e:
                logger.warning("Erreur pose tête salutation (fallback): %s", e)
            except (TypeError, KeyError, IndexError) as e:
                logger.warning("Erreur inattendue pose tête salutation: %s", e)
                # Fallback final: rotation corps subtile
                if hasattr(self.robot_api, "set_joint_pos"):
                    try:
                        self.robot_api.set_joint_pos("yaw_body", 0.12)
                        time.sleep(0.5)
                        self.robot_api.set_joint_pos("yaw_body", 0.0)
                    except Exception:
                        logger.exception("Erreur fallback rotation corps")
        elif self.robot_api and hasattr(self.robot_api, "set_joint_pos"):
            # Fallback: rotation corps subtile si SDK non disponible
            try:
                self.robot_api.set_joint_pos("yaw_body", 0.12)
                time.sleep(0.5)
                self.robot_api.set_joint_pos("yaw_body", 0.0)
            except Exception:
                logger.exception("Erreur fallback rotation corps")

        # OPTIMISATION SDK: Passer robot_api pour utiliser haut-parleur 5W
        _dire_texte_safe(greeting, robot_api=self.robot_api)
        logger.info("Synthèse vocale : %s", greeting)
        return True


class EmotionalResponseBehavior(BBIABehavior):
    """Comportement de réponse émotionnelle conforme au SDK Reachy Mini officiel."""

    def __init__(
        self,
        emotions: BBIAEmotions,
        robot_api: RobotAPI | None = None,
    ) -> None:
        super().__init__(
            "emotional_response",
            "Réponse émotionnelle automatique",
            robot_api=robot_api,
        )
        self.emotions = emotions
        self.priority = 8

    def execute(self, context: dict[str, Any]) -> bool:
        stimulus = context.get("stimulus", "")
        logger.info("Stimulus reçu pour réponse émotionnelle : %s", stimulus)
        if stimulus:
            emotion = self.emotions.emotional_response(stimulus)
            logger.info("Réponse émotionnelle générée : %s", emotion)

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
                _dire_texte_safe(comment, robot_api=self.robot_api)
                logger.info("Synthèse vocale (émotion) : %s", comment)

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

    def _try_look_at_world(self, first_object: dict[str, Any]) -> bool:
        """Tente look_at_world si position 3D disponible."""
        robot = self.robot_api
        if robot is None or not hasattr(robot, "look_at_world"):
            return False

        pos = first_object.get("position", {})
        if not pos:
            logger.debug("Pas de position 3D disponible")
            return False

        try:
            x = float(pos.get("x", 0.2))
            y = float(pos.get("y", 0.1))
            z = float(pos.get("z", 0.0))
            if not (-2.0 <= x <= 2.0 and -2.0 <= y <= 2.0 and -1.0 <= z <= 1.0):
                logger.debug("Coordonnées 3D hors limites: (%s, %s, %s)", x, y, z)
                return False

            robot.look_at_world(x, y, z, duration=1.0, perform_movement=True)
            logger.info("Look_at_world vers position 3D: (%.2f, %.2f, %.2f)", x, y, z)
            return True
        except (ValueError, TypeError, AttributeError) as e:
            logger.debug("Erreur look_at_world: %s", e)
            return False

    def _try_look_at_image(self, first_object: dict[str, Any]) -> bool:
        """Tente look_at_image si position 2D disponible."""
        robot = self.robot_api
        if robot is None or not hasattr(robot, "look_at_image"):
            return False

        bbox = first_object.get("bbox", {})
        if not bbox:
            logger.debug("Pas de bbox disponible pour look_at_image")
            return False

        try:
            u = int(bbox.get("center_x", 320))
            v = int(bbox.get("center_y", 240))
            if not (0 <= u <= 640 and 0 <= v <= 480):
                logger.debug("Coordonnées image hors limites: (%s, %s)", u, v)
                return False

            robot.look_at_image(u, v, duration=1.0)
            logger.info("Look_at_image vers pixel: (%s, %s)", u, v)
            return True
        except (ValueError, TypeError, AttributeError) as e:
            logger.debug("Erreur look_at_image: %s", e)
            return False

    def _apply_curious_emotion_fallback(self) -> None:
        """Applique émotion curious comme fallback."""
        robot = self.robot_api
        if robot is None or not hasattr(robot, "set_emotion"):
            return
        try:
            robot.set_emotion("curious", 0.6)
            logger.info("Fallback: émotion curious appliquée")
        except Exception as e:
            logger.debug("Erreur fallback émotion: %s", e)

    def _track_object(self, first_object: dict[str, Any]) -> None:
        """Effectue le suivi de l'objet détecté."""
        if not self.robot_api:
            return

        # Essayer les méthodes de tracking dans l'ordre de priorité
        if self._try_look_at_world(first_object):
            return
        if self._try_look_at_image(first_object):
            return
        self._apply_curious_emotion_fallback()

    def _handle_object_detected(self, first_object: dict[str, Any]) -> bool:
        """Gère le cas où un objet est détecté."""
        object_name = first_object.get("name", "quelque chose")
        logger.info("Suivi de l'objet : %s", object_name)
        self.vision.track_object(first_object["name"])

        detection_comments = [
            f"Je vois {object_name} !",
            f"Oh, il y a {object_name} là-bas !",
            f"Je regarde {object_name}.",
            f"{object_name} m'intrigue !",
            f"Intéressant, je vois {object_name}.",
        ]
        comment = secrets.choice(detection_comments)
        _dire_texte_safe(comment, robot_api=self.robot_api)
        logger.info("Synthèse vocale (vision) : %s", comment)

        self._track_object(first_object)
        return True

    def _handle_no_object_detected(self) -> bool:
        """Gère le cas où aucun objet n'est détecté."""
        if self.robot_api and hasattr(self.robot_api, "set_emotion"):
            self.robot_api.set_emotion("curious", 0.6)

        no_object_comments = [
            "Je ne vois rien d'intéressant pour l'instant.",
            "Rien de nouveau dans mon champ de vision.",
            "Je cherche, mais je ne vois rien de particulier.",
            "Mon environnement semble vide pour le moment.",
            "Aucun objet détecté autour de moi.",
        ]
        comment = secrets.choice(no_object_comments)
        _dire_texte_safe(comment, robot_api=self.robot_api)
        logger.info("Synthèse vocale (vision, aucun objet) : %s", comment)
        logger.info("Aucun objet détecté pour le suivi visuel")
        return False

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        """Exécute le comportement de suivi visuel."""
        logger.info("Activation du suivi visuel")

        result = self.vision.scan_environment()
        logger.info("Résultat du scan environnement : %s", result)

        if result["objects"]:
            return self._handle_object_detected(result["objects"][0])
        return self._handle_no_object_detected()


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

        # Tentative d'importation de BBIAHuggingFace avec outils LLM (optionnel)
        self.hf_chat = None
        try:
            from .bbia_huggingface import BBIAHuggingFace
            from .bbia_tools import BBIATools

            # BBIAVision déjà importé globalement (ligne 26)

            # Initialiser outils LLM pour function calling
            _ensure_behavior_deps()
            vision = _BBIAVision(robot_api=robot_api) if (robot_api and _BBIAVision) else None
            tools = BBIATools(robot_api=robot_api, vision=vision)

            # Créer BBIAHuggingFace avec outils intégrés
            self.hf_chat = BBIAHuggingFace(tools=tools)
            logger.info(
                (
                    "✅ BBIAHuggingFace avec outils LLM disponible - "
                    "Conversation intelligente + function calling activé"
                ),
            )
        except (ImportError, RuntimeError, AttributeError) as e:
            logger.info(
                "ℹ️  BBIAHuggingFace non disponible - Mode enrichi activé: %s",
                e,
            )
            self.hf_chat = None

        # Système de réponses enrichies (fallback)
        self.enriched_responses = self._init_enriched_responses()

    def _init_enriched_responses(self) -> dict[str, list[str]]:
        """Initialise les réponses enrichies avec variété."""
        return {
            "greeting": [
                "Bonjour ! Je suis ravi de vous entendre. "
                "Comment allez-vous aujourd'hui ?",
                "Salut ! Ça fait plaisir de vous voir ! Qu'est-ce qui vous amène ?",
                "Hello ! Je suis BBIA, votre compagnon robotique. "
                "Que puis-je faire pour vous ?",
                "Bonjour ! Prêt pour une belle conversation ? "
                "Dites-moi ce qui vous passe par la tête.",
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
                "C'est intéressant ! Pouvez-vous m'en dire plus ? "
                "J'aimerais mieux comprendre votre point de vue.",
                "Je comprends. Continuez, je vous écoute attentivement. "
                "N'hésitez pas à développer.",
                "Ah, c'est fascinant ! Racontez-moi davantage, ça m'intrigue.",
                "Je vois. C'est un sujet qui mérite réflexion. "
                "Que pensez-vous vous-même de ça ?",
                "Hmm, intriguant ! Qu'est-ce qui vous a amené à penser à ça ?",
                "Intéressant ! Je suis curieux d'en apprendre plus. "
                "Partagez-moi vos réflexions.",
                "Je comprends votre perspective. "
                "Voulez-vous approfondir ce sujet ensemble ?",
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

    def _get_greeting_messages(self) -> list[str]:
        """Retourne la liste des messages d'accueil."""
        return [
            "Je vous écoute attentivement.",
            "Je suis tout ouïe, parlez-moi.",
            "Dites-moi ce qui vous passe par la tête.",
            "Je suis là pour vous, que souhaitez-vous me dire ?",
        ]

    def _handle_huggingface_response(self, texte: str) -> bool:
        """Gère la réponse via BBIAHuggingFace si disponible."""
        if not self.hf_chat:
            return False

        try:
            response = self.hf_chat.chat(texte, enable_tools=True)
            _dire_texte_safe(response, robot_api=self.robot_api)
            logger.info("Synthèse vocale (HF + outils LLM) : %s", response)

            sentiment_result = self.hf_chat.analyze_sentiment(texte)
            sentiment_dict: dict[str, Any] = {
                "sentiment": sentiment_result.get("sentiment", "neutral"),
                "score": sentiment_result.get("score", 0.5),
            }
            self._apply_sentiment_to_robot(sentiment_dict)
            return True
        except (ValueError, TypeError, KeyError) as e:
            logger.warning("Erreur BBIAHuggingFace, fallback enrichi : %s", e)
            return False

    def _handle_enriched_response(self, texte_lower: str) -> None:
        """Gère la réponse via le système enrichi."""
        response = self._generate_enriched_response(texte_lower)
        _dire_texte_safe(response, robot_api=self.robot_api)
        logger.info("Synthèse vocale (enrichi) : %s", response)

        emotion = self._detect_emotion_from_text(texte_lower)
        if emotion and self.robot_api:
            try:
                self.robot_api.set_emotion(emotion, 0.6)
                logger.info("Émotion appliquée au robot : %s", emotion)
            except (ValueError, RuntimeError, AttributeError) as e:
                logger.warning("Erreur application émotion : %s", e)

    def execute(self, context: dict[str, Any]) -> bool:  # noqa: ARG002
        """Exécute une conversation intelligente avec reconnaissance vocale."""
        logger.info("Activation du mode conversation intelligente")

        greeting = secrets.choice(self._get_greeting_messages())  # nosec B311
        _dire_texte_safe(greeting, robot_api=self.robot_api)
        logger.info("Synthèse vocale : %s", greeting)

        texte = _reconnaitre_parole_safe(duree=5, robot_api=self.robot_api)
        logger.info("Texte reconnu : %s", texte)

        if texte:
            texte_lower = texte.lower()
            if not self._handle_huggingface_response(texte):
                self._handle_enriched_response(texte_lower)
        else:
            response = self._get_enriched_response("not_heard")
            _dire_texte_safe(response, robot_api=self.robot_api)
            logger.info("Synthèse vocale : %s", response)

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
        # IMPORTANT: Vérifier "excited" AVANT "happy" pour éviter conflit
        # (excité peut être dans les deux)
        if any(
            word in texte_lower for word in ["excité", "enthousiaste", "euphorique"]
        ):
            return "excited"
        if any(
            word in texte_lower for word in ["super", "génial", "content", "heureux"]
        ):
            return "happy"
        if any(word in texte_lower for word in ["triste", "déçu", "malheureux"]):
            return "sad"
        if any(word in texte_lower for word in ["curieux", "intrigué", "intéressant"]):
            return "curious"
        if any(word in texte_lower for word in ["calme", "serein", "détendu"]):
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
                    (
                        f"Émotion appliquée via sentiment : {emotion} "
                        f"(intensité: {intensity:.2f})"
                    ),
                )
        except (ValueError, RuntimeError, AttributeError) as e:
            logger.warning("Erreur application sentiment au robot : %s", e)


class AntennaAnimationBehavior(BBIABehavior):
    """Comportement d'animation expressivité (alternative aux antennes).

    ✅ IMPORTANT: Les antennes sont maintenant ANIMABLES dans le modèle
    officiel (range=[-0.300, 0.300]).
    Ce comportement utilise les antennes avec limites de sécurité
    (-0.3 à 0.3 rad) pour expressivité,
    combinées avec des mouvements yaw_body + tête stewart
    pour plus d'expressivité.
    """

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        super().__init__(
            "antenna_animation",
            "Animation expressive selon l'émotion (antennes protégées)",
            robot_api=robot_api,
        )
        self.priority = 5

    def execute(self, context: dict[str, Any]) -> bool:
        emotion = context.get("emotion", "neutral")
        logger.info("Animation expressive pour l'émotion : %s", emotion)

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

            # OPTIMISATION EXPERT: Utiliser goto_target pour mouvements
            # fluides expressifs au lieu de set_joint_pos répétés
            # (meilleure fluidité et performance)
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
                except (ValueError, RuntimeError, AttributeError) as e:
                    logger.warning("Erreur mouvement expressif %s: %s", emotion, e)

        logger.info("Animation expressive appliquée pour : %s", emotion)
        time.sleep(1)
        return True


class HideBehavior(BBIABehavior):
    """Comportement 'se cacher' (hide) conforme au SDK Reachy Mini officiel.

    Simule le repli en baissant la tête (pitch négatif)
    et rotation corps vers l'arrière.
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
        logger.info("🙈 [BBIA] Séquence 'se cacher'...")
        logger.info("Début de la séquence 'se cacher'")

        # Utiliser émotion "sad" conforme SDK (pitch=-0.1 * intensity)
        logger.info("🤖 Tête qui s'abaisse lentement...")
        logger.info("Étape : Tête qui s'abaisse lentement")
        if self.robot_api and hasattr(self.robot_api, "set_emotion"):
            # Émotion sad avec intensité forte = pitch négatif (tête baissée)
            self.robot_api.set_emotion("sad", 0.9)  # Conforme SDK: pitch=-0.09 rad

        # Note: Les antennes ne sont pas contrôlées (protégées pour sécurité hardware)
        logger.info("📡 Antennes qui se replient devant le visage...")
        logger.info("Étape : Antennes (simulation - non contrôlées pour sécurité)")

        logger.info("👁️ Yeux qui se ferment (ou s'éteignent)...")
        logger.info("Étape : Yeux qui se ferment")

        # OPTIMISATION EXPERT: Utiliser goto_target pour mouvement combiné
        # tête+corps fluide au lieu de mouvements séparés
        # (meilleure synchronisation)
        if self.robot_api:
            try:
                if (
                    _ensure_create_head_pose()
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
                    if _ensure_create_head_pose() and create_head_pose:
                        if hasattr(self.robot_api, "set_target_head_pose"):
                            # Pose tête baissée (conforme SDK sad: pitch=-0.1,
                            # ici plus fort)
                            pose = create_head_pose(pitch=-0.15, yaw=0.0, degrees=False)
                            self.robot_api.set_target_head_pose(pose)
            except (ValueError, RuntimeError, AttributeError) as e:
                logger.warning("Erreur mouvement hide (continuation): %s", e)

        time.sleep(1.0)

        logger.info("💤 BBIA se cache et devient silencieux.")
        logger.info("Étape : BBIA se cache et devient silencieux")
        logger.info("(BBIA attend discrètement...)")
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
        _dire_texte_safe(hide_message, robot_api=self.robot_api)
        logger.info("Synthèse vocale : %s", hide_message)
        time.sleep(1)
        logger.info("Fin de la séquence 'se cacher'")
        return True


class BBIABehaviorManager:
    """Gestionnaire de comportements pour BBIA."""

    def __init__(self, robot_api: RobotAPI | None = None) -> None:
        """Initialise le gestionnaire de comportements.

        Args:
            robot_api: Instance RobotAPI pour contrôler le robot (optionnel).
                       Si None, les comportements fonctionneront
                       en mode simulation/logs uniquement.

        """
        self.behaviors: dict[str, BBIABehavior] = {}
        self.active_behaviors: list[str] = []
        # OPTIMISATION RAM: Limiter queue comportements (max 50)
        self.behavior_queue: Queue[tuple[str, dict[str, Any]]] = Queue(maxsize=50)
        self.is_running = False
        self.worker_thread: threading.Thread | None = None

        # RobotAPI pour contrôler le robot physique
        self.robot_api = robot_api

        # OPTIMISATION PERFORMANCE: Bibliothèque de mouvements enregistrés
        # pour réutilisation
        self.saved_moves: dict[str, Any] = {}

        # Initialiser les modules BBIA
        _ensure_behavior_deps()
        if _BBIAEmotions is None:
            raise ImportError("BBIAEmotions non disponible")
        self.emotions = _BBIAEmotions()
        # OPTIMISATION RAM: Utiliser singleton BBIAVision si disponible
        try:
            if _get_bbia_vision_singleton is not None:
                self.vision = _get_bbia_vision_singleton(robot_api)
            elif _BBIAVision is not None:
                self.vision = _BBIAVision(robot_api=robot_api)
            else:
                raise ImportError("BBIAVision non disponible")
        except (ImportError, AttributeError):
            if _BBIAVision is None:
                raise
            self.vision = _BBIAVision(robot_api=robot_api)

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

        # Enregistrer les nouveaux comportements avancés
        try:
            from .behaviors import (
                AlarmClockBehavior,
                DanceBehavior,
                EmotionShowBehavior,
                ExerciseBehavior,
                FollowFaceBehavior,
                FollowObjectBehavior,
                GameBehavior,
                MeditationBehavior,
                MusicReactionBehavior,
                NewsReaderBehavior,
                PhotoBoothBehavior,
                StorytellingBehavior,
                TeachingBehavior,
                WeatherReportBehavior,
            )
            from .behaviors import (
                ConversationBehavior as NewConversationBehavior,
            )

            # Comportements améliorés (remplacer ou complémenter les existants)
            self.register_behavior(
                FollowFaceBehavior(vision=self.vision, robot_api=self.robot_api),
            )
            self.register_behavior(
                FollowObjectBehavior(vision=self.vision, robot_api=self.robot_api),
            )
            self.register_behavior(
                NewConversationBehavior(robot_api=self.robot_api),
            )

            # Nouveaux comportements expressifs
            self.register_behavior(DanceBehavior(robot_api=self.robot_api))
            self.register_behavior(EmotionShowBehavior(robot_api=self.robot_api))
            self.register_behavior(
                PhotoBoothBehavior(vision=self.vision, robot_api=self.robot_api),
            )

            # Nouveaux comportements interactifs
            self.register_behavior(StorytellingBehavior(robot_api=self.robot_api))
            self.register_behavior(TeachingBehavior(robot_api=self.robot_api))
            self.register_behavior(GameBehavior(robot_api=self.robot_api))

            # Nouveaux comportements utilitaires
            self.register_behavior(MeditationBehavior(robot_api=self.robot_api))
            self.register_behavior(ExerciseBehavior(robot_api=self.robot_api))
            self.register_behavior(AlarmClockBehavior(robot_api=self.robot_api))
            self.register_behavior(WeatherReportBehavior(robot_api=self.robot_api))
            self.register_behavior(NewsReaderBehavior(robot_api=self.robot_api))
            self.register_behavior(MusicReactionBehavior(robot_api=self.robot_api))

            logger.info(
                "✅ Tous les comportements avancés enregistrés avec succès (15/15)",
            )
        except ImportError as e:
            logger.warning("⚠️  Comportements avancés non disponibles: %s", e)

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
            except Exception:
                logger.exception("Erreur dans le worker de comportements ")
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
                (
                    f"🎬 Enregistrement mouvement comportement "
                    f"'{behavior_name}' ({duration}s)..."
                ),
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
            logger.warning("⚠️  Aucun mouvement enregistré pour '%s'", behavior_name)
            return None
        except Exception:
            logger.exception(
                f"❌ Erreur enregistrement comportement '{behavior_name}':"
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
            logger.warning("⚠️  Aucun mouvement sauvegardé pour '%s'", behavior_name)
            return False

        if not self.robot_api:
            logger.warning("Robot API non disponible")
            return False

        try:
            move = self.saved_moves[behavior_name]

            # Issue #344: Améliorer enchaînement fluide des mouvements
            # Utiliser initial_goto_duration > 0 pour transition fluide
            # depuis position actuelle
            initial_goto_duration = 0.5  # Transition de 0.5s pour enchaînement fluide

            # OPTIMISATION PERFORMANCE: Utiliser async_play_move si disponible
            # (non bloquant)
            if use_async and hasattr(self.robot_api, "async_play_move"):
                self.robot_api.async_play_move(
                    move,
                    play_frequency=100.0,
                    initial_goto_duration=initial_goto_duration,
                )
                logger.info(
                    f"▶️  Rejoué '{behavior_name}' (async, non bloquant) - "
                    f"{len(move) if isinstance(move, list) else 'N/A'} frames",
                )
                return True
            if hasattr(self.robot_api, "play_move"):
                self.robot_api.play_move(
                    move,
                    play_frequency=100.0,
                    initial_goto_duration=initial_goto_duration,
                )
                logger.info(
                    f"▶️  Rejoué '{behavior_name}' (sync, bloquant) - "
                    f"{len(move) if isinstance(move, list) else 'N/A'} frames",
                )
                return True
            logger.warning("play_move non disponible dans robot_api")
            return False
        except Exception:
            logger.exception("❌ Erreur rejouant '%s':", behavior_name)
            return False

    def clear_saved_moves(self) -> None:
        """Efface tous les mouvements enregistrés."""
        self.saved_moves.clear()
        logger.info("🗑️  Tous les mouvements enregistrés ont été effacés")


# Instance globale pour les fonctions standalone
_global_manager: BBIABehaviorManager | None = None


def _get_global_manager() -> BBIABehaviorManager:
    """Obtient ou crée l'instance globale du gestionnaire de comportements."""
    global _global_manager
    if _global_manager is None:
        _global_manager = BBIABehaviorManager()
    return _global_manager


def clear_saved_moves() -> None:
    """Fonction standalone pour effacer tous les mouvements enregistrés."""
    manager = _get_global_manager()
    manager.clear_saved_moves()


class SimpleMove:
    """Classe Move simple standalone (ne dépend pas de reachy_mini).

    Permet de créer et interpoler des mouvements sans dépendance externe.
    """

    def __init__(
        self,
        positions: list[dict[str, float]],
        duration: float,
    ) -> None:
        """Initialise un mouvement simple.

        Args:
            positions: Liste de positions de joints
            duration: Durée du mouvement en secondes

        """
        self._positions = positions
        self._duration = duration

    def duration(self) -> float:
        """Retourne la durée du mouvement."""
        return float(self._duration)

    def evaluate(self, t: float) -> dict[str, float]:
        """Évalue la position à un temps t (0.0 à 1.0).

        Args:
            t: Temps normalisé entre 0.0 et 1.0

        Returns:
            Positions interpolées des joints

        """
        if not self._positions or t <= 0:
            return self._positions[0] if self._positions else {}
        if t >= 1:
            return self._positions[-1] if self._positions else {}

        idx = int(t * (len(self._positions) - 1))
        if idx >= len(self._positions) - 1:
            return dict(self._positions[-1]) if self._positions else {}

        pos1 = self._positions[idx]
        pos2 = self._positions[idx + 1]

        result: dict[str, float] = {}
        for key in pos1:
            if key in pos2:
                result[key] = pos1[key] + (pos2[key] - pos1[key]) * (
                    t * (len(self._positions) - 1) - idx
                )
            else:
                result[key] = pos1[key]
        return result


def _try_create_move_via_backend(
    positions_list: list[dict[str, float]],
    duration: float,
) -> SimpleMove | None:
    """Tente de créer un Move via le backend SDK si disponible.

    Returns:
        Objet Move du backend ou None si non disponible

    """
    try:
        from .robot_factory import RobotFactory

        backend = RobotFactory.create_backend("reachy_mini")
        if not backend:
            backend = RobotFactory.create_backend("mujoco")

        if backend and hasattr(backend, "create_move_from_positions"):
            try:
                result = backend.create_move_from_positions(positions_list, duration)
                if result is not None:
                    # Wrapper le résultat en SimpleMove si nécessaire
                    if isinstance(result, SimpleMove):
                        return result  # type: ignore[return-value]
                    # Si c'est un autre type d'objet Move, créer un SimpleMove
                    return SimpleMove(positions_list, duration)  # type: ignore[return-value]
            except (ImportError, ModuleNotFoundError) as e:
                logger.debug(
                    "Backend create_move_from_positions échoué (import): %s",
                    e,
                )
            except Exception as e:
                logger.debug(
                    "Backend create_move_from_positions échoué: %s",
                    e,
                )
    except (ImportError, ModuleNotFoundError) as e:
        logger.debug("RobotFactory/imports non disponibles: %s", e)
    except Exception as e:
        logger.debug("RobotFactory non disponible: %s", e)

    return None


def create_move_from_positions(
    positions: dict[str, float] | list[dict[str, float]],
    duration: float = 1.0,
) -> object | None:
    """Crée un objet Move à partir de positions.

    Args:
        positions: Dict de positions de joints ou liste de dicts
        duration: Durée du mouvement en secondes

    Returns:
        Objet Move ou None si erreur

    """
    try:
        # Convertir dict unique en liste si nécessaire
        if isinstance(positions, dict):
            positions_list: list[dict[str, float]] = [positions]
        else:
            positions_list = positions

        # Essayer d'utiliser le backend SDK si disponible
        backend_result = _try_create_move_via_backend(positions_list, duration)
        if backend_result is not None:
            return backend_result

        # Fallback: SimpleMove (fonctionne sans reachy_mini)
        return SimpleMove(positions_list, duration)

    except Exception:
        logger.exception("Erreur création Move depuis positions")
        return None


def create_move_task(
    positions: dict[str, float] | list[dict[str, float]],
    duration: float = 1.0,
) -> UUID | None:
    """Crée une tâche de mouvement avec UUID.

    Args:
        positions: Dict de positions de joints ou liste de dicts
        duration: Durée du mouvement en secondes

    Returns:
        UUID de la tâche ou None si erreur
    """
    try:
        # Créer le move d'abord
        move = create_move_from_positions(positions, duration)
        if move is None:
            return None

        # Retourner un UUID pour la tâche
        return uuid4()
    except Exception:
        logger.exception("Erreur création tâche mouvement")
        return None


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
