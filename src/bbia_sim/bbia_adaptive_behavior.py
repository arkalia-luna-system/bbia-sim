#!/usr/bin/env python3
"""BBIA Adaptive Behavior - Module de comportements adaptatifs contextuels
Génération de comportements dynamiques basés sur le contexte et l'état émotionnel
"""

import logging
import random
import time
from typing import TYPE_CHECKING, Any, Optional

import numpy as np

if TYPE_CHECKING:
    from .robot_api import RobotAPI

logger = logging.getLogger(__name__)


class BBIAAdaptiveBehavior:
    """Module de comportements adaptatifs contextuels pour BBIA-SIM.

    Fonctionnalités :
    - Génération de comportements basés sur le contexte
    - Adaptation émotionnelle des mouvements
    - Apprentissage des préférences utilisateur
    - Comportements proactifs et réactifs

    ⚠️ IMPORTANT EXPERT ROBOTIQUE:
    Les comportements générés utilisent "head_pose"
    au lieu de joints stewart individuels,
    car la plateforme Stewart nécessite la cinématique inverse (IK).
    Les comportements doivent être exécutés via goto_target(head=pose)
    ou look_at_world() du SDK Reachy Mini officiel.
    """

    def __init__(self, robot_api: Optional["RobotAPI"] = None) -> None:
        """Initialise le module de comportements adaptatifs.

        Args:
            robot_api: Instance RobotAPI pour exécuter les comportements
                générés (optionnel)

        """
        self.is_active = False
        self.current_context = "neutral"
        self.behavior_history: list[dict[str, Any]] = []
        self.robot_api = (
            robot_api  # OPTIMISATION SDK: Passer robot_api pour exécution conforme
        )

        # Contexte et états
        self.contexts = {
            "greeting": {
                "priority": 0.9,
                "duration": 3.0,
                "emotions": ["happy", "excited"],
            },
            "conversation": {
                "priority": 0.8,
                "duration": 10.0,
                "emotions": ["neutral", "curious"],
            },
            "attention": {
                "priority": 0.7,
                "duration": 5.0,
                "emotions": ["curious", "excited"],
            },
            "rest": {
                "priority": 0.3,
                "duration": 15.0,
                "emotions": ["calm", "neutral"],
            },
            "playful": {
                "priority": 0.6,
                "duration": 8.0,
                "emotions": ["happy", "excited"],
            },
            "serious": {
                "priority": 0.8,
                "duration": 12.0,
                "emotions": ["neutral", "determined"],
            },
            "sleepy": {
                "priority": 0.2,
                "duration": 20.0,
                "emotions": ["calm", "neutral"],
            },
        }

        # Comportements disponibles
        # ⚠️ IMPORTANT EXPERT ROBOTIQUE: Les joints stewart (stewart_1-6)
        # NE PEUVENT PAS être contrôlés individuellement
        # car la plateforme Stewart utilise la cinématique inverse (IK).
        # Utiliser goto_target() avec create_head_pose() pour contrôler la tête.
        # Les joints stewart listés ici sont indicatifs uniquement
        # pour documentation.
        self.behaviors = {
            "nod": {
                "description": "Hochement de tête",
                "emotions": ["happy", "excited", "curious"],
                "duration": 2.0,
                "intensity_range": (0.3, 0.8),
                "joints": [
                    "yaw_body",
                    "head_pose",
                ],  # head_pose = goto_target(head=pose) avec IK
            },
            "shake": {
                "description": "Secouement de tête",
                "emotions": ["sad", "confused", "disgusted"],
                "duration": 2.5,
                "intensity_range": (0.4, 0.9),
                "joints": [
                    "yaw_body",
                    "head_pose",
                ],  # head_pose = goto_target(head=pose) avec IK
            },
            "look_around": {
                "description": "Regarder autour",
                "emotions": ["curious", "excited", "neutral"],
                "duration": 4.0,
                "intensity_range": (0.2, 0.6),
                "joints": [
                    "yaw_body",
                    "head_pose",
                ],  # head_pose = look_at_world() ou goto_target(head=pose)
            },
            "focus": {
                "description": "Se concentrer sur un point",
                "emotions": ["determined", "curious", "neutral"],
                "duration": 3.0,
                "intensity_range": (0.1, 0.4),
                "joints": [
                    "head_pose",
                ],  # head_pose = goto_target(head=pose) ou look_at_world()
            },
            "stretch": {
                "description": "Étirement",
                "emotions": ["calm", "neutral", "tired"],
                "duration": 5.0,
                "intensity_range": (0.3, 0.7),
                "joints": [
                    "head_pose",
                ],  # head_pose = goto_target(head=pose) avec séquence de poses
            },
            "dance": {
                "description": "Mouvement de danse",
                "emotions": ["happy", "excited", "playful"],
                "duration": 6.0,
                "intensity_range": (0.5, 1.0),
                "joints": [
                    "yaw_body",
                    "head_pose",
                ],  # head_pose = séquence goto_target(head=pose)
            },
            "hide": {
                "description": "Se cacher",
                "emotions": ["fearful", "shy", "sad"],
                "duration": 3.0,
                "intensity_range": (0.2, 0.5),
                "joints": [
                    "head_pose",
                    "yaw_body",
                ],  # head_pose = goto_target(head=pose_down, body_yaw=-0.2)
            },
            "celebrate": {
                "description": "Célébration",
                "emotions": ["happy", "excited", "proud"],
                "duration": 4.0,
                "intensity_range": (0.6, 1.0),
                "joints": [
                    "yaw_body",
                    "head_pose",
                ],  # head_pose = séquence goto_target(head=pose) animée
            },
        }

        # Apprentissage des préférences
        self.user_preferences: dict[str, dict[str, float]] = {
            "preferred_emotions": {},
            "preferred_behaviors": {},
            "interaction_patterns": {},
            "timing_preferences": {},
        }

        # État interne
        self.current_emotion = "neutral"
        self.emotion_intensity = 0.5
        self.last_interaction_time = time.time()
        self.interaction_count = 0

        logger.info("🧠 BBIA Adaptive Behavior initialisé")

    def set_context(self, context: str, confidence: float = 1.0) -> bool:
        """Définit le contexte actuel.

        Args:
            context: Nouveau contexte
            confidence: Confiance dans le contexte (0.0-1.0)

        Returns:
            True si contexte valide

        """
        if context not in self.contexts:
            logger.warning("Contexte inconnu: %s", context)
            return False

        self.current_context = context
        logger.info("🎭 Contexte changé: %s (confiance: %.2f)", context, confidence)
        return True

    def set_emotion_state(self, emotion: str, intensity: float = 0.5) -> bool:
        """Définit l'état émotionnel actuel.

        Args:
            emotion: Émotion actuelle
            intensity: Intensité de l'émotion (0.0-1.0)

        Returns:
            True si émotion valide

        """
        self.current_emotion = emotion
        self.emotion_intensity = max(0.0, min(1.0, intensity))
        logger.info("😊 Émotion: %s (intensité: %.2f)", emotion, self.emotion_intensity)
        return True

    def generate_behavior(self, trigger: str | None = None) -> dict[str, Any]:
        """Génère un comportement adaptatif basé sur le contexte et l'émotion.

        Args:
            trigger: Élément déclencheur (optionnel)

        Returns:
            Dictionnaire décrivant le comportement généré

        """
        try:
            # Sélection du comportement basée sur le contexte et l'émotion
            suitable_behaviors = self._get_suitable_behaviors()

            if not suitable_behaviors:
                # Comportement par défaut
                behavior_name = "look_around"
                behavior_config = self.behaviors[behavior_name]
            else:
                # Sélection pondérée
                behavior_name = self._select_weighted_behavior(suitable_behaviors)
                behavior_config = self.behaviors[behavior_name]

            # Génération des paramètres du comportement
            behavior_params = self._generate_behavior_parameters(behavior_config)

            # Création du comportement
            behavior = {
                "name": behavior_name,
                "description": behavior_config["description"],
                "context": self.current_context,
                "emotion": self.current_emotion,
                "emotion_intensity": self.emotion_intensity,
                "parameters": behavior_params,
                "trigger": trigger,
                "timestamp": time.time(),
                "id": f"behavior_{int(time.time() * 1000)}",
            }

            # Ajout à l'historique
            self._add_to_history(behavior)

            # Mise à jour des préférences
            self._update_preferences(behavior)

            logger.info(
                (
                    f"🎭 Comportement généré: {behavior_name} "
                    f"pour contexte {self.current_context}"
                ),
            )
            return behavior

        except Exception as e:
            logger.exception("❌ Erreur génération comportement: %s", e)
            # Retourner un comportement par défaut en cas d'erreur
            return {
                "name": "look_around",
                "description": "Regarder autour",
                "context": self.current_context,
                "emotion": self.current_emotion,
                "emotion_intensity": self.emotion_intensity,
                "parameters": {
                    "duration": 3.0,
                    "joints": ["yaw_body"],
                    "intensity": 0.3,
                    "timing": {"start_delay": 0.0, "end_delay": 0.0},
                    "variations": [],
                },
                "trigger": "error_fallback",
                "timestamp": time.time(),
                "id": f"behavior_error_{int(time.time() * 1000)}",
            }

    def _get_suitable_behaviors(self) -> list[str]:
        """Retourne la liste des comportements adaptés au contexte et à l'émotion."""
        suitable = []

        for behavior_name, behavior_config in self.behaviors.items():
            # Vérification compatibilité émotion
            emotions_list = behavior_config.get("emotions", [])
            if (
                (
                    isinstance(emotions_list, list)
                    and self.current_emotion in emotions_list
                )
                or (
                    self.current_context in ("playful", "greeting")
                    and behavior_name
                    in (
                        "dance",
                        "celebrate",
                    )
                )
                or (
                    self.current_context in ("serious", "attention")
                    and behavior_name
                    in (
                        "focus",
                        "nod",
                    )
                )
                or (
                    self.current_context in ("rest", "sleepy")
                    and behavior_name
                    in (
                        "stretch",
                        "hide",
                    )
                )
            ):
                suitable.append(behavior_name)

        return suitable if suitable else ["look_around"]

    def _select_weighted_behavior(self, suitable_behaviors: list[str]) -> str:
        """Sélectionne un comportement avec pondération basée sur les préférences."""
        weights = []

        for behavior_name in suitable_behaviors:
            # Poids de base
            weight = 1.0

            # Bonus pour les préférences utilisateur
            if behavior_name in self.user_preferences["preferred_behaviors"]:
                weight += (
                    self.user_preferences["preferred_behaviors"][behavior_name] * 0.5
                )

            # Bonus pour l'intensité émotionnelle
            if self.emotion_intensity > 0.7:
                if behavior_name in ["dance", "celebrate", "nod"]:
                    weight += 0.3
            elif self.emotion_intensity < 0.3:
                if behavior_name in ["stretch", "hide", "focus"]:
                    weight += 0.3

            weights.append(weight)

        # Sélection pondérée
        total_weight = sum(weights)
        if total_weight == 0:
            return suitable_behaviors[0]

        normalized_weights = [w / total_weight for w in weights]
        selected_idx = np.random.choice(len(suitable_behaviors), p=normalized_weights)

        return suitable_behaviors[selected_idx]

    def _generate_behavior_parameters(
        self,
        behavior_config: dict[str, Any],
    ) -> dict[str, Any]:
        """Génère les paramètres spécifiques d'un comportement."""
        params = {
            "duration": behavior_config["duration"],
            "joints": behavior_config["joints"].copy(),
            "intensity": 0.0,
            "timing": {},
            "variations": [],
        }

        # Intensité basée sur l'émotion et le contexte
        min_intensity, max_intensity = behavior_config["intensity_range"]

        # Ajustement basé sur l'intensité émotionnelle
        base_intensity = (
            min_intensity + (max_intensity - min_intensity) * self.emotion_intensity
        )

        # Variation aléatoire
        intensity_variation = random.uniform(-0.1, 0.1)  # nosec B311
        params["intensity"] = max(0.0, min(1.0, base_intensity + intensity_variation))

        # Timing basé sur le contexte
        self.contexts[self.current_context]
        params["timing"]["start_delay"] = random.uniform(0.0, 0.5)  # nosec B311
        params["timing"]["end_delay"] = random.uniform(0.0, 0.3)  # nosec B311

        # Variations pour chaque joint
        for joint in params["joints"]:
            variation = {
                "joint": joint,
                "amplitude": random.uniform(0.8, 1.2),  # nosec B311
                "phase_offset": random.uniform(0.0, 2 * np.pi),  # nosec B311
                "frequency": random.uniform(0.8, 1.2),  # nosec B311
            }
            params["variations"].append(variation)

        return params

    def _add_to_history(self, behavior: dict[str, Any]) -> None:
        """Ajoute un comportement à l'historique."""
        self.behavior_history.append(behavior)

        # Limitation de la taille de l'historique
        max_history = 100
        if len(self.behavior_history) > max_history:
            self.behavior_history = self.behavior_history[-max_history:]

    def _update_preferences(self, behavior: dict[str, Any]) -> None:
        """Met à jour les préférences utilisateur basées sur le comportement."""
        behavior_name = behavior["name"]
        emotion = behavior["emotion"]

        # Mise à jour des préférences de comportement
        if behavior_name not in self.user_preferences["preferred_behaviors"]:
            self.user_preferences["preferred_behaviors"][behavior_name] = 0.0

        # Augmentation graduelle des préférences
        self.user_preferences["preferred_behaviors"][behavior_name] += 0.01

        # Mise à jour des préférences d'émotion
        if emotion not in self.user_preferences["preferred_emotions"]:
            self.user_preferences["preferred_emotions"][emotion] = 0.0

        self.user_preferences["preferred_emotions"][emotion] += 0.01

        # Normalisation périodique
        if len(self.behavior_history) % 20 == 0:
            self._normalize_preferences()

    def _normalize_preferences(self) -> None:
        """Normalise les préférences pour éviter l'explosion des valeurs."""
        for category in ["preferred_behaviors", "preferred_emotions"]:
            if self.user_preferences[category]:
                max_val = max(self.user_preferences[category].values())
                if max_val > 1.0:
                    for key in self.user_preferences[category]:
                        self.user_preferences[category][key] /= max_val

    def get_proactive_behavior(self) -> dict[str, Any] | None:
        """Génère un comportement proactif basé sur l'état interne."""
        try:
            current_time = time.time()
            time_since_interaction = current_time - self.last_interaction_time

            # Comportements proactifs basés sur le temps
            if time_since_interaction > 30:  # 30 secondes sans interaction
                if self.current_context == "rest":
                    return self.generate_behavior("proactive_attention")
                if self.current_context == "conversation":
                    return self.generate_behavior("proactive_engagement")

            # Comportements basés sur l'émotion
            if self.current_emotion == "excited" and self.emotion_intensity > 0.8:
                return self.generate_behavior("proactive_celebration")
            if self.current_emotion == "curious" and self.emotion_intensity > 0.6:
                return self.generate_behavior("proactive_exploration")

            return None

        except Exception as e:
            logger.exception("❌ Erreur comportement proactif: %s", e)
            return None

    def adapt_to_feedback(self, behavior_id: str, feedback: str, score: float) -> None:
        """Adapte les comportements basés sur les retours utilisateur.

        Args:
            behavior_id: ID du comportement évalué
            feedback: Type de retour ("positive", "negative", "neutral")
            score: Score numérique (0.0-1.0)

        """
        try:
            # Recherche du comportement dans l'historique
            behavior = None
            for b in self.behavior_history:
                if b.get("id") == behavior_id:
                    behavior = b
                    break

            if not behavior:
                logger.warning(
                    f"Comportement {behavior_id} non trouvé dans l'historique",
                )
                return

            behavior_name = behavior["name"]

            # Ajustement des préférences
            if feedback == "positive":
                adjustment = score * 0.1
            elif feedback == "negative":
                adjustment = -score * 0.1
            else:  # neutral
                adjustment = 0.0

            # Mise à jour des préférences
            if behavior_name in self.user_preferences["preferred_behaviors"]:
                self.user_preferences["preferred_behaviors"][
                    behavior_name
                ] += adjustment
                self.user_preferences["preferred_behaviors"][behavior_name] = max(
                    0.0,
                    self.user_preferences["preferred_behaviors"][behavior_name],
                )

            logger.info(
                f"🔄 Adaptation basée sur retour: {feedback} (score: {score:.2f})",
            )

        except Exception as e:
            logger.exception("❌ Erreur adaptation feedback: %s", e)

    def get_behavior_statistics(self) -> dict[str, Any]:
        """Retourne les statistiques des comportements."""
        if not self.behavior_history:
            return {"message": "Aucun historique de comportement"}

        # Statistiques par comportement
        behavior_counts: dict[str, int] = {}
        emotion_counts: dict[str, int] = {}
        context_counts: dict[str, int] = {}

        for behavior in self.behavior_history:
            name = behavior["name"]
            emotion = behavior["emotion"]
            context = behavior["context"]

            behavior_counts[name] = behavior_counts.get(name, 0) + 1
            emotion_counts[emotion] = emotion_counts.get(emotion, 0) + 1
            context_counts[context] = context_counts.get(context, 0) + 1

        return {
            "total_behaviors": len(self.behavior_history),
            "behavior_distribution": behavior_counts,
            "emotion_distribution": emotion_counts,
            "context_distribution": context_counts,
            "user_preferences": self.user_preferences,
            "current_state": {
                "context": self.current_context,
                "emotion": self.current_emotion,
                "emotion_intensity": self.emotion_intensity,
                "interaction_count": self.interaction_count,
            },
        }

    def reset_learning(self) -> None:
        """Remet à zéro l'apprentissage des préférences."""
        self.user_preferences = {
            "preferred_emotions": {},
            "preferred_behaviors": {},
            "interaction_patterns": {},
            "timing_preferences": {},
        }
        self.behavior_history = []
        logger.info("🔄 Apprentissage réinitialisé")

    def execute_behavior(
        self,
        behavior: dict[str, Any],
        robot_api: Optional["RobotAPI"] = None,
    ) -> bool:
        """Exécute un comportement généré de manière conforme au SDK Reachy Mini.

        Utilise goto_target avec IK pour les mouvements tête (conforme SDK),
        au lieu de contrôler joints stewart individuellement.

        Args:
            behavior: Comportement généré (retourné par generate_behavior)
            robot_api: Instance RobotAPI (utilise self.robot_api si None)

        Returns:
            True si exécuté avec succès

        """
        robot_api = robot_api or self.robot_api
        if not robot_api:
            logger.warning("⚠️ robot_api non disponible - comportement non exécuté")
            return False

        try:
            behavior_name = behavior.get("name")
            params = behavior.get("parameters", {})
            duration = params.get("duration", 2.0)
            intensity = params.get("intensity", 0.5)
            joints = params.get("joints", [])

            logger.info(
                (
                    f"🎭 Exécution comportement '{behavior_name}' "
                    f"(durée={duration:.1f}s, intensité={intensity:.2f})"
                ),
            )

            # OPTIMISATION EXPERT SDK: Exécuter selon type de comportement
            # avec goto_target + IK pour mouvements tête (conforme SDK)

            try:
                from reachy_mini.utils import create_head_pose

                REACHY_UTILS_AVAILABLE = True
            except ImportError:
                REACHY_UTILS_AVAILABLE = False
                create_head_pose = None

            # Mapping comportements → exécution SDK conforme
            if behavior_name == "nod":
                # Hochement tête (happy/excited/curious)
                if (
                    REACHY_UTILS_AVAILABLE
                    and create_head_pose
                    and hasattr(robot_api, "goto_target")
                ):
                    # Mouvement tête: pitch vers haut puis retour
                    pose_up = create_head_pose(
                        pitch=0.1 * intensity,
                        yaw=0.0,
                        degrees=False,
                    )
                    robot_api.goto_target(
                        head=pose_up,
                        duration=duration / 2,
                        method="minjerk",
                    )
                    time.sleep(duration / 2 + 0.1)
                    pose_neutral = create_head_pose(pitch=0.0, yaw=0.0, degrees=False)
                    robot_api.goto_target(
                        head=pose_neutral,
                        duration=duration / 2,
                        method="minjerk",
                    )
                    logger.info("✅ Hochement exécuté via goto_target (conforme SDK)")
                    return True

            elif behavior_name == "shake":
                # Secouement tête (sad/confused)
                if (
                    REACHY_UTILS_AVAILABLE
                    and create_head_pose
                    and hasattr(robot_api, "goto_target")
                ):
                    # Mouvement tête: yaw gauche/droite
                    for _ in range(2):
                        pose_left = create_head_pose(
                            pitch=0.0,
                            yaw=-0.15 * intensity,
                            degrees=False,
                        )
                        robot_api.goto_target(
                            head=pose_left,
                            duration=duration / 4,
                            method="cartoon",
                        )
                        time.sleep(duration / 4 + 0.1)
                        pose_right = create_head_pose(
                            pitch=0.0,
                            yaw=0.15 * intensity,
                            degrees=False,
                        )
                        robot_api.goto_target(
                            head=pose_right,
                            duration=duration / 4,
                            method="cartoon",
                        )
                        time.sleep(duration / 4 + 0.1)
                    pose_neutral = create_head_pose(pitch=0.0, yaw=0.0, degrees=False)
                    robot_api.goto_target(
                        head=pose_neutral,
                        duration=0.3,
                        method="minjerk",
                    )
                    logger.info("✅ Secouement exécuté via goto_target (conforme SDK)")
                    return True

            elif behavior_name == "look_around":
                # Regarder autour (curious/excited)
                if REACHY_UTILS_AVAILABLE and hasattr(robot_api, "look_at_world"):
                    # Utiliser look_at_world pour mouvement naturel
                    positions = [
                        (0.3, 0.0, 0.2),
                        (0.0, 0.3, 0.2),
                        (-0.3, 0.0, 0.2),
                        (0.0, -0.3, 0.2),
                        (0.0, 0.0, 0.2),
                    ]
                    for x, y, z in positions:
                        robot_api.look_at_world(
                            x,
                            y,
                            z,
                            duration=duration / len(positions),
                            perform_movement=True,
                        )
                    logger.info(
                        "✅ Look around exécuté via look_at_world (conforme SDK)",
                    )
                    return True

            elif behavior_name == "focus":
                # Se concentrer (determined/curious)
                if (
                    REACHY_UTILS_AVAILABLE
                    and create_head_pose
                    and hasattr(robot_api, "goto_target")
                ):
                    pose_focus = create_head_pose(
                        pitch=0.05 * intensity,
                        yaw=0.0,
                        degrees=False,
                    )
                    robot_api.goto_target(
                        head=pose_focus,
                        duration=duration,
                        method="minjerk",
                    )
                    logger.info("✅ Focus exécuté via goto_target (conforme SDK)")
                    return True

            elif "head_pose" in joints and "yaw_body" in joints:
                # Comportement générique avec tête + corps
                if (
                    REACHY_UTILS_AVAILABLE
                    and create_head_pose
                    and hasattr(robot_api, "goto_target")
                ):
                    # Mouvement combiné tête+corps (optimal SDK)
                    head_pitch = 0.1 * intensity * random.uniform(-1, 1)  # nosec B311
                    head_yaw = 0.1 * intensity * random.uniform(-1, 1)  # nosec B311
                    body_yaw = 0.15 * intensity * random.uniform(-1, 1)  # nosec B311

                    pose = create_head_pose(
                        pitch=head_pitch,
                        yaw=head_yaw,
                        degrees=False,
                    )
                    robot_api.goto_target(
                        head=pose,
                        body_yaw=body_yaw,
                        duration=duration,
                        method="minjerk",
                    )
                    logger.info(
                        (
                            f"✅ Comportement '{behavior_name}' exécuté "
                            f"via goto_target combiné (conforme SDK)"
                        ),
                    )
                    return True

            # Fallback: utiliser set_emotion si comportement correspond à une émotion
            emotion = behavior.get("emotion")
            if emotion and hasattr(robot_api, "set_emotion"):
                robot_api.set_emotion(emotion, intensity)
                logger.info("✅ Comportement exécuté via set_emotion (fallback)")
                return True

            logger.warning(
                (
                    f"⚠️ Comportement '{behavior_name}' non exécuté "
                    f"(robot_api ou méthodes SDK manquantes)"
                ),
            )
            return False

        except Exception as e:
            logger.exception(
                "❌ Erreur exécution comportement '%s': %s", behavior_name, e
            )
            return False


def main() -> None:
    """Test du module BBIA Adaptive Behavior."""
    # Initialisation
    adaptive_behavior = BBIAAdaptiveBehavior()

    # Test changement de contexte
    logging.info("🎭 Test changement de contexte...")
    adaptive_behavior.set_context("greeting")
    adaptive_behavior.set_emotion_state("happy", 0.8)

    # Test génération de comportement
    logging.info("\n🎭 Test génération comportement...")
    behavior = adaptive_behavior.generate_behavior("user_arrival")
    logging.info(f"Comportement généré: {behavior['name']}")
    logging.info(f"Description: {behavior['description']}")
    logging.info(f"Paramètres: {behavior['parameters']}")

    # Test comportement proactif
    logging.info("\n🚀 Test comportement proactif...")
    proactive = adaptive_behavior.get_proactive_behavior()
    if proactive:
        logging.info(f"Comportement proactif: {proactive['name']}")
    else:
        logging.info("Aucun comportement proactif généré")

    # Test adaptation feedback
    logging.info("\n🔄 Test adaptation feedback...")
    adaptive_behavior.adapt_to_feedback(behavior["id"], "positive", 0.8)

    # Statistiques
    logging.info(f"\n📊 Statistiques: {adaptive_behavior.get_behavior_statistics()}")


if __name__ == "__main__":
    main()
