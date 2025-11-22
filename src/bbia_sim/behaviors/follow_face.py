#!/usr/bin/env python3
"""Comportement de suivi de visage amélioré pour BBIA.

Améliorations :
- Précision suivi améliorée
- Émotions selon distance visage
- Réaction si visage disparaît
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from bbia_sim.bbia_vision import BBIAVision
    from bbia_sim.robot_api import RobotAPI

try:
    from bbia_sim.bbia_emotions import BBIAEmotions
    from bbia_sim.bbia_voice import dire_texte
except ImportError:
    BBIAEmotions = None  # type: ignore[assignment, misc]
    dire_texte = None  # type: ignore[assignment, misc]

from bbia_sim.behaviors.base import BBIABehavior

logger = logging.getLogger("BBIA")


class FollowFaceBehavior(BBIABehavior):
    """Comportement de suivi de visage amélioré.

    Suit un visage détecté avec MediaPipe, ajuste les émotions
    selon la distance, et réagit si le visage disparaît.
    """

    def __init__(
        self,
        vision: BBIAVision | None = None,
        robot_api: RobotAPI | None = None,
    ) -> None:
        """Initialise le comportement de suivi de visage.

        Args:
            vision: Instance BBIAVision pour détection visage
            robot_api: Instance RobotAPI pour contrôler le robot

        """
        super().__init__(
            "follow_face",
            "Suivi de visage avec émotions adaptatives",
            robot_api=robot_api,
        )
        self.vision = vision
        self.priority = 6
        self.is_tracking = False
        self.last_face_time: float | None = None
        self.face_lost_threshold = 3.0  # secondes avant réaction

        # Initialiser émotions si disponible
        if BBIAEmotions is not None:
            self.emotions = BBIAEmotions()
        else:
            self.emotions = None

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si la vision est disponible

        """
        if not self.vision:
            logger.warning("Vision non disponible pour suivi visage")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le suivi de visage.

        Args:
            context: Contexte d'exécution (peut contenir duration)

        Returns:
            True si le suivi a réussi

        """
        if not self.vision:
            logger.error("Vision non disponible")
            return False

        duration = context.get("duration", 30.0)  # 30 secondes par défaut
        logger.info("Démarrage suivi visage pour %ss", duration)

        self.is_tracking = True
        start_time = time.time()
        face_detected_count = 0
        face_lost_count = 0

        try:
            while self.is_tracking and (time.time() - start_time) < duration:
                # Détecter visages
                faces = self.vision.detect_faces()

                if faces and len(faces) > 0:
                    # Prendre le visage le plus proche (premier dans la liste)
                    face = faces[0]
                    face_detected_count += 1
                    self.last_face_time = time.time()

                    # Calculer distance estimée (basé sur taille bbox)
                    bbox = face.get("bbox", {})
                    if bbox:
                        width = bbox.get("width", 0)
                        height = bbox.get("height", 0)
                        # Estimation distance basée sur taille (plus grand = plus proche)
                        face_size = width * height
                        distance_estimate = max(
                            0.3,
                            min(2.0, 2.0 - (face_size / 50000)),
                        )

                        # Ajuster émotion selon distance
                        self._adjust_emotion_by_distance(distance_estimate)

                        # Suivre le visage
                        self._track_face(face)

                        logger.debug(
                            "Visage détecté - Distance estimée: %.2fm",
                            distance_estimate,
                        )
                    else:
                        # Suivre même sans bbox détaillé
                        self._track_face(face)

                # Aucun visage détecté
                elif self.last_face_time:
                    time_since_last = time.time() - self.last_face_time
                    if time_since_last > self.face_lost_threshold:
                        face_lost_count += 1
                        self._react_to_face_lost()
                        self.last_face_time = None

                time.sleep(0.1)  # 10 Hz pour suivi fluide

            logger.info(
                "Suivi terminé - Visages détectés: %d, Perdus: %d",
                face_detected_count,
                face_lost_count,
            )

        except KeyboardInterrupt:
            logger.info("Suivi interrompu par l'utilisateur")
        except Exception:
            logger.exception("Erreur durant suivi visage")
        finally:
            self.is_tracking = False
            # Retour à l'émotion neutre
            if self.robot_api and hasattr(self.robot_api, "set_emotion"):
                self.robot_api.set_emotion("neutral", 0.5)

        return True

    def _adjust_emotion_by_distance(self, distance: float) -> None:
        """Ajuste l'émotion selon la distance du visage.

        Args:
            distance: Distance estimée en mètres

        """
        if not self.robot_api or not hasattr(self.robot_api, "set_emotion"):
            return

        try:
            if distance < 0.5:
                # Très proche - émotion excited
                emotion = "excited"
                intensity = 0.8
            elif distance < 1.0:
                # Proche - émotion happy
                emotion = "happy"
                intensity = 0.7
            elif distance < 1.5:
                # Moyenne distance - émotion curious
                emotion = "curious"
                intensity = 0.6
            else:
                # Loin - émotion neutral
                emotion = "neutral"
                intensity = 0.5

            self.robot_api.set_emotion(emotion, intensity)
            logger.debug("Émotion ajustée: %s (intensité: %s)", emotion, intensity)

        except Exception as e:
            logger.warning("Erreur ajustement émotion: %s", e)

    def _track_face(self, face: dict[str, Any]) -> None:
        """Suit un visage détecté.

        Args:
            face: Dictionnaire contenant les informations du visage

        """
        if not self.robot_api:
            return

        try:
            # Utiliser look_at_image si bbox disponible
            bbox = face.get("bbox", {})
            if bbox and hasattr(self.robot_api, "look_at_image"):
                center_x = int(bbox.get("center_x", 320))
                center_y = int(bbox.get("center_y", 240))

                # Validation coordonnées
                if 0 <= center_x <= 640 and 0 <= center_y <= 480:
                    self.robot_api.look_at_image(center_x, center_y, duration=0.5)
                    logger.debug("Suivi visage vers (%s, %s)", center_x, center_y)

            # Alternative: utiliser position 3D si disponible
            elif face.get("position") and hasattr(self.robot_api, "look_at_world"):
                pos = face["position"]
                x = float(pos.get("x", 0.2))
                y = float(pos.get("y", 0.0))
                z = float(pos.get("z", 0.0))

                # Validation coordonnées
                if -2.0 <= x <= 2.0 and -2.0 <= y <= 2.0 and -1.0 <= z <= 1.0:
                    self.robot_api.look_at_world(x, y, z, duration=0.5)
                    logger.debug("Suivi visage vers 3D (%.2f, %.2f, %.2f)", x, y, z)

        except Exception as e:
            logger.warning("Erreur suivi visage: %s", e)

    def _react_to_face_lost(self) -> None:
        """Réagit quand le visage est perdu."""
        if dire_texte is None:
            return

        reactions = [
            "Où êtes-vous parti ?",
            "Je ne vous vois plus...",
            "Revenez dans mon champ de vision !",
            "Je vous cherche...",
        ]

        import secrets

        reaction = secrets.choice(reactions)
        dire_texte(reaction, robot_api=self.robot_api)
        logger.info("Réaction visage perdu: %s", reaction)

        # Émotion curious pour chercher
        if self.robot_api and hasattr(self.robot_api, "set_emotion"):
            self.robot_api.set_emotion("curious", 0.6)

    def stop(self) -> None:
        """Arrête le suivi."""
        super().stop()
        self.is_tracking = False
