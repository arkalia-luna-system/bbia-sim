#!/usr/bin/env python3

"""Comportement de photo booth pour BBIA.

Poses pré-définies, détection visage pour cadrage, compte à rebours,
capture photo automatique.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from bbia_sim.bbia_vision import BBIAVision
    from bbia_sim.robot_api import RobotAPI

try:
    from bbia_sim.bbia_voice import dire_texte
except ImportError:
    dire_texte = None  # type: ignore[assignment, misc]

from bbia_sim.behaviors.base import BBIABehavior

logger = logging.getLogger("BBIA")

# Poses pré-définies
PHOTO_POSES = {
    "happy": {"emotion": "happy", "body_yaw": 0.1, "description": "Pose joyeuse"},
    "cool": {"emotion": "neutral", "body_yaw": 0.0, "description": "Pose cool"},
    "surprised": {
        "emotion": "curious",
        "body_yaw": 0.05,
        "description": "Pose surprise",
    },
    "proud": {"emotion": "happy", "body_yaw": 0.12, "description": "Pose fière"},
}


class PhotoBoothBehavior(BBIABehavior):
    """Comportement de photo booth.

    Prend des photos avec poses expressives et détection de visage.
    """

    def __init__(
        self,
        vision: BBIAVision | None = None,
        robot_api: RobotAPI | None = None,
    ) -> None:
        """Initialise le comportement de photo booth.

        Args:
            vision: Instance BBIAVision pour détection visage
            robot_api: Instance RobotAPI pour contrôler le robot

        """
        super().__init__(
            "photo_booth",
            "Mode photo avec poses expressives",
            robot_api=robot_api,
        )
        self.vision = vision
        self.priority = 6

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si robot_api disponible

        """
        return self.robot_api is not None

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le mode photo booth.

        Args:
            context: Contexte d'exécution (peut contenir pose, countdown, auto_capture)

        Returns:
            True si le mode photo a réussi

        """
        if not self.robot_api:
            logger.error("Robot API non disponible")
            return False

        # Paramètres
        pose_name = context.get("pose", "happy")
        countdown = context.get("countdown", True)
        auto_capture = context.get("auto_capture", True)
        num_photos = context.get("num_photos", 1)

        logger.info(f"Démarrage photo booth - Pose: {pose_name}, Photos: {num_photos}")

        try:
            for photo_num in range(num_photos):
                if num_photos > 1:
                    logger.info(f"Photo {photo_num + 1}/{num_photos}")

                # Détection visage pour cadrage
                if self.vision:
                    self._frame_face()

                # Prendre la pose
                self._take_pose(pose_name)

                # Compte à rebours
                if countdown:
                    self._countdown()

                # Capture photo
                if auto_capture:
                    self._capture_photo()

                # Pause entre photos
                if photo_num < num_photos - 1:
                    time.sleep(2.0)

            logger.info("Session photo booth terminée")

        except KeyboardInterrupt:
            logger.info("Photo booth interrompu par l'utilisateur")
        except Exception as e:
            logger.error(f"Erreur durant photo booth: {e}")
            return False

        return True

    def _frame_face(self) -> None:
        """Cadre le visage dans l'image."""
        if not self.vision:
            return

        try:
            faces = self.vision.detect_faces()
            if faces and len(faces) > 0:
                face = faces[0]
                bbox = face.get("bbox", {})

                if bbox and self.robot_api and hasattr(self.robot_api, "look_at_image"):
                    center_x = int(bbox.get("center_x", 320))
                    center_y = int(bbox.get("center_y", 240))

                    if 0 <= center_x <= 640 and 0 <= center_y <= 480:
                        look_at_image = getattr(self.robot_api, "look_at_image", None)
                        if look_at_image:
                            look_at_image(center_x, center_y, duration=0.8)
                            logger.debug(
                                f"Cadrage visage vers ({center_x}, {center_y})"
                            )

        except Exception as e:
            logger.warning(f"Erreur cadrage visage: {e}")

    def _take_pose(self, pose_name: str) -> None:
        """Prend une pose pré-définie.

        Args:
            pose_name: Nom de la pose (happy, cool, surprised, proud)

        """
        if not self.robot_api:
            return

        pose = PHOTO_POSES.get(pose_name, PHOTO_POSES["happy"])

        try:
            # Appliquer émotion
            if hasattr(self.robot_api, "set_emotion"):
                # Cast explicite pour mypy
                emotion_str: str = str(pose["emotion"])
                self.robot_api.set_emotion(emotion_str, 0.8)

            # Appliquer mouvement corps
            if hasattr(self.robot_api, "goto_target"):
                # Cast explicite pour mypy
                body_yaw_val: float | None = (
                    float(pose["body_yaw"])
                    if isinstance(pose["body_yaw"], int | float)
                    else None
                )
                self.robot_api.goto_target(
                    body_yaw=body_yaw_val,
                    duration=0.6,
                    method="minjerk",
                )

            logger.debug(f"Pose '{pose_name}' appliquée: {pose['description']}")
            time.sleep(0.8)

        except Exception as e:
            logger.warning(f"Erreur prise pose {pose_name}: {e}")

    def _countdown(self) -> None:
        """Compte à rebours avant photo."""
        if dire_texte is None:
            return

        try:
            for i in range(3, 0, -1):
                if dire_texte is not None:
                    dire_texte(str(i), robot_api=self.robot_api)
                time.sleep(1.0)

            if dire_texte is not None:
                dire_texte("Souriez !", robot_api=self.robot_api)
            time.sleep(0.5)

        except Exception as e:
            logger.warning(f"Erreur compte à rebours: {e}")

    def _capture_photo(self) -> None:
        """Capture une photo."""
        if not self.robot_api:
            return

        try:
            # Utiliser robot.media.camera si disponible
            if hasattr(self.robot_api, "get_image"):
                get_image = getattr(self.robot_api, "get_image", None)
                if get_image:
                    image = get_image()
                    if image is not None:
                        logger.info("Photo capturée avec succès")
                        if dire_texte is not None:
                            dire_texte("Photo prise !", robot_api=self.robot_api)
                    else:
                        logger.warning("Échec capture photo")
                else:
                    logger.warning("get_image non disponible - simulation capture")
                    if dire_texte is not None:
                        dire_texte("Photo !", robot_api=self.robot_api)
            else:
                logger.warning("get_image non disponible - simulation capture")
                if dire_texte is not None:
                    dire_texte("Photo !", robot_api=self.robot_api)

        except Exception as e:
            logger.error(f"Erreur capture photo: {e}")
