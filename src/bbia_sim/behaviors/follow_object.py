#!/usr/bin/env python3
"""Comportement de suivi d'objet amélioré pour BBIA.

Améliorations :
- Suivi multi-objets
- Priorisation objets (personne > objet)
- Réaction si objet perdu
"""

from __future__ import annotations

import logging
import secrets
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


class FollowObjectBehavior(BBIABehavior):
    """Comportement de suivi d'objet amélioré.

    Suit des objets détectés avec YOLO, priorise les personnes,
    et réagit si l'objet est perdu.
    """

    # Priorités d'objets (plus élevé = plus prioritaire)
    OBJECT_PRIORITIES: dict[str, int] = {
        "person": 10,
        "human": 10,
        "face": 9,
        "hand": 8,
        "phone": 5,
        "book": 4,
        "cup": 3,
        "bottle": 3,
        "default": 1,
    }

    def __init__(
        self,
        vision: BBIAVision | None = None,
        robot_api: RobotAPI | None = None,
    ) -> None:
        """Initialise le comportement de suivi d'objet.

        Args:
            vision: Instance BBIAVision pour détection objets
            robot_api: Instance RobotAPI pour contrôler le robot

        """
        super().__init__(
            "follow_object",
            "Suivi d'objets avec priorisation intelligente",
            robot_api=robot_api,
        )
        self.vision = vision
        self.priority = 6
        self.is_tracking = False
        self.current_target: dict[str, Any] | None = None
        self.last_object_time: float | None = None
        self.object_lost_threshold = 2.0  # secondes avant réaction

    def can_execute(self, context: dict[str, Any]) -> bool:
        """Vérifie si le comportement peut être exécuté.

        Args:
            context: Contexte d'exécution

        Returns:
            True si la vision est disponible

        """
        if not self.vision:
            logger.warning("Vision non disponible pour suivi objet")
            return False
        return True

    def execute(self, context: dict[str, Any]) -> bool:
        """Exécute le suivi d'objet.

        Args:
            context: Contexte d'exécution (peut contenir duration, target_object)

        Returns:
            True si le suivi a réussi

        """
        if not self.vision:
            logger.error("Vision non disponible")
            return False

        duration = context.get("duration", 30.0)  # 30 secondes par défaut
        target_object_name = context.get("target_object")  # Objet spécifique à suivre

        target_info = f" (cible: {target_object_name})" if target_object_name else ""
        logger.info("Démarrage suivi objet pour %ss%s", duration, target_info)

        self.is_tracking = True
        start_time = time.time()
        objects_detected_count = 0
        objects_lost_count = 0

        try:
            while self.is_tracking and (time.time() - start_time) < duration:
                # Scanner l'environnement
                result = self.vision.scan_environment()
                objects = result.get("objects", [])

                if objects:
                    # Sélectionner l'objet à suivre
                    selected_object = self._select_object_to_track(
                        objects,
                        target_object_name,
                    )

                    if selected_object:
                        objects_detected_count += 1
                        self.current_target = selected_object
                        self.last_object_time = time.time()

                        object_name = selected_object.get("name", "objet")
                        logger.debug("Suivi de: %s", object_name)

                        # Suivre l'objet
                        self._track_object(selected_object)

                        # Commentaire vocal occasionnel (toutes les 5 secondes)
                        if objects_detected_count % 50 == 0:  # ~5 secondes à 10 Hz
                            self._comment_on_object(selected_object)

                    else:
                        # Aucun objet sélectionnable
                        if self.current_target:
                            # Objet perdu
                            if self.last_object_time:
                                time_since_last = time.time() - self.last_object_time
                                if time_since_last > self.object_lost_threshold:
                                    objects_lost_count += 1
                                    self._react_to_object_lost()
                                    self.current_target = None
                                    self.last_object_time = None

                else:
                    # Aucun objet détecté
                    if self.current_target:
                        if self.last_object_time:
                            time_since_last = time.time() - self.last_object_time
                            if time_since_last > self.object_lost_threshold:
                                objects_lost_count += 1
                                self._react_to_object_lost()
                                self.current_target = None
                                self.last_object_time = None

                time.sleep(0.1)  # 10 Hz pour suivi fluide

            logger.info(
                f"Suivi terminé - Objets détectés: {objects_detected_count}, "
                f"Perdus: {objects_lost_count}",
            )

        except KeyboardInterrupt:
            logger.info("Suivi interrompu par l'utilisateur")
        except Exception as e:
            logger.exception("Erreur durant suivi objet: %s", e)
        finally:
            self.is_tracking = False
            self.current_target = None
            # Retour à l'émotion neutre
            if self.robot_api and hasattr(self.robot_api, "set_emotion"):
                self.robot_api.set_emotion("neutral", 0.5)

        return True

    def _select_object_to_track(
        self,
        objects: list[dict[str, Any]],
        target_name: str | None = None,
    ) -> dict[str, Any] | None:
        """Sélectionne l'objet à suivre selon priorité.

        Args:
            objects: Liste d'objets détectés
            target_name: Nom d'objet spécifique à suivre (optionnel)

        Returns:
            Objet sélectionné ou None

        """
        if not objects:
            return None

        # Si objet spécifique demandé, le chercher
        if target_name:
            for obj in objects:
                if obj.get("name", "").lower() == target_name.lower():
                    return obj

        # Sinon, prioriser selon OBJECT_PRIORITIES
        best_object: dict[str, Any] | None = None
        best_priority = -1

        for obj in objects:
            obj_name = obj.get("name", "default").lower()
            priority = self.OBJECT_PRIORITIES.get(
                obj_name, self.OBJECT_PRIORITIES["default"]
            )

            # Si même priorité, prendre le plus proche (plus grand bbox)
            if priority == best_priority and best_object:
                bbox_current = obj.get("bbox", {})
                bbox_best = best_object.get("bbox", {})
                size_current = bbox_current.get("width", 0) * bbox_current.get(
                    "height", 0
                )
                size_best = bbox_best.get("width", 0) * bbox_best.get("height", 0)
                if size_current > size_best:
                    best_object = obj
            elif priority > best_priority:
                best_priority = priority
                best_object = obj

        return best_object

    def _track_object(self, obj: dict[str, Any]) -> None:
        """Suit un objet détecté.

        Args:
            obj: Dictionnaire contenant les informations de l'objet

        """
        if not self.robot_api:
            return

        try:
            # Utiliser look_at_image si bbox disponible
            bbox = obj.get("bbox", {})
            if bbox and hasattr(self.robot_api, "look_at_image"):
                center_x = int(bbox.get("center_x", 320))
                center_y = int(bbox.get("center_y", 240))

                # Validation coordonnées
                if 0 <= center_x <= 640 and 0 <= center_y <= 480:
                    self.robot_api.look_at_image(center_x, center_y, duration=0.5)
                    logger.debug("Suivi objet vers (%s, %s)", center_x, center_y)

            # Alternative: utiliser position 3D si disponible
            elif obj.get("position") and hasattr(self.robot_api, "look_at_world"):
                pos = obj["position"]
                x = float(pos.get("x", 0.2))
                y = float(pos.get("y", 0.0))
                z = float(pos.get("z", 0.0))

                # Validation coordonnées
                if -2.0 <= x <= 2.0 and -2.0 <= y <= 2.0 and -1.0 <= z <= 1.0:
                    self.robot_api.look_at_world(x, y, z, duration=0.5)
                    logger.debug("Suivi objet vers 3D (%s, %s, %s)", x:.2f, y:.2f, z:.2f)

            # Appliquer émotion curious pour suivi
            if hasattr(self.robot_api, "set_emotion"):
                self.robot_api.set_emotion("curious", 0.6)

        except Exception as e:
            logger.warning("Erreur suivi objet: %s", e)

    def _comment_on_object(self, obj: dict[str, Any]) -> None:
        """Commentaire vocal sur l'objet suivi.

        Args:
            obj: Objet suivi

        """
        if dire_texte is None:
            return

        obj_name = obj.get("name", "objet")
        comments = [
            f"Je vois {obj_name} !",
            f"Je regarde {obj_name}.",
            f"{obj_name} m'intrigue !",
            f"Intéressant, je vois {obj_name}.",
        ]

        comment = secrets.choice(comments)
        dire_texte(comment, robot_api=self.robot_api)
        logger.debug("Commentaire objet: %s", comment)

    def _react_to_object_lost(self) -> None:
        """Réagit quand l'objet est perdu."""
        if dire_texte is None:
            return

        if self.current_target:
            obj_name = self.current_target.get("name", "objet")
            reactions = [
                f"Où est passé {obj_name} ?",
                f"Je ne vois plus {obj_name}...",
                f"{obj_name} a disparu !",
            ]
        else:
            reactions = [
                "Où est l'objet ?",
                "Je ne vois plus l'objet...",
                "L'objet a disparu !",
            ]

        reaction = secrets.choice(reactions)
        dire_texte(reaction, robot_api=self.robot_api)
        logger.info("Réaction objet perdu: %s", reaction)

        # Émotion curious pour chercher
        if self.robot_api and hasattr(self.robot_api, "set_emotion"):
            self.robot_api.set_emotion("curious", 0.6)

    def stop(self) -> None:
        """Arrête le suivi."""
        super().stop()
        self.is_tracking = False
        self.current_target = None
