#!/usr/bin/env python3
"""Module de détection de postures/gestes avec MediaPipe Pose.

Fonctionnalités :
- Détection de la posture complète du corps (33 points clés)
- Détection de gestes (bras levés, debout/assis, etc.)
- Compatible SDK Reachy Mini (pas de conflit)
- Déjà installé via MediaPipe (pas besoin d'installer autre chose)
"""

import logging
from typing import Any

import numpy as np
import numpy.typing as npt

logger = logging.getLogger(__name__)

# Import conditionnel MediaPipe Pose
MEDIAPIPE_POSE_AVAILABLE = False
try:
    import mediapipe as mp

    # Vérifier que mp.solutions existe (peut être absent même si import réussit)
    if hasattr(mp, "solutions"):
        MEDIAPIPE_POSE_AVAILABLE = True
    else:
        MEDIAPIPE_POSE_AVAILABLE = False
        mp = None
        logger.debug(
            "MediaPipe importé mais solutions non disponible. "
            "Installer avec: pip install mediapipe"
        )
except ImportError:
    mp = None
    logger.debug("MediaPipe non disponible. Installer avec: pip install mediapipe")


class BBIAPoseDetection:
    """Module de détection de postures et gestes avec MediaPipe Pose.

    Permet à BBIA de détecter la posture complète des personnes,
    leurs gestes (bras levés, debout/assis, etc.).
    """

    def __init__(
        self,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
        model_complexity: int = 1,
    ) -> None:
        """Initialise le module de détection de pose.

        Args:
            min_detection_confidence: Confiance minimale pour détection (0.0-1.0)
            min_tracking_confidence: Confiance minimale pour tracking (0.0-1.0)
            model_complexity: Complexité du modèle (0=rapide, 1=équilibré, 2=précis)

        """
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        self.model_complexity = model_complexity
        self.pose_detector = None
        self.is_initialized = False

        if not MEDIAPIPE_POSE_AVAILABLE or mp is None:
            logger.warning(
                "⚠️ MediaPipe non disponible. Installer avec: pip install mediapipe",
            )
            return

        try:
            # Vérifier que mp a l'attribut solutions avant de l'utiliser
            if not hasattr(mp, "solutions"):
                raise AttributeError("mediapipe module has no attribute 'solutions'")
            self.pose_detector = mp.solutions.pose.Pose(
                static_image_mode=False,
                model_complexity=model_complexity,
                enable_segmentation=False,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )
            logger.info(
                f"✅ BBIAPoseDetection initialisé (complexité: {model_complexity})",
            )
            self.is_initialized = True
        except (AttributeError, ImportError, RuntimeError) as e:
            logger.error("❌ Erreur initialisation MediaPipe Pose (critique): %s", e)
        except Exception as e:  # noqa: BLE001 - Erreur initialisation MediaPipe Pose
            logger.error("❌ Erreur initialisation MediaPipe Pose (critique): %s", e)

    def detect_pose(self, image: npt.NDArray[np.uint8]) -> dict[str, Any] | None:
        """Détecte la posture complète dans une image.

        Args:
            image: Image numpy array (BGR ou RGB)

        Returns:
            Dict avec 'landmarks' (33 points), 'gestures' (gestes détectés),
            'posture' (debout/assis), ou None si pas de personne

        """
        if not self.is_initialized or not self.pose_detector:
            return None

        try:
            # MediaPipe nécessite RGB
            if len(image.shape) == 3 and image.shape[2] == 3:
                # Vérifier si BGR (OpenCV) ou RGB
                import cv2

                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) if cv2 else image
            else:
                image_rgb = image

            results = self.pose_detector.process(image_rgb)

            if not results.pose_landmarks:
                return None

            # Extraire les landmarks (33 points)
            landmarks = []
            height, width = image.shape[:2]

            for landmark in results.pose_landmarks.landmark:
                landmarks.append(
                    {
                        "x": landmark.x,
                        "y": landmark.y,
                        "z": landmark.z,
                        "visibility": landmark.visibility,
                        "pixel_x": int(landmark.x * width),
                        "pixel_y": int(landmark.y * height),
                    },
                )

            # Détecter les gestes et posture
            gestures = self._detect_gestures(landmarks)
            posture = self._detect_posture(landmarks)

            return {
                "landmarks": landmarks,
                "gestures": gestures,
                "posture": posture,
                "num_landmarks": len(landmarks),
            }

        except Exception as e:  # noqa: BLE001 - Erreur détection pose
            logger.error("❌ Erreur détection pose (critique): %s", e)
            return None

    def _detect_gestures(self, landmarks: list[dict[str, Any]]) -> dict[str, bool]:
        """Détecte les gestes à partir des landmarks.

        Args:
            landmarks: Liste des 33 landmarks MediaPipe

        Returns:
            Dict avec gestes détectés (bras_levés, debout, etc.)

        """
        if len(landmarks) < 33:
            return {}

        gestures = {
            "bras_levés": False,
            "bras_baissés": True,
            "bras_droit_levé": False,
            "bras_gauche_levé": False,
            "mains_sur_tete": False,
        }

        # Indices MediaPipe Pose (simplifiés)
        # Épaules : 11 (gauche), 12 (droite)
        # Poignets : 15 (gauche), 16 (droite)
        # Coude gauche : 13, coude droit : 14
        # Hanches : 23 (gauche), 24 (droite)
        # Genoux : 25 (gauche), 26 (droite)

        try:
            # Bras levés (poignet au-dessus de l'épaule)
            epaule_gauche = landmarks[11]
            epaule_droite = landmarks[12]
            poignet_gauche = landmarks[15]
            poignet_droit = landmarks[16]

            # Bras gauche levé
            if poignet_gauche["y"] < epaule_gauche["y"] - 0.05:
                gestures["bras_gauche_levé"] = True
                gestures["bras_baissés"] = False

            # Bras droit levé
            if poignet_droit["y"] < epaule_droite["y"] - 0.05:
                gestures["bras_droit_levé"] = True
                gestures["bras_baissés"] = False

            # Bras levés (les deux)
            if gestures["bras_gauche_levé"] and gestures["bras_droit_levé"]:
                gestures["bras_levés"] = True

            # Mains sur tête (approximatif : poignets très haut)
            tete_y = (
                landmarks[0]["y"] if len(landmarks) > 0 else 0.5
            )  # Landmark 0 = nez
            if poignet_gauche["y"] < tete_y + 0.1 or poignet_droit["y"] < tete_y + 0.1:
                gestures["mains_sur_tete"] = True

        except (IndexError, KeyError) as e:
            logger.debug("Erreur détection gestes (landmarks incomplets): %s", e)

        return gestures

    def _detect_posture(self, landmarks: list[dict[str, Any]]) -> str:
        """Détecte la posture principale (debout/assis).

        Args:
            landmarks: Liste des 33 landmarks MediaPipe

        Returns:
            "debout", "assis", ou "inconnu"

        """
        if len(landmarks) < 33:
            return "inconnu"

        try:
            # Hanches : 23 (gauche), 24 (droite)
            # Genoux : 25 (gauche), 26 (droite)
            # Chevilles : 27 (gauche), 28 (droite)

            hanche_gauche = landmarks[23]
            hanche_droite = landmarks[24]
            genou_gauche = landmarks[25]
            genou_droit = landmarks[26]

            # Distance moyenne hanche-cheville (debout) vs hanche-genou (assis)
            # Simplification : si genoux très bas par rapport aux hanches
            # = probablement assis
            avg_hanche_y = (hanche_gauche["y"] + hanche_droite["y"]) / 2
            avg_genou_y = (genou_gauche["y"] + genou_droit["y"]) / 2

            # Si genoux proches des hanches (différence < 0.15) = probablement assis
            if abs(avg_hanche_y - avg_genou_y) < 0.15:
                return "assis"

            # Sinon = probablement debout
            return "debout"

        except (IndexError, KeyError) as e:
            logger.debug("Erreur détection posture (landmarks incomplets): %s", e)
            return "inconnu"


def create_pose_detector(
    min_detection_confidence: float = 0.5,
    min_tracking_confidence: float = 0.5,
    model_complexity: int = 1,
) -> BBIAPoseDetection | None:
    """Factory function pour créer une instance BBIAPoseDetection.

    Args:
        min_detection_confidence: Confiance minimale pour détection
        min_tracking_confidence: Confiance minimale pour tracking
        model_complexity: Complexité du modèle (0=rapide, 1=équilibré, 2=précis)

    Returns:
        Instance BBIAPoseDetection ou None si MediaPipe non disponible

    """
    if not MEDIAPIPE_POSE_AVAILABLE:
        logger.warning("⚠️ MediaPipe non disponible")
        return None

    return BBIAPoseDetection(
        min_detection_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence,
        model_complexity=model_complexity,
    )


# Test rapide
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    logger.info("🧪 Test module Pose Detection BBIA")
    logger.info("=" * 40)

    pose_detector = BBIAPoseDetection()

    if pose_detector.is_initialized:
        logger.info("✅ Module Pose Detection créé")
        logger.info("   • Initialisé: %s", pose_detector.is_initialized)
        logger.info("   • Complexité modèle: %s", pose_detector.model_complexity)
    else:
        logger.info("❌ Impossible de créer le module (MediaPipe non disponible)")
        logger.info("   Installer avec: pip install mediapipe")
