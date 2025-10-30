#!/usr/bin/env python3

"""BBIA Vision - Module de vision avancé pour Reachy Mini Wireless
Reconnaissance d'objets, détection de visages, suivi d'objets.
"""

import logging
import math
from datetime import datetime
from typing import Any, Optional, cast

import numpy as np
import numpy.typing as npt

logger = logging.getLogger(__name__)

# Réduction du bruit de logs TensorFlow/MediaPipe (avant tout import MediaPipe)
try:
    import os as _os  # noqa: F401

    _os.environ.setdefault("GLOG_minloglevel", "2")  # 0=INFO,1=WARNING,2=ERROR
    _os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")  # 1=WARNING,2=ERROR
    _os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")  # éviter logs GPU inutiles
except Exception:
    pass

# Import conditionnel pour YOLO et MediaPipe
try:
    from .vision_yolo import create_yolo_detector

    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    import mediapipe as mp

    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False

# Import conditionnel cv2 pour conversions couleur
try:
    import cv2

    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class BBIAVision:
    """Module de vision avancé pour BBIA.

    Utilise robot.media.camera si disponible (SDK Reachy Mini officiel),
    sinon utilise simulation pour compatibilité.
    """

    def __init__(self, robot_api: Optional[Any] = None) -> None:
        """
        Initialise le module de vision.

        Args:
            robot_api: Interface RobotAPI (optionnel) pour accès robot.media.camera
        """
        self.robot_api = robot_api
        self.camera_active = True
        self.vision_quality = "HD"
        self.detection_range = 3.0  # mètres
        self.objects_detected: list[dict[str, Any]] = []
        self.faces_detected: list[dict[str, Any]] = []
        self.tracking_active = False
        self.current_focus: Optional[dict[str, Any]] = None

        # Spécifications hardware réelles
        self.specs = {
            "camera": "Grand angle",
            "resolution": "1080p",
            "fov": "120°",
            "focus": "Auto",
            "night_vision": False,
        }

        # Vérifier disponibilité robot.media.camera
        self._camera_sdk_available = False
        self._camera = None
        if robot_api and hasattr(robot_api, "media") and robot_api.media:
            try:
                self._camera = getattr(robot_api.media, "camera", None)
                if self._camera is not None:
                    self._camera_sdk_available = True
                    logger.info("✅ Caméra SDK disponible: robot.media.camera")
            except Exception as e:
                logger.debug(f"Caméra SDK non disponible (fallback simulation): {e}")

        # Initialiser détecteurs (YOLO pour objets, MediaPipe pour visages)
        self.yolo_detector = None
        if YOLO_AVAILABLE and create_yolo_detector is not None:
            try:
                self.yolo_detector = create_yolo_detector(model_size="n")
                if self.yolo_detector:
                    self.yolo_detector.load_model()
                    logger.info("✅ Détecteur YOLO initialisé")
            except Exception as e:
                logger.warning(f"⚠️ YOLO non disponible: {e}")

        self.face_detector = None
        if MEDIAPIPE_AVAILABLE and mp:
            try:
                self.face_detector = mp.solutions.face_detection.FaceDetection(
                    model_selection=0, min_detection_confidence=0.5
                )
                logger.info("✅ Détecteur MediaPipe Face initialisé")
            except Exception as e:
                logger.warning(f"⚠️ MediaPipe non disponible: {e}")

    def _capture_image_from_camera(self) -> Optional[npt.NDArray[np.uint8]]:
        """Capture une image depuis robot.media.camera si disponible.

        CORRECTION EXPERTE: Validation robuste du format d'image SDK avec gestion
        des différents formats possibles (RGB, BGR, grayscale, etc.)

        Returns:
            Image numpy array (BGR pour compatibilité OpenCV) ou None si non disponible
        """
        if not self._camera_sdk_available or not self._camera:
            return None

        try:
            # OPTIMISATION SDK: Capturer image depuis robot.media.camera
            # Le SDK Reachy Mini expose généralement get_image() ou capture()
            # ou peut être un stream
            image = None

            if hasattr(self._camera, "get_image"):
                image = self._camera.get_image()
            elif hasattr(self._camera, "capture"):
                image = self._camera.capture()
            elif hasattr(self._camera, "read"):
                # Si camera expose read() comme OpenCV VideoCapture
                ret, image = self._camera.read()
                if not ret:
                    return None
            elif callable(self._camera):
                # Si camera est callable directement
                image = self._camera()
            else:
                logger.warning(
                    "⚠️ robot.media.camera disponible mais méthode de capture inconnue"
                )
                return None

            if image is None:
                return None

            # CORRECTION EXPERTE: Convertir en numpy array avec validation robuste
            if not isinstance(image, np.ndarray):
                try:
                    if hasattr(image, "toarray"):
                        image = image.toarray()
                    elif hasattr(image, "numpy"):
                        image = image.numpy()
                    elif hasattr(image, "__array__"):
                        # Support PIL Image, torch Tensor, etc.
                        image = np.array(image)
                    else:
                        logger.warning("Format d'image non supporté par SDK camera")
                        return None
                except Exception as e:
                    logger.debug(f"Erreur conversion image: {e}")
                    return None

            # CORRECTION EXPERTE: Validation format image (shape, dtype, channels)
            if not isinstance(image, np.ndarray):
                return None

            # Valider shape (doit être 2D ou 3D)
            if image.ndim < 2 or image.ndim > 3:
                logger.warning(
                    f"Format image invalide (ndim={image.ndim}), "
                    "attendu 2 ou 3 dimensions"
                )
                return None

            # CORRECTION EXPERTE: Convertir grayscale → BGR si nécessaire
            if not CV2_AVAILABLE or cv2 is None:
                # Fallback sans cv2: supposer format correct
                logger.debug("cv2 non disponible, format image supposé correct")
            else:
                # Convertir grayscale → BGR si nécessaire
                if image.ndim == 2:
                    # Grayscale: ajouter channel
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                elif image.ndim == 3:
                    # Image couleur: vérifier nombre de canaux
                    channels = image.shape[2]
                    if channels == 1:
                        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                    elif channels == 4:
                        # RGBA → BGR
                        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
                    elif channels == 3:
                        # CORRECTION EXPERTE: SDK retourne généralement RGB
                        # Mais OpenCV/YOLO attendent BGR, donc convertir RGB → BGR
                        # Si SDK retourne déjà BGR, cette conversion sera effectuée
                        # mais généralement les SDKs retournent RGB
                        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                else:
                    logger.warning(
                        f"Nombre de canaux invalide ({image.shape[2] if image.ndim == 3 else 'N/A'}), attendu 1, 3 ou 4"
                    )
                    return None

            # Validation finale: s'assurer que l'image est valide
            if image.size == 0:
                logger.warning("Image vide retournée par SDK camera")
                return None

            # Assurer dtype uint8 (format standard pour images)
            if image.dtype != np.uint8:
                try:
                    # Normaliser si float [0,1] → [0,255]
                    if image.dtype == np.float32 or image.dtype == np.float64:
                        if image.max() <= 1.0:
                            image = (image * 255).astype(np.uint8)
                        else:
                            image = image.astype(np.uint8)
                    else:
                        image = image.astype(np.uint8)
                except Exception as e:
                    logger.debug(f"Erreur conversion dtype: {e}")
                    return None

            logger.debug("✅ Image capturée depuis robot.media.camera (format validé)")
            # Type narrowing: image est maintenant np.ndarray après toutes les validations
            return cast(npt.NDArray[np.uint8], image)

        except Exception as e:
            logger.debug(f"Erreur capture caméra SDK (fallback simulation): {e}")

        return None

    def scan_environment(self) -> dict[str, Any]:
        """Scanne l'environnement et détecte les objets.

        Utilise robot.media.camera si disponible (SDK officiel) avec détection YOLO/MediaPipe,
        sinon utilise simulation pour compatibilité.
        """
        # OPTIMISATION SDK: Utiliser robot.media.camera avec détection réelle si disponible
        image = self._capture_image_from_camera()

        if image is not None:
            # Détection réelle depuis caméra SDK
            objects = []
            faces = []

            # Détection d'objets avec YOLO
            if self.yolo_detector and self.yolo_detector.is_loaded:
                try:
                    detections = self.yolo_detector.detect_objects(image)
                    for det in detections:
                        # Convertir détection YOLO au format BBIA
                        obj = {
                            "name": det.get("class", "objet"),
                            "distance": det.get(
                                "distance", 1.5
                            ),  # Estimation basée sur bbox
                            "confidence": det.get("confidence", 0.5),
                            "position": (
                                det.get("bbox", {}).get("center_x", 320) / 640.0,
                                det.get("bbox", {}).get("center_y", 240) / 480.0,
                            ),
                            "bbox": det.get("bbox", {}),
                        }
                        objects.append(obj)
                except Exception as e:
                    logger.warning(f"Erreur détection YOLO: {e}")

            # Détection de visages avec MediaPipe
            if self.face_detector:
                try:
                    import cv2

                    # MediaPipe nécessite RGB
                    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    results = self.face_detector.process(image_rgb)

                    if results.detections:
                        height, width = image.shape[:2]
                        for detection in results.detections:
                            bbox = detection.location_data.relative_bounding_box
                            face = {
                                "name": "humain",
                                "distance": 1.5,  # Estimation basée sur bbox size
                                "confidence": (
                                    detection.score[0] if detection.score else 0.8
                                ),
                                "emotion": (
                                    "neutral"
                                ),  # MediaPipe ne détecte pas l'émotion directement
                                "position": (
                                    bbox.xmin + bbox.width / 2,
                                    bbox.ymin + bbox.height / 2,
                                ),
                                "bbox": {
                                    "x": int(bbox.xmin * width),
                                    "y": int(bbox.ymin * height),
                                    "width": int(bbox.width * width),
                                    "height": int(bbox.height * height),
                                },
                            }
                            faces.append(face)
                except Exception as e:
                    logger.warning(f"Erreur détection MediaPipe: {e}")

            if objects or faces:
                logger.info(
                    f"✅ Détection réelle: {len(objects)} objets, {len(faces)} visages"
                )
                self.objects_detected = objects
                self.faces_detected = faces
                return {
                    "objects": objects,
                    "faces": faces,
                    "timestamp": datetime.now().isoformat(),
                    "source": "camera_sdk",
                }

        # Simulation de détection d'objets (fallback ou mode sim)
        objects = [
            {
                "name": "chaise",
                "distance": 1.2,
                "confidence": 0.95,
                "position": (0.5, 0.3),
            },
            {
                "name": "table",
                "distance": 2.1,
                "confidence": 0.92,
                "position": (0.8, 0.1),
            },
            {
                "name": "livre",
                "distance": 0.8,
                "confidence": 0.88,
                "position": (0.2, 0.4),
            },
            {
                "name": "fenêtre",
                "distance": 3.0,
                "confidence": 0.97,
                "position": (1.0, 0.5),
            },
            {
                "name": "plante",
                "distance": 1.5,
                "confidence": 0.85,
                "position": (0.6, 0.2),
            },
        ]

        # Simulation de détection de visages
        faces = [
            {
                "name": "humain",
                "distance": 1.8,
                "confidence": 0.94,
                "emotion": "neutral",
                "position": (0.4, 0.3),
            },
            {
                "name": "humain",
                "distance": 2.3,
                "confidence": 0.91,
                "emotion": "happy",
                "position": (0.7, 0.2),
            },
        ]

        self.objects_detected = objects
        self.faces_detected = faces

        return {
            "objects": objects,
            "faces": faces,
            "timestamp": datetime.now().isoformat(),
            "source": "simulation",  # Fallback simulation
        }

    def recognize_object(self, object_name: str) -> Optional[dict[str, Any]]:
        """Reconnaît un objet spécifique."""
        for obj in self.objects_detected:
            if obj["name"] == object_name:
                return obj

        return None

    def detect_faces(self) -> list[dict[str, Any]]:
        """Détecte les visages dans le champ de vision."""
        if not self.faces_detected:
            self.scan_environment()

        for _face in self.faces_detected:
            pass

        return self.faces_detected

    def track_object(self, object_name: str) -> bool:
        """Active le suivi d'un objet."""
        obj = self.recognize_object(object_name)
        if obj:
            self.tracking_active = True
            self.current_focus = obj
            return True
        else:
            return False

    def stop_tracking(self) -> None:
        """Arrête le suivi d'objet."""
        if self.tracking_active:
            self.tracking_active = False
            self.current_focus = None
        else:
            pass

    def get_focus_status(self) -> dict[str, Any]:
        """Retourne le statut du focus actuel."""
        return {
            "tracking_active": self.tracking_active,
            "current_focus": self.current_focus,
            "objects_count": len(self.objects_detected),
            "faces_count": len(self.faces_detected),
        }

    def analyze_emotion(self, face_data: dict[str, Any]) -> str:
        """Analyse l'émotion d'un visage."""
        detected_emotion = face_data.get("emotion", "neutral")
        return str(detected_emotion)

    def calculate_distance(self, object_position: tuple[float, float]) -> float:
        """Calcule la distance d'un objet."""
        # Simulation simple basée sur la position
        x, y = object_position
        distance = math.sqrt(x**2 + y**2)
        return distance

    def get_vision_stats(self) -> dict[str, Any]:
        """Retourne les statistiques de vision."""
        return {
            "camera_active": self.camera_active,
            "vision_quality": self.vision_quality,
            "detection_range": self.detection_range,
            "objects_detected": len(self.objects_detected),
            "faces_detected": len(self.faces_detected),
            "tracking_active": self.tracking_active,
            "specs": self.specs,
        }


def main() -> None:
    """Test du module BBIA Vision."""
    # Créer l'instance
    vision = BBIAVision()

    # Test scan environnement

    # Test reconnaissance objet
    vision.recognize_object("chaise")

    # Test détection visages

    # Test suivi objet
    vision.track_object("livre")

    # Test statuts
    vision.get_focus_status()
    vision.get_vision_stats()

    # Arrêt suivi
    vision.stop_tracking()


if __name__ == "__main__":
    main()
