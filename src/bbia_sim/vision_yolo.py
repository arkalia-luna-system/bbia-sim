#!/usr/bin/env python3
"""bbia_vision_yolo.py - Module YOLOv8n pour BBIA
Détection d'objets légère avec YOLOv8n (optionnel)
"""

import logging
import threading
from typing import Any

import numpy as np

try:
    import cv2
    from ultralytics import YOLO

    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    # Variables globales pour éviter les erreurs de type
    YOLO = None  # type: ignore
    cv2 = None  # type: ignore

logger = logging.getLogger(__name__)

# OPTIMISATION RAM: Cache global LRU pour modèles YOLO (max 2 modèles: n, s)
_yolo_model_cache: dict[str, Any] = {}
_yolo_model_last_used: dict[str, float] = {}
_yolo_cache_lock = threading.Lock()
_MAX_YOLO_CACHE_SIZE = 2  # OPTIMISATION RAM: Limiter à 2 modèles YOLO max (n, s)

# OPTIMISATION PERFORMANCE: Cache global pour MediaPipe FaceDetection (une seule instance suffit)
_mediapipe_face_detection_cache: Any | None = None
_mediapipe_cache_lock = threading.Lock()

# Réduction du bruit de logs TensorFlow/MediaPipe (avant import potentiel de MediaPipe)
try:
    import os as _os

    _os.environ.setdefault("GLOG_minloglevel", "2")
    _os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
    _os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")
except Exception:
    pass


class YOLODetector:
    """Module de détection d'objets utilisant YOLOv8n."""

    def __init__(self, model_size: str = "n", confidence_threshold: float = 0.25):
        """Initialise le détecteur YOLO.

        Args:
            model_size: Taille du modèle ("n", "s", "m", "l", "x")
            confidence_threshold: Seuil de confiance pour les détections

        """
        self.model_size = model_size
        self.confidence_threshold = confidence_threshold
        self.model: Any | None = None
        self.is_loaded = False

        # Classes d'intérêt pour BBIA
        self.target_classes = {
            "person": "look_at",
            "face": "look_at",
            "hand": "look_at",
            "phone": "curious",
            "book": "curious",
            "cup": "curious",
            "bottle": "curious",
        }

        if not YOLO_AVAILABLE:
            logger.warning("⚠️ YOLO non disponible. Fallback vers détection basique.")
            return

        logger.info(
            f"🔍 Initialisation YOLO détecteur (modèle: {model_size}, seuil: {confidence_threshold})",
        )

    def load_model(self) -> bool:
        """Charge le modèle YOLO (utilise cache global si disponible)."""
        if not YOLO_AVAILABLE:
            return False

        # OPTIMISATION PERFORMANCE: Utiliser cache global pour éviter chargements répétés
        cache_key = f"yolov8{self.model_size}"

        global _yolo_model_cache, _yolo_model_last_used
        import time as time_module

        with _yolo_cache_lock:
            if cache_key in _yolo_model_cache:
                logger.debug(f"♻️ Réutilisation modèle YOLO depuis cache ({cache_key})")
                self.model = _yolo_model_cache[cache_key]
                # OPTIMISATION RAM: Mettre à jour timestamp usage
                _yolo_model_last_used[cache_key] = time_module.time()
                self.is_loaded = True
                return True

            # OPTIMISATION RAM: Vérifier limite cache et décharger LRU si nécessaire
            if len(_yolo_model_cache) >= _MAX_YOLO_CACHE_SIZE:
                # Trouver modèle le moins récemment utilisé
                if _yolo_model_last_used:
                    oldest_key = min(_yolo_model_last_used.items(), key=lambda x: x[1])[
                        0
                    ]
                    del _yolo_model_cache[oldest_key]
                    del _yolo_model_last_used[oldest_key]
                    logger.debug(
                        f"♻️ Modèle YOLO LRU déchargé: {oldest_key} (optimisation RAM)",
                    )

        try:
            logger.info(f"📥 Chargement modèle YOLOv8{self.model_size}...")
            start_time = time_module.time()

            model = YOLO(f"yolov8{self.model_size}.pt")  # type: ignore

            load_time = time_module.time() - start_time
            logger.info(f"✅ Modèle YOLO chargé en {load_time:.1f}s")

            # OPTIMISATION RAM: Mettre en cache avec timestamp
            with _yolo_cache_lock:
                _yolo_model_cache[cache_key] = model
                _yolo_model_last_used[cache_key] = time_module.time()

            self.model = model
            self.is_loaded = True
            return True

        except Exception as e:
            logger.error(f"❌ Erreur chargement YOLO: {e}")
            return False

    def detect_objects(self, image: np.ndarray) -> list[dict[str, Any]]:
        """Détecte les objets dans une image.

        Args:
            image: Image numpy array (BGR)

        Returns:
            Liste des détections avec bbox, confiance, classe

        """
        if not self.is_loaded:
            if not self.load_model():
                return []

        try:
            # Détection YOLO
            if self.model is None:
                logger.error("❌ Modèle YOLO non chargé")
                return []

            results = self.model(image, conf=self.confidence_threshold, verbose=False)  # type: ignore

            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Coordonnées bbox
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                        # Confiance et classe
                        confidence = box.conf[0].cpu().numpy()
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = self.model.names[class_id]  # type: ignore

                        detection = {
                            "bbox": [int(x1), int(y1), int(x2), int(y2)],
                            "confidence": float(confidence),
                            "class_id": class_id,
                            "class_name": class_name,
                            "center": [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                            "area": int((x2 - x1) * (y2 - y1)),
                        }
                        detections.append(detection)

            logger.debug(f"🔍 {len(detections)} objets détectés")
            return detections

        except Exception as e:
            logger.error(f"❌ Erreur détection YOLO: {e}")
            return []

    def get_best_detection(
        self,
        detections: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        """Retourne la meilleure détection selon les critères BBIA.

        Args:
            detections: Liste des détections

        Returns:
            Meilleure détection ou None

        """
        if not detections:
            return None

        # Filtrer par classes d'intérêt
        relevant_detections = [
            d for d in detections if d["class_name"] in self.target_classes
        ]

        if not relevant_detections:
            return None

        # Priorité: confiance + taille
        best_detection = max(
            relevant_detections,
            key=lambda d: d["confidence"]
            * (1 + d["area"] / 100000),  # Bonus pour grande taille
        )

        return best_detection

    def map_detection_to_action(
        self,
        detection: dict[str, Any],
    ) -> dict[str, Any] | None:
        """Mappe une détection vers une action RobotAPI.

        Args:
            detection: Détection d'objet

        Returns:
            Action mappée ou None

        """
        if not detection:
            return None

        class_name = detection["class_name"]

        if class_name in self.target_classes:
            action = self.target_classes[class_name]
            confidence = detection["confidence"]

            # Calcul direction de regard basée sur position
            center_x = detection["center"][0]
            direction = "left" if center_x < 320 else "right"  # 640px width assumption

            action_data = {
                "action": action,
                "confidence": confidence,
                "direction": direction,
                "object": class_name,
                "bbox": detection["bbox"],
            }

            logger.info(f"🎯 Détection mappée: {class_name} → {action} ({direction})")
            return action_data

        return None


class FaceDetector:
    """Module de détection de visages utilisant MediaPipe."""

    def __init__(self):
        """Initialise le détecteur de visages (utilise cache global si disponible)."""
        self.mp_face_detection = None
        self.mp_drawing = None
        self.face_detection = None

        # OPTIMISATION PERFORMANCE: Réutiliser instance MediaPipe depuis cache global
        global _mediapipe_face_detection_cache
        with _mediapipe_cache_lock:
            if _mediapipe_face_detection_cache is not None:
                logger.debug("♻️ Réutilisation détecteur MediaPipe depuis cache")
                self.face_detection = _mediapipe_face_detection_cache
                try:
                    import mediapipe as mp

                    self.mp_face_detection = mp.solutions.face_detection
                    self.mp_drawing = mp.solutions.drawing_utils
                except ImportError:
                    pass
                return

        try:
            import mediapipe as mp

            self.mp_face_detection = mp.solutions.face_detection
            self.mp_drawing = mp.solutions.drawing_utils
            face_detection = self.mp_face_detection.FaceDetection(
                model_selection=0,  # 0 pour visages proches, 1 pour distants
                min_detection_confidence=0.5,
            )

            # Mettre en cache global
            with _mediapipe_cache_lock:
                _mediapipe_face_detection_cache = face_detection

            self.face_detection = face_detection
            logger.info("👤 Détecteur de visages MediaPipe initialisé")

        except ImportError:
            logger.warning("⚠️ MediaPipe non disponible")

    def detect_faces(self, image: np.ndarray) -> list[dict[str, Any]]:
        """Détecte les visages dans une image.

        Args:
            image: Image numpy array (RGB)

        Returns:
            Liste des détections de visages

        """
        if not self.face_detection:
            return []

        try:
            # Conversion BGR → RGB
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Détection
            results = self.face_detection.process(rgb_image)

            detections = []
            if results.detections:
                h, w, _ = image.shape

                for detection in results.detections:
                    # Bounding box
                    bbox = detection.location_data.relative_bounding_box
                    x1 = int(bbox.xmin * w)
                    y1 = int(bbox.ymin * h)
                    x2 = int((bbox.xmin + bbox.width) * w)
                    y2 = int((bbox.ymin + bbox.height) * h)

                    # Confiance
                    confidence = detection.score[0]

                    face_data = {
                        "bbox": [x1, y1, x2, y2],
                        "confidence": float(confidence),
                        "center": [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                        "area": int((x2 - x1) * (y2 - y1)),
                    }
                    detections.append(face_data)

            logger.debug(f"👤 {len(detections)} visages détectés")
            return detections

        except Exception as e:
            logger.error(f"❌ Erreur détection visages: {e}")
            return []

    def get_best_face(self, detections: list[dict[str, Any]]) -> dict[str, Any] | None:
        """Retourne le meilleur visage détecté.

        Args:
            detections: Liste des détections de visages

        Returns:
            Meilleur visage ou None

        """
        if not detections:
            return None

        # Priorité: confiance + taille
        best_face = max(
            detections,
            key=lambda d: d["confidence"] * (1 + d["area"] / 50000),
        )

        return best_face


def create_yolo_detector(
    model_size: str = "n",
    confidence_threshold: float = 0.25,
) -> YOLODetector | None:
    """Factory function pour créer une instance YOLODetector.

    Args:
        model_size: Taille du modèle YOLO ("n", "s", "m", "l", "x")
        confidence_threshold: Seuil de confiance (0.0-1.0), défaut: 0.25

    Returns:
        Instance YOLODetector ou None si non disponible

    """
    if not YOLO_AVAILABLE:
        logger.warning("⚠️ YOLO non disponible")
        return None

    return YOLODetector(
        model_size=model_size,
        confidence_threshold=confidence_threshold,
    )


def create_face_detector() -> FaceDetector | None:
    """Factory function pour créer une instance FaceDetector.

    Returns:
        Instance FaceDetector ou None si non disponible

    """
    try:
        import mediapipe as mp  # noqa: F401

        return FaceDetector()
    except ImportError:
        logger.warning("⚠️ MediaPipe non disponible")
        return None


# Test rapide
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("🧪 Test modules Vision BBIA")
    print("=" * 40)

    # Test disponibilité
    print(f"YOLO disponible: {YOLO_AVAILABLE}")

    try:
        import mediapipe as mp  # noqa: F401

        print("MediaPipe disponible: True")
    except ImportError:
        print("MediaPipe disponible: False")

    # Test création détecteurs
    yolo = create_yolo_detector("n")
    if yolo:
        print("✅ Détecteur YOLO créé")
    else:
        print("❌ Impossible de créer le détecteur YOLO")

    face_detector = create_face_detector()
    if face_detector:
        print("✅ Détecteur de visages créé")
    else:
        print("❌ Impossible de créer le détecteur de visages")
