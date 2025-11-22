#!/usr/bin/env python3
"""bbia_vision_yolo.py - Module YOLOv8n pour BBIA
Détection d'objets légère avec YOLOv8n (optionnel).
"""

import logging
import operator
import threading
from typing import TYPE_CHECKING, Any

import numpy as np
import numpy.typing as npt

# Import DetectionResult avec fallback pour compatibilité CI/CD
if TYPE_CHECKING:
    from bbia_sim.utils.types import DetectionResult
else:
    try:
        from bbia_sim.utils.types import DetectionResult
    except ImportError:
        # Fallback final : utiliser TypedDict inline si l'import absolu échoue
        from typing import TypedDict

        class DetectionResult(TypedDict, total=False):
            """Résultat de détection d'objet (YOLO) - définition inline."""

            bbox: list[int]
            confidence: float
            class_id: int
            class_name: str
            center: list[int]
            area: int


try:
    import cv2  # type: ignore[import-untyped]
    from ultralytics import YOLO  # type: ignore[import-untyped]

    YOLO_AVAILABLE = True
    CV2_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    CV2_AVAILABLE = False
    cv2 = None  # type: ignore[assignment]
    # Imports non disponibles - le code vérifie YOLO_AVAILABLE avant utilisation

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
    _os.environ.setdefault(
        "TF_CPP_MIN_LOG_LEVEL",
        "3",
    )  # 0=INFO,1=WARNING,2=ERROR,3=FATAL
    _os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")
    # Supprimer les logs TensorFlow Lite
    _os.environ.setdefault("TFLITE_LOG_VERBOSITY", "0")  # 0=ERROR, 1=WARNING, 2=INFO
    # Supprimer les logs OpenGL (ne pas définir MUJOCO_GL sur macOS, utilise la valeur par défaut)
    # Sur macOS, laisser MuJoCo choisir automatiquement (glfw ou egl)
    import sys

    if sys.platform != "darwin":  # Pas macOS
        _os.environ.setdefault("MUJOCO_GL", "egl")  # Utiliser EGL sur Linux/Windows
except Exception as e:
    logger.debug(
        "Impossible de configurer variables d'environnement MediaPipe/TensorFlow: %s",
        e,
    )


class YOLODetector:
    """Module de détection d'objets utilisant YOLOv8n."""

    def __init__(
        self, model_size: str = "n", confidence_threshold: float = 0.25
    ) -> None:
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
                logger.debug("♻️ Réutilisation modèle YOLO depuis cache (%s)", cache_key)
                self.model = _yolo_model_cache[cache_key]
                # OPTIMISATION RAM: Mettre à jour timestamp usage
                _yolo_model_last_used[cache_key] = time_module.time()
                self.is_loaded = True
                return True

            # OPTIMISATION RAM: Vérifier limite cache et décharger LRU si nécessaire
            if len(_yolo_model_cache) >= _MAX_YOLO_CACHE_SIZE:
                # Trouver modèle le moins récemment utilisé
                if _yolo_model_last_used:
                    # OPTIMISATION: operator.itemgetter plus rapide que lambda
                    oldest_key = min(
                        _yolo_model_last_used.items(),
                        key=operator.itemgetter(1),
                    )[0]
                    del _yolo_model_cache[oldest_key]
                    del _yolo_model_last_used[oldest_key]
                    logger.debug(
                        "♻️ Modèle YOLO LRU déchargé: %s (optimisation RAM)",
                        oldest_key,
                    )

        try:
            logger.info("📥 Chargement modèle YOLOv8%s...", self.model_size)
            start_time = time_module.time()

            model = YOLO(f"yolov8{self.model_size}.pt")

            load_time = time_module.time() - start_time
            logger.info("✅ Modèle YOLO chargé en %.1fs", load_time)

            # OPTIMISATION RAM: Mettre en cache avec timestamp
            with _yolo_cache_lock:
                _yolo_model_cache[cache_key] = model
                _yolo_model_last_used[cache_key] = time_module.time()

            self.model = model
            self.is_loaded = True
            return True

        except Exception as e:
            logger.exception("❌ Erreur chargement YOLO")
            return False

    def detect_objects(self, image: npt.NDArray[np.uint8]) -> list[DetectionResult]:
        """Détecte les objets dans une image.

        Args:
            image: Image numpy array (BGR)

        Returns:
            Liste des détections avec bbox, confiance, classe (typée avec DetectionResult)

        """
        if not self.is_loaded and not self.load_model():
            return []

        try:
            # Détection YOLO
            if self.model is None:
                logger.error("❌ Modèle YOLO non chargé")
                return []

            # OPTIMISATION PERFORMANCE: Réduire résolution image avant traitement YOLO
            # (640x480 au lieu de 1280x720) pour réduire latence
            # YOLO fonctionne bien avec résolution réduite et c'est beaucoup plus rapide
            original_height, original_width = image.shape[:2]
            target_width = 640
            target_height = 480

            # Resize seulement si image plus grande que cible
            if original_width > target_width or original_height > target_height:
                # Calculer ratio de resize pour maintenir aspect ratio
                width_ratio = target_width / original_width
                height_ratio = target_height / original_height
                ratio = min(width_ratio, height_ratio)

                new_width = int(original_width * ratio)
                new_height = int(original_height * ratio)

                # Resize avec cv2 (rapide et optimisé)
                if CV2_AVAILABLE and cv2 is not None:
                    resized_image = cv2.resize(
                        image,
                        (new_width, new_height),
                        interpolation=cv2.INTER_LINEAR,
                    )
                else:
                    # Fallback sans cv2 (ne devrait pas arriver si YOLO disponible)
                    resized_image = image
                    new_width, new_height = original_width, original_height
            else:
                resized_image = image
                new_width, new_height = original_width, original_height

            results = self.model(
                resized_image,
                conf=self.confidence_threshold,
                verbose=False,
            )

            detections = []
            # Calculer ratio de scale pour convertir bbox de l'image resize vers originale
            scale_x = original_width / new_width if new_width > 0 else 1.0
            scale_y = original_height / new_height if new_height > 0 else 1.0

            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Coordonnées bbox (dans l'image resize)
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                        # OPTIMISATION PERFORMANCE: Convertir bbox vers résolution originale
                        # pour cohérence avec le reste du pipeline
                        x1_orig = int(x1 * scale_x)
                        y1_orig = int(y1 * scale_y)
                        x2_orig = int(x2 * scale_x)
                        y2_orig = int(y2 * scale_y)

                        # Confiance et classe
                        confidence = box.conf[0].cpu().numpy()
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = self.model.names[class_id]

                        detection: DetectionResult = {
                            "bbox": [x1_orig, y1_orig, x2_orig, y2_orig],
                            "confidence": float(confidence),
                            "class_id": class_id,
                            "class_name": class_name,
                            "center": [
                                int((x1_orig + x2_orig) / 2),
                                int((y1_orig + y2_orig) / 2),
                            ],
                            "area": int((x2_orig - x1_orig) * (y2_orig - y1_orig)),
                        }
                        detections.append(detection)

            logger.debug("🔍 %s objets détectés", len(detections))
            return detections

        except Exception as e:
            logger.exception("❌ Erreur détection YOLO")
            return []

    def detect_objects_batch(
        self,
        images: list[npt.NDArray[np.uint8]],
    ) -> list[list[DetectionResult]]:
        """Détecte les objets dans un batch d'images (OPTIMISATION PERFORMANCE).

        Args:
            images: Liste d'images numpy array (BGR)

        Returns:
            Liste de listes de détections (une par image) avec bbox, confiance, classe
            (typée avec DetectionResult)

        Note:
            Le batch processing est beaucoup plus efficace que d'appeler detect_objects
            plusieurs fois, car YOLO peut traiter plusieurs images en parallèle sur GPU.

        """
        if not self.is_loaded and not self.load_model():
            return [[] for _ in images]

        if not images:
            return []

        try:
            if self.model is None:
                logger.error("❌ Modèle YOLO non chargé")
                return [[] for _ in images]

            # OPTIMISATION PERFORMANCE: Réduire résolution images avant traitement YOLO
            # (640x480 max) pour réduire latence batch
            target_width = 640
            target_height = 480
            resized_images = []
            image_scales = []  # Stocker les ratios de scale pour chaque image

            for image in images:
                original_height, original_width = image.shape[:2]

                # Resize seulement si image plus grande que cible
                if original_width > target_width or original_height > target_height:
                    # Calculer ratio de resize pour maintenir aspect ratio
                    width_ratio = target_width / original_width
                    height_ratio = target_height / original_height
                    ratio = min(width_ratio, height_ratio)

                    new_width = int(original_width * ratio)
                    new_height = int(original_height * ratio)

                    # Resize avec cv2 (rapide et optimisé)
                    if CV2_AVAILABLE and cv2 is not None:
                        resized_image = cv2.resize(
                            image,
                            (new_width, new_height),
                            interpolation=cv2.INTER_LINEAR,
                        )
                    else:
                        resized_image = image
                        new_width, new_height = original_width, original_height

                    scale_x = original_width / new_width if new_width > 0 else 1.0
                    scale_y = original_height / new_height if new_height > 0 else 1.0
                else:
                    resized_image = image
                    scale_x = 1.0
                    scale_y = 1.0

                resized_images.append(resized_image)
                image_scales.append((scale_x, scale_y))

            # OPTIMISATION PERFORMANCE: YOLO traite le batch en une seule passe
            # (beaucoup plus rapide que boucle sur images individuelles)
            results = self.model(
                resized_images,
                conf=self.confidence_threshold,
                verbose=False,
            )

            all_detections = []
            for idx, result in enumerate(results):
                detections = []
                boxes = result.boxes
                # Récupérer les ratios de scale pour cette image
                scale_x, scale_y = (
                    image_scales[idx] if idx < len(image_scales) else (1.0, 1.0)
                )

                if boxes is not None:
                    for box in boxes:
                        # Coordonnées bbox (dans l'image resize)
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                        # OPTIMISATION PERFORMANCE: Convertir bbox vers résolution originale
                        x1_orig = int(x1 * scale_x)
                        y1_orig = int(y1 * scale_y)
                        x2_orig = int(x2 * scale_x)
                        y2_orig = int(y2 * scale_y)

                        # Confiance et classe
                        confidence = box.conf[0].cpu().numpy()
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = self.model.names[class_id]

                        detection: DetectionResult = {
                            "bbox": [x1_orig, y1_orig, x2_orig, y2_orig],
                            "confidence": float(confidence),
                            "class_id": class_id,
                            "class_name": class_name,
                            "center": [
                                int((x1_orig + x2_orig) / 2),
                                int((y1_orig + y2_orig) / 2),
                            ],
                            "area": int((x2_orig - x1_orig) * (y2_orig - y1_orig)),
                        }
                        detections.append(detection)
                all_detections.append(detections)

            total_detections = sum(len(d) for d in all_detections)
            logger.debug(
                "🔍 Batch processing: %d images, %d objets détectés au total",
                len(images),
                total_detections,
            )
            return all_detections

        except Exception as e:
            logger.exception("❌ Erreur détection YOLO batch")
            return [[] for _ in images]

    def get_best_detection(
        self,
        detections: list[DetectionResult],
    ) -> DetectionResult | None:
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
        return max(
            relevant_detections,
            key=lambda d: d["confidence"]
            * (1 + d["area"] / 100000),  # Bonus pour grande taille
        )

    def map_detection_to_action(
        self,
        detection: dict[str, Any] | None,
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

            logger.info(
                "🎯 Détection mappée: %s → %s (%s)",
                class_name,
                action,
                direction,
            )
            return action_data

        return None


class FaceDetector:
    """Module de détection de visages utilisant MediaPipe."""

    def __init__(self) -> None:
        """Initialise le détecteur de visages (utilise cache global si disponible)."""
        self.mp_face_detection: Any | None = None
        self.mp_drawing: Any | None = None
        self.face_detection: Any | None = None

        # OPTIMISATION PERFORMANCE: Réutiliser instance MediaPipe depuis cache global
        global _mediapipe_face_detection_cache
        with _mediapipe_cache_lock:
            if _mediapipe_face_detection_cache is not None:
                logger.debug("♻️ Réutilisation détecteur MediaPipe depuis cache")
                self.face_detection = _mediapipe_face_detection_cache
                try:
                    import mediapipe as mp  # type: ignore[import-untyped]

                    self.mp_face_detection = mp.solutions.face_detection
                    self.mp_drawing = mp.solutions.drawing_utils
                except ImportError:
                    pass
                return

        try:
            import mediapipe as mp  # type: ignore[import-untyped]

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
            logger.debug("👤 Détecteur de visages MediaPipe initialisé")

        except ImportError:
            logger.warning("⚠️ MediaPipe non disponible")

    def detect_faces(self, image: npt.NDArray[np.uint8]) -> list[dict[str, Any]]:
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

            logger.debug("👤 %s visages détectés", len(detections))
            return detections

        except Exception as e:
            logger.exception("❌ Erreur détection visages")
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
        return max(
            detections,
            key=lambda d: d["confidence"] * (1 + d["area"] / 50000),
        )


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
        import mediapipe as mp  # type: ignore[import-untyped]  # noqa: F401

        return FaceDetector()
    except ImportError:
        logger.warning("⚠️ MediaPipe non disponible")
        return None


# Test rapide
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    logging.info("🧪 Test modules Vision BBIA")
    logging.info("=" * 40)

    # Test disponibilité
    logging.info(f"YOLO disponible: {YOLO_AVAILABLE}")

    try:
        import mediapipe as mp  # type: ignore[import-untyped]  # noqa: F401

        logging.info("MediaPipe disponible: True")
    except ImportError:
        logging.info("MediaPipe disponible: False")

    # Test création détecteurs
    yolo = create_yolo_detector("n")
    if yolo:
        logging.info("✅ Détecteur YOLO créé")
    else:
        logging.error("❌ Impossible de créer le détecteur YOLO")

    face_detector = create_face_detector()
    if face_detector:
        logging.info("✅ Détecteur de visages créé")
    else:
        logging.error("❌ Impossible de créer le détecteur de visages")
