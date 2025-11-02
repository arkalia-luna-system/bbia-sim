#!/usr/bin/env python3

"""BBIA Vision - Module de vision avancé pour Reachy Mini Wireless
Reconnaissance d'objets, détection de visages, suivi d'objets.
"""

import logging
import math
import os
import threading
from collections import deque
from collections.abc import Callable
from datetime import datetime
from typing import TYPE_CHECKING, Any, cast

if TYPE_CHECKING:
    from .face_recognition import BBIAPersonRecognition
    from .pose_detection import BBIAPoseDetection

import numpy as np
import numpy.typing as npt

logger = logging.getLogger(__name__)

# OPTIMISATION RAM: Singleton BBIAVision partagé (évite créations multiples)
_bbia_vision_singleton: "BBIAVision | None" = None
_bbia_vision_lock = threading.Lock()


def get_bbia_vision_singleton(robot_api: Any | None = None) -> "BBIAVision":
    """OPTIMISATION RAM: Retourne instance singleton BBIAVision partagée.

    Args:
        robot_api: Interface RobotAPI (optionnel)

    Returns:
        Instance singleton BBIAVision (créée si nécessaire)
    """
    global _bbia_vision_singleton
    if _bbia_vision_singleton is None:
        with _bbia_vision_lock:
            if _bbia_vision_singleton is None:
                _bbia_vision_singleton = BBIAVision(robot_api=robot_api)
                logger.debug("✅ Singleton BBIAVision créé (optimisation RAM)")
    return _bbia_vision_singleton


# Réduction du bruit de logs TensorFlow/MediaPipe (avant tout import MediaPipe)
try:
    import os as _os  # noqa: F401

    _os.environ.setdefault("GLOG_minloglevel", "2")  # 0=INFO,1=WARNING,2=ERROR
    _os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")  # 1=WARNING,2=ERROR
    _os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")  # éviter logs GPU inutiles
except Exception:
    pass  # noqa: S101 - Ignorer erreur configuration variables d'environnement

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

# Import conditionnel DeepFace pour reconnaissance visage personnalisée
create_face_recognition: Callable[[str, str], "BBIAPersonRecognition | None"] | None = (
    None
)
DEEPFACE_AVAILABLE = False
try:
    from .face_recognition import (
        create_face_recognition as _create_face_recognition_imported,
    )

    create_face_recognition = _create_face_recognition_imported
    DEEPFACE_AVAILABLE = True
except ImportError:
    pass  # create_face_recognition reste None

# Import conditionnel MediaPipe Pose pour détection postures/gestes
create_pose_detector: Callable[..., "BBIAPoseDetection | None"] | None = None
MEDIAPIPE_POSE_AVAILABLE = False
try:
    from .pose_detection import create_pose_detector as _create_pose_detector_imported

    create_pose_detector = _create_pose_detector_imported
    MEDIAPIPE_POSE_AVAILABLE = True
except ImportError:
    pass  # create_pose_detector reste None

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

    face_recognition: "BBIAPersonRecognition | None"

    def __init__(self, robot_api: Any | None = None) -> None:
        """
        Initialise le module de vision.

        Args:
            robot_api: Interface RobotAPI (optionnel) pour accès robot.media.camera
        """
        self.robot_api = robot_api
        self.camera_active = True
        self.vision_quality = "HD"
        self.detection_range = 3.0  # mètres
        # OPTIMISATION RAM: Limiter historique détections avec deque (max 50 objets/visages)
        self._max_detections_history = 50
        self.objects_detected: deque[dict[str, Any]] = deque(
            maxlen=self._max_detections_history
        )
        self.faces_detected: deque[dict[str, Any]] = deque(
            maxlen=self._max_detections_history
        )
        self.tracking_active = False
        self.current_focus: dict[str, Any] | None = None

        # Spécifications hardware réelles
        # Note: Résolution simulation = 1280x720 (XML officiel)
        # Résolution réelle peut être différente - vérifier avec robot physique
        self.specs = {
            "camera": "Grand angle",
            "resolution": "1280x720 (simulation) / HD grand-angle (réel)",  # Clarifié
            "fov": "80° (simulation) / ~120° (réel estimé)",  # Clarifié
            "focus": "Auto",
            "night_vision": False,
        }

        # Vérifier disponibilité robot.media.camera (toujours disponible via shim)
        self._camera_sdk_available = False
        self._camera = None
        if robot_api and hasattr(robot_api, "media"):
            try:
                media = robot_api.media
                # Media est maintenant toujours disponible (shim en simulation)
                if media:
                    self._camera = getattr(media, "camera", None)
                    if self._camera is not None:
                        # Vérifier si c'est une vraie caméra SDK ou un shim
                        from ..backends.simulation_shims import SimulationCamera

                        if isinstance(self._camera, SimulationCamera):
                            self._camera_sdk_available = False
                            logger.debug("Caméra simulation (shim) disponible")
                        else:
                            self._camera_sdk_available = True
                            logger.info("✅ Caméra SDK disponible: robot.media.camera")
            except Exception as e:
                logger.debug(f"Caméra SDK non disponible (fallback simulation): {e}")

        # Support webcam USB via OpenCV (fallback si pas de SDK)
        self._opencv_camera = None
        self._opencv_camera_available = False
        if not self._camera_sdk_available and CV2_AVAILABLE and cv2:
            try:
                # Lire device index depuis variable d'environnement
                camera_index_str = os.environ.get("BBIA_CAMERA_INDEX", "0")
                camera_device = os.environ.get("BBIA_CAMERA_DEVICE")

                # Essayer device path si fourni, sinon index
                if camera_device:
                    self._opencv_camera = cv2.VideoCapture(camera_device)
                    logger.info(f"🔌 Tentative ouverture webcam: {camera_device}")
                else:
                    try:
                        camera_index = int(camera_index_str)
                        self._opencv_camera = cv2.VideoCapture(camera_index)
                        logger.info(
                            f"🔌 Tentative ouverture webcam (index {camera_index})"
                        )
                    except ValueError:
                        logger.warning(
                            f"⚠️ BBIA_CAMERA_INDEX invalide: {camera_index_str}, utilisation index 0"
                        )
                        self._opencv_camera = cv2.VideoCapture(0)

                # Tester si la caméra fonctionne
                if self._opencv_camera is not None and self._opencv_camera.isOpened():
                    ret, test_frame = self._opencv_camera.read()
                    if ret and test_frame is not None:
                        self._opencv_camera_available = True
                        logger.info("✅ Webcam USB OpenCV disponible")
                    else:
                        self._opencv_camera.release()
                        self._opencv_camera = None
                        logger.debug(
                            "Webcam OpenCV ouverte mais ne capture pas d'images"
                        )
                else:
                    if self._opencv_camera:
                        self._opencv_camera.release()
                    self._opencv_camera = None
                    logger.debug("Webcam OpenCV non disponible (fallback simulation)")
            except Exception as e:
                if self._opencv_camera:
                    try:
                        self._opencv_camera.release()
                    except Exception:
                        pass  # noqa: S101 - Ignorer erreur libération caméra OpenCV (déjà libérée ou non critique)
                self._opencv_camera = None
                logger.debug(f"Erreur initialisation webcam OpenCV: {e}")

        # OPTIMISATION RAM: Lazy loading YOLO/MediaPipe - ne charger que si caméra réelle disponible
        self.yolo_detector = None
        # Charger YOLO uniquement si caméra SDK réelle disponible (pas shim simulation)
        if (
            self._camera_sdk_available
            and YOLO_AVAILABLE
            and create_yolo_detector is not None
        ):
            try:
                # Seuil confiance 0.25 pour meilleure détection (au lieu de 0.5 par défaut)
                confidence_threshold = float(
                    os.environ.get("BBIA_YOLO_CONFIDENCE", "0.25")
                )
                # Créer détecteur avec seuil personnalisé
                self.yolo_detector = create_yolo_detector(
                    model_size="n", confidence_threshold=confidence_threshold
                )
                if self.yolo_detector:
                    self.yolo_detector.load_model()
                    logger.info(
                        "✅ Détecteur YOLO initialisé (lazy loading - caméra réelle)"
                    )
            except Exception as e:
                logger.warning(f"⚠️ YOLO non disponible: {e}")
        else:
            logger.debug(
                "YOLO non chargé (lazy loading - caméra simulation ou non disponible)"
            )

        self.face_detector = None
        if MEDIAPIPE_AVAILABLE and mp:
            try:
                # OPTIMISATION PERFORMANCE: Réutiliser instance MediaPipe depuis cache si disponible
                try:
                    from .vision_yolo import (
                        _mediapipe_cache_lock,
                        _mediapipe_face_detection_cache,
                    )

                    with _mediapipe_cache_lock:
                        if _mediapipe_face_detection_cache is not None:
                            logger.debug(
                                "♻️ Réutilisation détecteur MediaPipe depuis cache (bbia_vision)"
                            )
                            self.face_detector = _mediapipe_face_detection_cache
                            logger.info(
                                "✅ Détecteur MediaPipe Face initialisé (cache)"
                            )
                        else:
                            # Créer nouvelle instance et mettre en cache
                            self.face_detector = (
                                mp.solutions.face_detection.FaceDetection(
                                    model_selection=0, min_detection_confidence=0.5
                                )
                            )
                            _mediapipe_face_detection_cache = self.face_detector
                            logger.info("✅ Détecteur MediaPipe Face initialisé")
                except ImportError:
                    # Fallback si import cache échoue
                    self.face_detector = mp.solutions.face_detection.FaceDetection(
                        model_selection=0, min_detection_confidence=0.5
                    )
                    logger.info("✅ Détecteur MediaPipe Face initialisé")
            except Exception as e:
                logger.warning(f"⚠️ MediaPipe non disponible: {e}")

        # Module DeepFace pour reconnaissance visage personnalisée + émotions
        self.face_recognition = None
        if DEEPFACE_AVAILABLE and create_face_recognition is not None:
            try:
                db_path = os.environ.get("BBIA_FACES_DB", "faces_db")
                model_name = os.environ.get("BBIA_DEEPFACE_MODEL", "VGG-Face")
                self.face_recognition = create_face_recognition(db_path, model_name)
                if self.face_recognition and self.face_recognition.is_initialized:
                    logger.info(
                        f"✅ DeepFace initialisé (db: {db_path}, modèle: {model_name})"
                    )
            except Exception as e:
                logger.warning(f"⚠️ DeepFace non disponible: {e}")

        # Module MediaPipe Pose pour détection postures/gestes
        self.pose_detector = None
        if MEDIAPIPE_POSE_AVAILABLE and create_pose_detector is not None:
            try:
                model_complexity = int(
                    os.environ.get("BBIA_POSE_COMPLEXITY", "1")
                )  # 0=rapide, 1=équilibré, 2=précis
                self.pose_detector = create_pose_detector(
                    model_complexity=model_complexity
                )
                if self.pose_detector and self.pose_detector.is_initialized:
                    logger.info(
                        f"✅ MediaPipe Pose initialisé (complexité: {model_complexity})"
                    )
            except Exception as e:
                logger.warning(f"⚠️ MediaPipe Pose non disponible: {e}")

    def _capture_image_from_camera(self) -> npt.NDArray[np.uint8] | None:
        """Capture une image depuis robot.media.camera si disponible, sinon webcam USB OpenCV.

        CORRECTION EXPERTE: Validation robuste du format d'image SDK avec gestion
        des différents formats possibles (RGB, BGR, grayscale, etc.)

        Returns:
            Image numpy array (BGR pour compatibilité OpenCV) ou None si non disponible
        """
        # Priorité 1: SDK Reachy Mini camera
        if self._camera_sdk_available and self._camera:
            return self._capture_from_sdk_camera()

        # Priorité 2: Webcam USB via OpenCV
        if self._opencv_camera_available and self._opencv_camera:
            return self._capture_from_opencv_camera()

        # Pas de caméra disponible
        return None

    def _capture_from_sdk_camera(self) -> npt.NDArray[np.uint8] | None:
        """Capture depuis robot.media.camera (SDK Reachy Mini)."""
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

    def _capture_from_opencv_camera(self) -> npt.NDArray[np.uint8] | None:
        """Capture depuis webcam USB via OpenCV VideoCapture."""
        if not self._opencv_camera_available or not self._opencv_camera:
            return None

        try:
            if not CV2_AVAILABLE or cv2 is None:
                return None

            # Lire une frame depuis la webcam
            ret, image = self._opencv_camera.read()
            if not ret or image is None:
                logger.debug("Échec lecture frame webcam OpenCV")
                return None

            # Image déjà en BGR (format OpenCV standard)
            if not isinstance(image, np.ndarray):
                return None

            # Validation shape et dtype
            if image.ndim < 2 or image.ndim > 3:
                logger.warning(f"Format image invalide (ndim={image.ndim})")
                return None

            # Assurer dtype uint8
            if image.dtype != np.uint8:
                try:
                    if image.dtype in (np.float32, np.float64):
                        if image.max() <= 1.0:
                            image = (image * 255).astype(np.uint8)
                        else:
                            image = image.astype(np.uint8)
                    else:
                        image = image.astype(np.uint8)
                except Exception as e:
                    logger.debug(f"Erreur conversion dtype: {e}")
                    return None

            logger.debug("✅ Image capturée depuis webcam USB OpenCV")
            return cast(npt.NDArray[np.uint8], image)

        except Exception as e:
            logger.debug(f"Erreur capture webcam OpenCV: {e}")
            return None

    def scan_environment_from_image(
        self, image: npt.NDArray[np.uint8]
    ) -> dict[str, Any]:
        """Scanne l'environnement depuis une image fournie (au lieu de la caméra).

        Args:
            image: Image numpy array (BGR)

        Returns:
            Dict avec objets, visages, postures détectés
        """
        objects = []
        faces = []
        poses = []

        # Détection objets avec YOLO
        if self.yolo_detector and self.yolo_detector.is_loaded:
            try:
                detections = self.yolo_detector.detect_objects(image)
                height, width = image.shape[:2]

                for det in detections:
                    # YOLO retourne bbox comme liste [x1, y1, x2, y2]
                    bbox_yolo = det.get("bbox", [])
                    if isinstance(bbox_yolo, list) and len(bbox_yolo) == 4:
                        x1, y1, x2, y2 = bbox_yolo
                        # Calculer centre et dimensions
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2
                        w = x2 - x1
                        h = y2 - y1
                    else:
                        # Fallback si format inattendu
                        continue

                    obj = {
                        "name": det.get("class_name", det.get("class", "objet")),
                        "distance": 1.5,
                        "confidence": det.get("confidence", 0.5),
                        "position": (center_x / width, center_y / height),
                        "bbox": {
                            "x": int(x1),
                            "y": int(y1),
                            "width": int(w),
                            "height": int(h),
                            "center_x": int(center_x),
                            "center_y": int(center_y),
                        },
                    }
                    objects.append(obj)
            except Exception as e:
                logger.warning(f"Erreur détection YOLO: {e}")

        # Détection visages avec MediaPipe + DeepFace
        if self.face_detector:
            try:
                import cv2

                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = self.face_detector.process(image_rgb)

                if results.detections:
                    height, width = image.shape[:2]
                    for detection in results.detections:
                        bbox = detection.location_data.relative_bounding_box

                        # Extraire ROI pour DeepFace
                        face_roi = None
                        if (
                            self.face_recognition
                            and self.face_recognition.is_initialized
                        ):
                            x = int(bbox.xmin * width)
                            y = int(bbox.ymin * height)
                            w = int(bbox.width * width)
                            h = int(bbox.height * height)
                            margin = 20
                            x = max(0, x - margin)
                            y = max(0, y - margin)
                            w = min(width - x, w + 2 * margin)
                            h = min(height - y, h + 2 * margin)
                            face_roi = image[y : y + h, x : x + w]

                        # Détection DeepFace
                        recognized_name = "humain"
                        detected_emotion = "neutral"
                        emotion_confidence = 0.0

                        if (
                            face_roi is not None
                            and face_roi.size > 0
                            and self.face_recognition is not None
                            and self.face_recognition.is_initialized
                        ):
                            try:
                                person_result = self.face_recognition.recognize_person(
                                    face_roi, enforce_detection=False
                                )
                                if person_result:
                                    recognized_name = person_result["name"]

                                emotion_result = self.face_recognition.detect_emotion(
                                    face_roi, enforce_detection=False
                                )
                                if emotion_result:
                                    detected_emotion = emotion_result["emotion"]
                                    emotion_confidence = emotion_result["confidence"]
                            except Exception as deepface_error:
                                logger.debug(f"DeepFace erreur: {deepface_error}")

                        face = {
                            "name": recognized_name,
                            "distance": 1.5,
                            "confidence": (
                                detection.score[0] if detection.score else 0.8
                            ),
                            "emotion": detected_emotion,
                            "emotion_confidence": emotion_confidence,
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

        # Détection postures avec MediaPipe Pose
        if self.pose_detector and self.pose_detector.is_initialized:
            try:
                pose_result = self.pose_detector.detect_pose(image)
                if pose_result:
                    poses.append(
                        {
                            "landmarks_count": pose_result["num_landmarks"],
                            "gestures": pose_result["gestures"],
                            "posture": pose_result["posture"],
                        }
                    )
            except Exception as e:
                logger.debug(f"Erreur détection pose: {e}")

        # OPTIMISATION RAM: Limiter taille historique avec deque
        self.objects_detected = deque(
            objects[: self._max_detections_history], maxlen=self._max_detections_history
        )
        self.faces_detected = deque(
            faces[: self._max_detections_history], maxlen=self._max_detections_history
        )

        return {
            "objects": objects,
            "faces": faces,
            "poses": poses,
            "timestamp": datetime.now().isoformat(),
            "source": "image_upload",
        }

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
                    height, width = image.shape[:2]

                    for det in detections:
                        # YOLO retourne bbox comme liste [x1, y1, x2, y2]
                        bbox_yolo = det.get("bbox", [])
                        if isinstance(bbox_yolo, list) and len(bbox_yolo) == 4:
                            x1, y1, x2, y2 = bbox_yolo
                            # Calculer centre et dimensions
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2
                            w = x2 - x1
                            h = y2 - y1
                        else:
                            # Fallback si format inattendu
                            continue

                        # Convertir détection YOLO au format BBIA
                        obj = {
                            "name": det.get("class_name", det.get("class", "objet")),
                            "distance": (
                                1.5
                            ),  # Estimation basée sur bbox size (peut améliorer)
                            "confidence": det.get("confidence", 0.5),
                            "position": (center_x / width, center_y / height),
                            "bbox": {
                                "x": int(x1),
                                "y": int(y1),
                                "width": int(w),
                                "height": int(h),
                                "center_x": int(center_x),
                                "center_y": int(center_y),
                            },
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

                            # Extraire le visage détecté pour DeepFace (optionnel)
                            face_roi = None
                            if (
                                self.face_recognition
                                and self.face_recognition.is_initialized
                            ):
                                # Extraire ROI du visage pour DeepFace
                                x = int(bbox.xmin * width)
                                y = int(bbox.ymin * height)
                                w = int(bbox.width * width)
                                h = int(bbox.height * height)
                                # Ajouter marge de sécurité
                                margin = 20
                                x = max(0, x - margin)
                                y = max(0, y - margin)
                                w = min(width - x, w + 2 * margin)
                                h = min(height - y, h + 2 * margin)
                                face_roi = image[y : y + h, x : x + w]

                            # Détection DeepFace (reconnaissance + émotion) si disponible
                            recognized_name = "humain"
                            detected_emotion = "neutral"
                            emotion_confidence = 0.0

                            if (
                                face_roi is not None
                                and face_roi.size > 0
                                and self.face_recognition is not None
                                and self.face_recognition.is_initialized
                            ):
                                try:
                                    # Reconnaître la personne
                                    person_result = (
                                        self.face_recognition.recognize_person(
                                            face_roi, enforce_detection=False
                                        )
                                    )
                                    if person_result:
                                        recognized_name = person_result["name"]
                                        logger.debug(
                                            f"👤 Personne reconnue: {recognized_name} "
                                            f"(confiance: {person_result['confidence']:.2f})"
                                        )

                                    # Détecter l'émotion
                                    emotion_result = (
                                        self.face_recognition.detect_emotion(
                                            face_roi, enforce_detection=False
                                        )
                                    )
                                    if emotion_result:
                                        detected_emotion = emotion_result["emotion"]
                                        emotion_confidence = emotion_result[
                                            "confidence"
                                        ]
                                        logger.debug(
                                            f"😊 Émotion détectée: {detected_emotion} "
                                            f"(confiance: {emotion_confidence:.2f})"
                                        )
                                except Exception as deepface_error:
                                    logger.debug(
                                        f"DeepFace erreur (fallback): {deepface_error}"
                                    )

                            face = {
                                "name": recognized_name,
                                "distance": 1.5,  # Estimation basée sur bbox size
                                "confidence": (
                                    detection.score[0] if detection.score else 0.8
                                ),
                                "emotion": detected_emotion,
                                "emotion_confidence": emotion_confidence,
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

            # Détection de postures avec MediaPipe Pose (optionnel)
            poses = []
            if self.pose_detector and self.pose_detector.is_initialized:
                try:
                    pose_result = self.pose_detector.detect_pose(image)
                    if pose_result:
                        poses.append(
                            {
                                "landmarks_count": pose_result["num_landmarks"],
                                "gestures": pose_result["gestures"],
                                "posture": pose_result["posture"],
                            }
                        )
                        logger.debug(
                            f"🧍 Posture détectée: {pose_result['posture']}, "
                            f"gestes: {pose_result['gestures']}"
                        )
                except Exception as e:
                    logger.debug(f"Erreur détection pose: {e}")

            if objects or faces or poses:
                logger.info(
                    f"✅ Détection réelle: {len(objects)} objets, {len(faces)} visages, "
                    f"{len(poses)} postures"
                )
                # OPTIMISATION RAM: Limiter taille historique avec deque
                self.objects_detected = deque(
                    objects[: self._max_detections_history],
                    maxlen=self._max_detections_history,
                )
                self.faces_detected = deque(
                    faces[: self._max_detections_history],
                    maxlen=self._max_detections_history,
                )
                return {
                    "objects": objects,
                    "faces": faces,
                    "poses": poses,
                    "timestamp": datetime.now().isoformat(),
                    "source": "camera_sdk",
                }

        # OPTIMISATION RAM: Cache objets simulés réutilisé au lieu de recréer à chaque appel
        if not hasattr(self, "_simulated_objects_cache"):
            self._simulated_objects_cache = [
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
            self._simulated_faces_cache = [
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

        # Réutiliser cache au lieu de recréer
        objects = list(self._simulated_objects_cache)
        faces = list(self._simulated_faces_cache)

        # OPTIMISATION RAM: Convertir en list pour compatibilité, mais limiter taille
        self.objects_detected = deque(
            objects[: self._max_detections_history], maxlen=self._max_detections_history
        )
        self.faces_detected = deque(
            faces[: self._max_detections_history], maxlen=self._max_detections_history
        )

        return {
            "objects": objects,
            "faces": faces,
            "poses": [],  # Pas de pose en simulation
            "timestamp": datetime.now().isoformat(),
            "source": "simulation",  # Fallback simulation
        }

    def recognize_object(self, object_name: str) -> dict[str, Any] | None:
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

        # OPTIMISATION RAM: Convertir deque en list pour compatibilité API
        return list(self.faces_detected)

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
