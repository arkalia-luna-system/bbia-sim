#!/usr/bin/env python3

"""BBIA Vision - Module de vision avancé pour Reachy Mini Wireless
Reconnaissance d'objets, détection de visages, suivi d'objets.
"""

import logging
import math
import os
import queue
import sys
import threading
import time
from collections import deque
from collections.abc import Callable
from datetime import datetime
from typing import TYPE_CHECKING, Any, cast

# Note: Module error_handling disponible pour factorisation future
# from .utils.error_handling import safe_execute

if TYPE_CHECKING:
    from .face_recognition import BBIAPersonRecognition
    from .pose_detection import BBIAPoseDetection

import contextlib

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
    os.environ.setdefault("GLOG_minloglevel", "2")  # 0=INFO,1=WARNING,2=ERROR
    os.environ.setdefault(
        "TF_CPP_MIN_LOG_LEVEL",
        "3",
    )  # 0=INFO,1=WARNING,2=ERROR,3=FATAL
    os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")  # éviter logs GPU inutiles
    # Supprimer les logs TensorFlow Lite
    os.environ.setdefault("TFLITE_LOG_VERBOSITY", "0")  # 0=ERROR, 1=WARNING, 2=INFO
    # Supprimer les warnings MediaPipe/TensorFlow en CI
    if os.environ.get("CI", "false").lower() == "true":
        # Rediriger stderr pour MediaPipe en CI (warnings normaux mais bruyants)
        import warnings

        warnings.filterwarnings("ignore", category=UserWarning, module="mediapipe")
        warnings.filterwarnings("ignore", category=UserWarning, module="tensorflow")
    # Supprimer les logs OpenGL (ne pas définir MUJOCO_GL sur macOS, utilise la valeur par défaut)
    # Sur macOS, laisser MuJoCo choisir automatiquement (glfw ou egl)
    if sys.platform != "darwin":  # Pas macOS
        os.environ.setdefault("MUJOCO_GL", "egl")  # Utiliser EGL sur Linux/Windows
except (OSError, RuntimeError, ValueError, TypeError) as e:
    logger.debug(
        "Impossible de configurer variables d'environnement MediaPipe/TensorFlow: %s",
        e,
    )
except Exception as e:  # noqa: BLE001 - Fallback pour erreurs inattendues
    logger.debug("Erreur inattendue configuration variables d'environnement: %s", e)

# Import conditionnel pour YOLO et MediaPipe
try:
    from .vision_yolo import create_yolo_detector

    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    import mediapipe as mp  # type: ignore[import-untyped]

    # Vérifier que mp.solutions existe (peut être absent même si import réussit)
    if hasattr(mp, "solutions"):
        MEDIAPIPE_AVAILABLE = True
    else:
        MEDIAPIPE_AVAILABLE = False
        mp = None
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    mp = None

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
    from .pose_detection import (
        MEDIAPIPE_POSE_AVAILABLE as _mediapipe_pose_available,
    )
    from .pose_detection import (
        create_pose_detector as _create_pose_detector_imported,
    )

    create_pose_detector = _create_pose_detector_imported
    MEDIAPIPE_POSE_AVAILABLE = _mediapipe_pose_available
except ImportError:
    pass  # create_pose_detector reste None

# Import conditionnel cv2 pour conversions couleur
CV2_AVAILABLE = False
try:
    import cv2  # type: ignore[import-untyped]

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
        """Initialise le module de vision.

        Args:
            robot_api: Interface RobotAPI (optionnel) pour accès robot.media.camera

        """
        self._warn_if_not_singleton()
        self._init_basic_attributes(robot_api)
        self._init_threading()
        self._init_camera_buffer()
        self._init_specs()
        self._init_sdk_camera(robot_api)
        self._init_opencv_camera()
        self._init_yolo_detector()
        self._init_mediapipe_face_detector()
        self._init_deepface()
        self._init_pose_detector()

    def _warn_if_not_singleton(self) -> None:
        """Avertit si l'instance n'est pas créée via le singleton."""
        global _bbia_vision_singleton
        if _bbia_vision_singleton is not None and _bbia_vision_singleton is not self:
            if "pytest" in sys.modules or "unittest" in sys.modules:
                logger.debug(
                    "⚠️ Instance BBIAVision créée directement. "
                    "Utilisez get_bbia_vision_singleton() pour éviter duplication RAM.",
                )
            else:
                logger.warning(
                    "⚠️ Instance BBIAVision créée directement. "
                    "Utilisez get_bbia_vision_singleton() pour éviter duplication RAM.",
                )

    def _init_basic_attributes(self, robot_api: Any | None) -> None:
        """Initialise les attributs de base."""
        self.robot_api = robot_api
        self.camera_active = True
        self.vision_quality = "HD"
        self.detection_range = 3.0
        self._max_detections_history = 50
        self.objects_detected: deque[dict[str, Any]] = deque(
            maxlen=self._max_detections_history,
        )
        self.faces_detected: deque[dict[str, Any]] = deque(
            maxlen=self._max_detections_history,
        )
        self.tracking_active = False
        self.current_focus: dict[str, Any] | None = None

    def _init_threading(self) -> None:
        """Initialise les attributs de threading pour scan asynchrone."""
        self._scan_thread: threading.Thread | None = None
        self._scan_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=1)
        self._last_scan_result: dict[str, Any] | None = None
        self._scan_lock = threading.Lock()
        self._scan_interval = 0.1
        self._async_scan_active = False
        self._should_stop_scan = threading.Event()

    def _init_camera_buffer(self) -> None:
        """Initialise le buffer circulaire pour frames caméra."""
        buffer_size = int(os.environ.get("BBIA_CAMERA_BUFFER_SIZE", "10"))
        self._camera_frame_buffer: deque[npt.NDArray[np.uint8]] = deque(
            maxlen=buffer_size,
        )
        self._buffer_overrun_count = 0

    def _init_specs(self) -> None:
        """Initialise les spécifications hardware."""
        self.specs = {
            "camera": "Grand angle",
            "resolution": "1280x720 (simulation) / HD grand-angle (réel)",
            "fov": "80° (simulation) / ~120° (réel estimé)",
            "focus": "Auto",
            "night_vision": False,
        }

    def _init_sdk_camera(self, robot_api: Any | None) -> None:
        """Initialise la caméra SDK si disponible."""
        self._camera_sdk_available = False
        self._camera = None
        if not robot_api or not hasattr(robot_api, "media"):
            return

        try:
            media = robot_api.media
            if not media:
                return
            self._camera = getattr(media, "camera", None)
            if self._camera is None:
                return

            from ..backends.simulation_shims import (  # type: ignore[import-untyped]
                SimulationCamera,
            )

            if isinstance(self._camera, SimulationCamera):
                self._camera_sdk_available = False
                logger.debug("Caméra simulation (shim) disponible")
            else:
                self._camera_sdk_available = True
                logger.info("✅ Caméra SDK disponible: robot.media.camera")
        except (AttributeError, RuntimeError, OSError) as e:
            logger.debug("Caméra SDK non disponible (fallback simulation): %s", e)
        except Exception as e:  # noqa: BLE001
            logger.debug("Erreur inattendue caméra SDK: %s", e)

    def _init_opencv_camera(self) -> None:
        """Initialise la webcam USB via OpenCV (fallback)."""
        self._opencv_camera = None
        self._opencv_camera_available = False

        if self._camera_sdk_available or not CV2_AVAILABLE or not cv2:
            return

        try:
            camera_index_str = os.environ.get("BBIA_CAMERA_INDEX", "0")
            camera_device = os.environ.get("BBIA_CAMERA_DEVICE")

            if camera_device:
                self._opencv_camera = cv2.VideoCapture(camera_device)
                logger.info("🔌 Tentative ouverture webcam: %s", camera_device)
            else:
                try:
                    camera_index = int(camera_index_str)
                    self._opencv_camera = cv2.VideoCapture(camera_index)
                    logger.info(
                        "🔌 Tentative ouverture webcam (index %d)", camera_index
                    )
                except ValueError:
                    logger.warning(
                        "⚠️ BBIA_CAMERA_INDEX invalide: %s, utilisation index 0",
                        camera_index_str,
                    )
                    self._opencv_camera = cv2.VideoCapture(0)

            if self._opencv_camera is not None and self._opencv_camera.isOpened():
                ret, test_frame = self._opencv_camera.read()
                if ret and test_frame is not None:
                    self._opencv_camera_available = True
                    logger.info("✅ Webcam USB OpenCV disponible")
                else:
                    self._opencv_camera.release()
                    self._opencv_camera = None
                    logger.debug("Webcam OpenCV ouverte mais ne capture pas d'images")
            else:
                if self._opencv_camera:
                    self._opencv_camera.release()
                self._opencv_camera = None
                logger.debug("Webcam OpenCV non disponible (fallback simulation)")
        except (OSError, RuntimeError, AttributeError) as e:
            self._release_opencv_camera_on_error()
            logger.debug("Erreur initialisation webcam OpenCV: %s", e)
        except Exception as e:  # noqa: BLE001
            self._opencv_camera = None
            logger.debug("Erreur inattendue initialisation webcam OpenCV: %s", e)

    def _release_opencv_camera_on_error(self) -> None:
        """Libère la webcam OpenCV en cas d'erreur."""
        if self._opencv_camera:
            try:
                self._opencv_camera.release()
            except (OSError, RuntimeError) as release_error:
                logger.debug("Erreur libération webcam OpenCV: %s", release_error)
            except Exception as release_error:  # noqa: BLE001
                logger.debug(
                    "Erreur inattendue libération webcam OpenCV: %s", release_error
                )
        self._opencv_camera = None

    def _init_yolo_detector(self) -> None:
        """Initialise le détecteur YOLO si caméra SDK disponible."""
        self.yolo_detector = None

        if not (
            self._camera_sdk_available
            and YOLO_AVAILABLE
            and create_yolo_detector is not None
        ):
            logger.debug(
                "YOLO non chargé (lazy loading - caméra simulation ou non disponible)"
            )
            return

        try:
            confidence_threshold = float(os.environ.get("BBIA_YOLO_CONFIDENCE", "0.25"))
            self.yolo_detector = create_yolo_detector(
                model_size="n",
                confidence_threshold=confidence_threshold,
            )
            if self.yolo_detector:
                self.yolo_detector.load_model()
                logger.info(
                    "✅ Détecteur YOLO initialisé (lazy loading - caméra réelle)"
                )
        except (ImportError, RuntimeError, AttributeError) as e:
            logger.warning("⚠️ YOLO non disponible: %s", e)
        except Exception as e:  # noqa: BLE001
            logger.error("⚠️ Erreur inattendue YOLO (critique): %s", e)

    def _init_mediapipe_face_detector(self) -> None:
        """Initialise le détecteur MediaPipe Face."""
        self.face_detector = None

        if not MEDIAPIPE_AVAILABLE or not mp:
            return

        try:
            self._try_init_mediapipe_from_cache()
        except (ImportError, RuntimeError, AttributeError) as e:
            logger.warning("⚠️ MediaPipe non disponible: %s", e)
        except Exception as e:  # noqa: BLE001
            logger.error(
                "⚠️ MediaPipe non disponible (erreur inattendue critique): %s", e
            )

    def _try_init_mediapipe_from_cache(self) -> None:
        """Tente d'initialiser MediaPipe depuis le cache."""
        try:
            from .vision_yolo import (
                _mediapipe_cache_lock,
                _mediapipe_face_detection_cache,
            )

            with _mediapipe_cache_lock:
                if (
                    _mediapipe_face_detection_cache is not None
                    and MEDIAPIPE_AVAILABLE
                    and mp is not None
                ):
                    logger.debug(
                        "♻️ Réutilisation détecteur MediaPipe depuis cache (bbia_vision)"
                    )
                    self.face_detector = _mediapipe_face_detection_cache
                    logger.debug("✅ Détecteur MediaPipe Face initialisé (cache)")
                elif _mediapipe_face_detection_cache is not None:
                    logger.debug("🧹 Nettoyage cache MediaPipe (non disponible)")
                else:
                    self._create_new_mediapipe_detector()
        except ImportError:
            self._create_mediapipe_detector_fallback()

    def _create_new_mediapipe_detector(self) -> None:
        """Crée un nouveau détecteur MediaPipe et le met en cache."""
        if not hasattr(mp, "solutions"):
            raise AttributeError("mediapipe module has no attribute 'solutions'")
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            model_selection=0,
            min_detection_confidence=0.5,
        )
        logger.debug("✅ Détecteur MediaPipe Face initialisé")

    def _create_mediapipe_detector_fallback(self) -> None:
        """Crée un détecteur MediaPipe sans cache (fallback)."""
        if mp is not None and hasattr(mp, "solutions"):
            self.face_detector = mp.solutions.face_detection.FaceDetection(
                model_selection=0,
                min_detection_confidence=0.5,
            )
            logger.debug("✅ Détecteur MediaPipe Face initialisé")
        else:
            raise AttributeError("mediapipe module has no attribute 'solutions'")

    def _init_deepface(self) -> None:
        """Initialise DeepFace pour reconnaissance visage."""
        self.face_recognition = None

        if not DEEPFACE_AVAILABLE or create_face_recognition is None:
            return

        try:
            db_path = os.environ.get("BBIA_FACES_DB", "faces_db")
            model_name = os.environ.get("BBIA_DEEPFACE_MODEL", "VGG-Face")
            self.face_recognition = create_face_recognition(db_path, model_name)
            if self.face_recognition and self.face_recognition.is_initialized:
                logger.info(
                    "✅ DeepFace initialisé (db: %s, modèle: %s)", db_path, model_name
                )
        except (ImportError, RuntimeError, AttributeError) as e:
            logger.debug("⚠️ DeepFace non disponible: %s", e)
        except (TypeError, ValueError, OSError) as e:
            logger.debug("⚠️ DeepFace non disponible (type/value/os): %s", e)
        except Exception as e:  # noqa: BLE001
            logger.debug("⚠️ DeepFace non disponible (erreur inattendue): %s", e)

    def _init_pose_detector(self) -> None:
        """Initialise MediaPipe Pose pour détection postures/gestes."""
        self.pose_detector = None

        if not MEDIAPIPE_POSE_AVAILABLE or create_pose_detector is None:
            return

        try:
            model_complexity = int(os.environ.get("BBIA_POSE_COMPLEXITY", "1"))
            self.pose_detector = create_pose_detector(model_complexity=model_complexity)
            if self.pose_detector and self.pose_detector.is_initialized:
                logger.info(
                    "✅ MediaPipe Pose initialisé (complexité: %d)", model_complexity
                )
        except (ImportError, RuntimeError, AttributeError) as e:
            logger.warning("⚠️ MediaPipe Pose non disponible: %s", e)
        except (TypeError, ValueError, OSError) as e:
            logger.warning("⚠️ MediaPipe Pose non disponible (type/value/os): %s", e)
        except Exception as e:  # noqa: BLE001
            logger.warning("⚠️ MediaPipe Pose non disponible (erreur inattendue): %s", e)

    def _capture_image_from_camera(self) -> npt.NDArray[np.uint8] | None:
        """Capture une image depuis robot.media.camera si disponible,
        sinon webcam USB OpenCV.

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

    def _convert_to_numpy_array(self, image: Any) -> npt.NDArray[np.uint8] | None:
        """Convertit une image en numpy array.

        Args:
            image: Image à convertir (peut être numpy, PIL, torch, etc.)

        Returns:
            Image numpy array ou None si conversion impossible

        """
        if isinstance(image, np.ndarray):
            return image

        try:
            if hasattr(image, "toarray"):
                return image.toarray()
            if hasattr(image, "numpy"):
                return image.numpy()
            if hasattr(image, "__array__"):
                return np.array(image)
            logger.warning("Format d'image non supporté par SDK camera")
            return None
        except (ValueError, TypeError, AttributeError) as e:
            logger.debug("Erreur conversion image: %s", e)
        except (IndexError, KeyError) as e:
            logger.debug("Erreur conversion image (index/key): %s", e)
        except Exception as e:  # noqa: BLE001
            logger.debug("Erreur inattendue conversion image: %s", e)
        return None

    def _validate_and_convert_image(
        self, image: npt.NDArray[np.uint8]
    ) -> npt.NDArray[np.uint8] | None:
        """Valide et convertit le format d'image.

        Args:
            image: Image numpy array à valider

        Returns:
            Image validée en BGR ou None si invalide

        """
        # Valider shape (doit être 2D ou 3D)
        if image.ndim < 2 or image.ndim > 3:
            logger.warning(
                "Format image invalide (ndim=%d), attendu 2 ou 3 dimensions",
                image.ndim,
            )
            return None

        # Convertir format couleur vers BGR
        if not CV2_AVAILABLE or cv2 is None:
            logger.debug("cv2 non disponible, format image supposé correct")
        elif image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif image.ndim == 3:
            channels = image.shape[2]
            if channels == 1:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            elif channels == 4:
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            elif channels == 3:
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            else:
                logger.warning(
                    "Nombre de canaux invalide (%d), attendu 1, 3 ou 4", channels
                )
                return None

        # Validation finale
        if image.size == 0:
            logger.warning("Image vide retournée par SDK camera")
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
                logger.debug("Erreur conversion dtype: %s", e)
                return None

        return image

    def _add_to_camera_buffer(self, image: npt.NDArray[np.uint8]) -> None:
        """Ajoute une image au buffer circulaire de la caméra."""
        buffer_maxlen = self._camera_frame_buffer.maxlen
        if (
            buffer_maxlen is not None
            and len(self._camera_frame_buffer) >= buffer_maxlen
        ):
            self._buffer_overrun_count += 1
            if self._buffer_overrun_count % 100 == 0:
                logger.warning(
                    "⚠️ Camera buffer overrun: %d frames perdues (buffer size: %d)",
                    self._buffer_overrun_count,
                    buffer_maxlen,
                )
        self._camera_frame_buffer.append(image.copy())

    def _capture_raw_from_sdk(self) -> Any | None:
        """Capture une image brute depuis le SDK."""
        camera = self._camera
        if camera is None:
            return None
        if hasattr(camera, "get_image"):
            return camera.get_image()  # type: ignore[union-attr]
        if hasattr(camera, "capture"):
            return camera.capture()  # type: ignore[union-attr]
        if hasattr(camera, "read"):
            ret, image = camera.read()  # type: ignore[union-attr]
            return image if ret else None
        if callable(camera):
            return camera()
        logger.warning(
            "⚠️ robot.media.camera disponible mais méthode de capture inconnue"
        )
        return None

    def _capture_from_sdk_camera(self) -> npt.NDArray[np.uint8] | None:
        """Capture depuis robot.media.camera (SDK Reachy Mini)."""
        if not self._camera_sdk_available or not self._camera:
            return None

        try:
            # Capturer image brute
            raw_image = self._capture_raw_from_sdk()
            if raw_image is None:
                return None

            # Convertir en numpy array
            image = self._convert_to_numpy_array(raw_image)
            if image is None:
                return None

            # Valider et convertir format
            image = self._validate_and_convert_image(image)
            if image is None:
                return None

            logger.debug("✅ Image capturée depuis robot.media.camera (format validé)")

            # Ajouter au buffer circulaire
            self._add_to_camera_buffer(image)

            return cast("npt.NDArray[np.uint8]", image)

        except (AttributeError, RuntimeError, OSError) as e:
            logger.warning(
                "Caméra SDK indisponible: %s. Vérifier câble, daemon reachy-mini ou redémarrer l'app.",
                e,
            )
        except (TypeError, IndexError, KeyError) as e:
            logger.debug("Erreur capture caméra SDK (type/index/key): %s", e)
        except Exception as e:  # noqa: BLE001
            logger.warning(
                "Caméra SDK erreur inattendue: %s. Vérifier que le daemon et la caméra sont opérationnels.",
                e,
            )

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
                logger.warning("Format image invalide (ndim=%s)", image.ndim)
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
                    logger.debug("Erreur conversion dtype: %s", e)
                    return None

            logger.debug("✅ Image capturée depuis webcam USB OpenCV")

            # Ajouter au buffer circulaire (Issue #16 SDK officiel)
            buffer_maxlen = self._camera_frame_buffer.maxlen
            if (
                buffer_maxlen is not None
                and len(self._camera_frame_buffer) >= buffer_maxlen
            ):
                self._buffer_overrun_count += 1
                if self._buffer_overrun_count % 100 == 0:
                    logger.warning(
                        (
                            f"⚠️ Camera buffer overrun: "
                            f"{self._buffer_overrun_count} frames perdues "
                            f"(buffer size: {buffer_maxlen})"
                        ),
                    )
            self._camera_frame_buffer.append(image.copy())

            return cast("npt.NDArray[np.uint8]", image)

        except (OSError, RuntimeError, AttributeError) as e:
            logger.debug("Erreur capture webcam OpenCV: %s", e)
            return None
        except (TypeError, IndexError, KeyError) as e:
            logger.debug("Erreur capture webcam OpenCV (type/index/key): %s", e)
            return None
        except (
            Exception
        ) as e:  # noqa: BLE001 - Fallback final pour erreurs vraiment inattendues
            logger.debug("Erreur inattendue capture webcam OpenCV: %s", e)
            return None

    def _detect_objects_yolo(
        self, image: npt.NDArray[np.uint8]
    ) -> list[dict[str, Any]]:
        """Détecte les objets avec YOLO.

        Args:
            image: Image BGR numpy array

        Returns:
            Liste des objets détectés

        """
        objects: list[dict[str, Any]] = []

        if not self.yolo_detector or not self.yolo_detector.is_loaded:
            return objects

        try:
            detections = self.yolo_detector.detect_objects(image)
            height, width = image.shape[:2]

            for det in detections:
                bbox_yolo = det.get("bbox", [])
                if not isinstance(bbox_yolo, list) or len(bbox_yolo) != 4:
                    continue

                x1, y1, x2, y2 = bbox_yolo
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                w = x2 - x1
                h = y2 - y1

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
        except (AttributeError, RuntimeError, ValueError) as e:
            logger.warning("Erreur détection YOLO: %s", e)
        except (TypeError, IndexError, OSError) as e:
            logger.warning("Erreur détection YOLO (type/index/os): %s", e)
        except Exception as e:  # noqa: BLE001
            logger.warning("Erreur inattendue détection YOLO: %s", e)

        return objects

    def _analyze_face_with_deepface(
        self,
        face_roi: npt.NDArray[np.uint8] | None,
    ) -> tuple[str, str, float]:
        """Analyse un visage avec DeepFace.

        Returns:
            Tuple (nom_reconnu, emotion, confiance_emotion)

        """
        recognized_name = "humain"
        detected_emotion = "neutral"
        emotion_confidence = 0.0

        if (
            face_roi is None
            or face_roi.size == 0
            or self.face_recognition is None
            or not self.face_recognition.is_initialized
        ):
            return recognized_name, detected_emotion, emotion_confidence

        try:
            person_result = self.face_recognition.recognize_person(
                face_roi, enforce_detection=False
            )
            if person_result:
                recognized_name = str(person_result.get("name", recognized_name))

            emotion_result = self.face_recognition.detect_emotion(
                face_roi, enforce_detection=False
            )
            if emotion_result:
                detected_emotion = emotion_result["emotion"]
                emotion_confidence = emotion_result["confidence"]
        except (ValueError, RuntimeError, AttributeError) as e:
            logger.debug("DeepFace erreur: %s", e)
        except (TypeError, IndexError, KeyError) as e:
            logger.debug("DeepFace erreur (type/index/key): %s", e)
        except Exception as e:  # noqa: BLE001
            logger.debug("DeepFace erreur inattendue: %s", e)

        return recognized_name, detected_emotion, emotion_confidence

    def _detect_faces_mediapipe(
        self, image: npt.NDArray[np.uint8]
    ) -> list[dict[str, Any]]:
        """Détecte les visages avec MediaPipe + DeepFace.

        Args:
            image: Image BGR numpy array

        Returns:
            Liste des visages détectés

        """
        faces: list[dict[str, Any]] = []

        if not self.face_detector or not CV2_AVAILABLE:
            return faces

        try:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.face_detector.process(image_rgb)

            if not results.detections:
                return faces

            height, width = image.shape[:2]
            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box

                # Extraire ROI pour DeepFace
                face_roi = None
                if self.face_recognition and self.face_recognition.is_initialized:
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

                # Analyser avec DeepFace
                recognized_name, detected_emotion, emotion_confidence = (
                    self._analyze_face_with_deepface(face_roi)
                )

                # Calculer bbox et centre
                bbox_x = int(bbox.xmin * width)
                bbox_y = int(bbox.ymin * height)
                bbox_w = int(bbox.width * width)
                bbox_h = int(bbox.height * height)
                center_x = bbox_x + bbox_w / 2
                center_y = bbox_y + bbox_h / 2

                face = {
                    "name": recognized_name,
                    "distance": 1.5,
                    "confidence": detection.score[0] if detection.score else 0.8,
                    "emotion": detected_emotion,
                    "emotion_confidence": emotion_confidence,
                    "position": (
                        bbox.xmin + bbox.width / 2,
                        bbox.ymin + bbox.height / 2,
                    ),
                    "bbox": {
                        "x": bbox_x,
                        "y": bbox_y,
                        "width": bbox_w,
                        "height": bbox_h,
                        "center_x": int(center_x),
                        "center_y": int(center_y),
                    },
                }
                faces.append(face)
        except (AttributeError, RuntimeError, ValueError) as e:
            logger.warning("Erreur détection MediaPipe: %s", e)
        except (TypeError, IndexError, OSError) as e:
            logger.warning("Erreur détection MediaPipe (type/index/os): %s", e)
        except Exception as e:  # noqa: BLE001
            logger.warning("Erreur inattendue détection MediaPipe: %s", e)

        return faces

    def _detect_poses_mediapipe(
        self, image: npt.NDArray[np.uint8]
    ) -> list[dict[str, Any]]:
        """Détecte les postures avec MediaPipe Pose.

        Args:
            image: Image BGR numpy array

        Returns:
            Liste des postures détectées

        """
        poses: list[dict[str, Any]] = []

        if not self.pose_detector or not self.pose_detector.is_initialized:
            return poses

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
        except (AttributeError, RuntimeError, ValueError) as e:
            logger.debug("Erreur détection pose: %s", e)
        except (TypeError, IndexError, OSError) as e:
            logger.debug("Erreur détection pose (type/index/os): %s", e)
        except Exception as e:  # noqa: BLE001
            logger.debug("Erreur inattendue détection pose: %s", e)

        return poses

    def _update_detection_history(
        self,
        objects: list[dict[str, Any]],
        faces: list[dict[str, Any]],
    ) -> None:
        """Met à jour l'historique des détections."""
        self.objects_detected = deque(
            objects[: self._max_detections_history],
            maxlen=self._max_detections_history,
        )
        self.faces_detected = deque(
            faces[: self._max_detections_history],
            maxlen=self._max_detections_history,
        )

    def scan_environment_from_image(
        self,
        image: npt.NDArray[np.uint8],
    ) -> dict[str, Any]:
        """Scanne l'environnement depuis une image fournie (au lieu de la caméra).

        Args:
            image: Image numpy array (BGR)

        Returns:
            Dict avec objets, visages, postures détectés

        """
        objects = self._detect_objects_yolo(image)
        faces = self._detect_faces_mediapipe(image)
        poses = self._detect_poses_mediapipe(image)

        self._update_detection_history(objects, faces)

        return {
            "objects": objects,
            "faces": faces,
            "poses": poses,
            "timestamp": datetime.now().isoformat(),
            "source": "image_upload",
        }

    def _init_simulation_cache(self) -> None:
        """Initialise le cache de données simulées si nécessaire."""
        if hasattr(self, "_simulated_objects_cache"):
            return

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

    def _get_simulation_result(self) -> dict[str, Any]:
        """Retourne un résultat de scan simulé."""
        self._init_simulation_cache()

        objects = list(self._simulated_objects_cache)
        faces = list(self._simulated_faces_cache)

        self._update_detection_history(objects, faces)

        return {
            "objects": objects,
            "faces": faces,
            "poses": [],
            "timestamp": datetime.now().isoformat(),
            "source": "simulation",
        }

    def scan_environment(self) -> dict[str, Any]:
        """Scanne l'environnement et détecte les objets.

        Utilise robot.media.camera si disponible (SDK officiel)
        avec détection YOLO/MediaPipe,
        sinon utilise simulation pour compatibilité.
        """
        # Si SDK caméra non disponible, retourner simulation
        if not self._camera_sdk_available:
            return self._get_simulation_result()

        # Capturer image depuis caméra
        image = self._capture_image_from_camera()

        if image is not None:
            # Utiliser les fonctions de détection refactorisées
            objects = self._detect_objects_yolo(image)
            faces = self._detect_faces_mediapipe(image)
            poses = self._detect_poses_mediapipe(image)

            if objects or faces or poses:
                logger.info(
                    "✅ Détection réelle: %d objets, %d visages, %d postures",
                    len(objects),
                    len(faces),
                    len(poses),
                )
                self._update_detection_history(objects, faces)
                return {
                    "objects": objects,
                    "faces": faces,
                    "poses": poses,
                    "timestamp": datetime.now().isoformat(),
                    "source": "camera_sdk",
                }

        # Fallback simulation si pas de détection
        return self._get_simulation_result()

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
        return math.sqrt(x**2 + y**2)

    def get_latest_frame(self) -> npt.NDArray[np.uint8] | None:
        """Récupère la frame la plus récente du buffer circulaire.

        Permet d'accéder à la dernière image capturée sans appeler scan_environment().
        Utile pour éviter perte de frames si pas consommées assez vite
        (Issue #16 SDK officiel).

        Returns:
            Dernière frame capturée ou None si buffer vide

        """
        if self._camera_frame_buffer:
            return self._camera_frame_buffer[-1]
        return None

    def _scan_thread_worker(self) -> None:
        """Worker thread pour scans asynchrones en arrière-plan."""
        logger.debug("🔍 Thread scan asynchrone démarré")
        while not self._should_stop_scan.is_set():
            try:
                if self.camera_active:
                    # Effectuer scan (synchrone dans le thread)
                    result = self._scan_environment_sync()
                    with self._scan_lock:
                        self._last_scan_result = result
                    # Mettre à jour queue (remplace ancien résultat si queue pleine)
                    try:
                        self._scan_queue.put_nowait(result)
                    except queue.Full:
                        # Remplacer ancien résultat
                        with contextlib.suppress(queue.Empty):
                            self._scan_queue.get_nowait()
                        self._scan_queue.put_nowait(result)
                # Attendre intervalle avant prochain scan
                self._should_stop_scan.wait(self._scan_interval)
            except (RuntimeError, AttributeError, OSError) as e:
                logger.exception("Erreur thread scan asynchrone: %s", e)
                time.sleep(self._scan_interval)
            except (TypeError, IndexError, KeyError) as e:
                logger.exception(
                    "Erreur thread scan asynchrone (type/index/key): %s", e
                )
                time.sleep(self._scan_interval)
            except (
                Exception
            ) as e:  # noqa: BLE001 - Fallback final pour erreurs vraiment inattendues
                logger.exception("Erreur inattendue thread scan asynchrone: %s", e)
                time.sleep(self._scan_interval)

        logger.debug("🔍 Thread scan asynchrone arrêté")

    def start_async_scanning(self, interval: float = 0.1) -> bool:
        """Démarre le scan asynchrone en arrière-plan.

        Args:
            interval: Intervalle entre scans en secondes (défaut: 0.1s = 10 FPS)

        Returns:
            True si démarré avec succès, False sinon

        """
        # OPTIMISATION RAM: Lock pour éviter création threads multiples
        with self._scan_lock:
            if self._async_scan_active:
                logger.debug("Scan asynchrone déjà actif")
                return True

            # OPTIMISATION RAM: Vérifier si thread existe déjà et est actif
            if self._scan_thread is not None and self._scan_thread.is_alive():
                logger.warning("Thread scan déjà actif, réutilisation")
                return True

            self._scan_interval = max(0.05, interval)  # Min 50ms (20 FPS max)
            self._should_stop_scan.clear()
            self._async_scan_active = True

            self._scan_thread = threading.Thread(
                target=self._scan_thread_worker,
                daemon=True,
                name="BBIAVision-ScanThread",
            )
            self._scan_thread.start()
            logger.info(
                "✅ Scan asynchrone démarré (intervalle: %ss)",
                self._scan_interval,
            )
            return True

    def stop_async_scanning(self) -> None:
        """Arrête le scan asynchrone en arrière-plan."""
        if not self._async_scan_active:
            return

        self._should_stop_scan.set()
        self._async_scan_active = False

        if self._scan_thread and self._scan_thread.is_alive():
            self._scan_thread.join(timeout=1.0)
            if self._scan_thread.is_alive():
                logger.warning(
                    "Thread scan asynchrone n'a pas pu être arrêté proprement",
                )

        logger.info("✅ Scan asynchrone arrêté")

    def scan_environment_async(
        self,
        timeout: float | None = None,
    ) -> dict[str, Any] | None:
        """Scanne l'environnement de manière asynchrone (non-bloquant).

        Args:
            timeout: Timeout en secondes pour attendre résultat (None = retour immédiat)

        Returns:
            Résultat du scan ou None si pas disponible

        """
        # Si scan asynchrone actif, retourner dernier résultat
        if self._async_scan_active:
            with self._scan_lock:
                if self._last_scan_result is not None:
                    return (
                        self._last_scan_result.copy()
                        if isinstance(self._last_scan_result, dict)
                        else self._last_scan_result
                    )

            # Attendre nouveau résultat si timeout spécifié
            if timeout is not None and timeout > 0:
                try:
                    result = self._scan_queue.get(timeout=timeout)
                    with self._scan_lock:
                        self._last_scan_result = result
                    return result
                except queue.Empty:
                    return None

        # Fallback: scan synchrone si asynchrone non actif
        return self.scan_environment()

    def _scan_environment_sync(self) -> dict[str, Any]:
        """Version interne synchrone de scan_environment (utilisée par thread).

        Returns:
            Résultat du scan

        """
        # Utiliser la méthode scan_environment existante
        return self.scan_environment()

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
            "camera_buffer_size": len(self._camera_frame_buffer),
            "camera_buffer_max": self._camera_frame_buffer.maxlen,
            "buffer_overruns": self._buffer_overrun_count,
            "async_scan_active": self._async_scan_active,
            "scan_interval": self._scan_interval,
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
