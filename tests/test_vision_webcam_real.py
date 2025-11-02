#!/usr/bin/env python3
"""
Tests réels de vision BBIA avec webcam USB réelle.

Ces tests utilisent une vraie webcam (pas de mocks) pour valider:
- DeepFace sur images réelles
- MediaPipe Pose sur flux vidéo
- YOLO détection objets réels
- Fallback gracieux si modules absents

Prérequis:
    - Webcam USB branchée
    - venv-vision-py310 activé
    - Permissions caméra macOS accordées
"""

import os
import sys
from pathlib import Path

import pytest

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

try:
    import cv2

    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    from bbia_sim.bbia_vision import BBIAVision

    BBIA_VISION_AVAILABLE = True
except ImportError:
    BBIA_VISION_AVAILABLE = False


@pytest.mark.integration
@pytest.mark.skipif(
    not CV2_AVAILABLE or not BBIA_VISION_AVAILABLE,
    reason="OpenCV ou BBIAVision non disponible. Activez venv-vision-py310",
)
def test_webcam_capture_real() -> None:
    """Test capture image depuis webcam USB réelle."""
    camera_index = int(os.environ.get("BBIA_CAMERA_INDEX", "0"))

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        pytest.skip(f"Webcam index {camera_index} non disponible")

    try:
        ret, frame = cap.read()
        assert ret, "Impossible de lire une frame depuis la webcam"
        assert frame is not None, "Frame est None"
        assert frame.ndim == 3, f"Frame doit être 3D (H, W, C), reçu: {frame.ndim}D"
        assert frame.shape[2] == 3, "Frame doit avoir 3 canaux (BGR)"
        assert frame.dtype == "uint8", f"Frame doit être uint8, reçu: {frame.dtype}"

        height, width = frame.shape[:2]
        assert width > 0 and height > 0, f"Dimensions invalides: {width}x{height}"
        assert width >= 320 and height >= 240, "Résolution trop faible"
    finally:
        cap.release()


@pytest.mark.integration
@pytest.mark.skipif(
    not CV2_AVAILABLE or not BBIA_VISION_AVAILABLE,
    reason="OpenCV ou BBIAVision non disponible",
)
def test_bbia_vision_webcam_real() -> None:
    """Test BBIAVision avec webcam USB réelle."""
    camera_index = int(os.environ.get("BBIA_CAMERA_INDEX", "0"))

    # Vérifier que webcam est disponible
    cap_test = cv2.VideoCapture(camera_index)
    if not cap_test.isOpened():
        pytest.skip(f"Webcam index {camera_index} non disponible")
    cap_test.release()

    # Initialiser BBIAVision (sans robot, utilise OpenCV fallback)
    vision = BBIAVision(robot_api=None)

    # Capturer image
    image = vision.capture_image()
    assert image is not None, "BBIAVision n'a pas pu capturer depuis webcam"
    assert isinstance(image, type(vision.capture_image())), "Image invalide"

    # Scanner environnement
    result = vision.scan_environment()
    assert isinstance(result, dict), "scan_environment doit retourner un dict"
    assert "objects" in result, "Résultat doit contenir 'objects'"
    assert "faces" in result, "Résultat doit contenir 'faces'"


@pytest.mark.integration
@pytest.mark.skipif(
    not CV2_AVAILABLE or not BBIA_VISION_AVAILABLE,
    reason="OpenCV ou BBIAVision non disponible",
)
def test_deepface_webcam_real() -> None:
    """Test DeepFace sur image capturée depuis webcam réelle."""
    camera_index = int(os.environ.get("BBIA_CAMERA_INDEX", "0"))

    # Vérifier webcam
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        pytest.skip(f"Webcam index {camera_index} non disponible")

    try:
        ret, frame = cap.read()
        if not ret:
            pytest.skip("Impossible de capturer depuis webcam")

        vision = BBIAVision(robot_api=None)
        if not vision.face_recognition or not vision.face_recognition.is_initialized:
            pytest.skip("DeepFace non disponible ou non initialisé")

        # Détecter émotions sur image réelle
        result = vision.face_recognition.detect_emotions(frame)
        assert isinstance(
            result, list
        ), "Rétultat détection émotions doit être une liste"
        # Accepte 0 visages détectés (pas forcément quelqu'un devant la caméra)
    finally:
        cap.release()


@pytest.mark.integration
@pytest.mark.skipif(
    not CV2_AVAILABLE or not BBIA_VISION_AVAILABLE,
    reason="OpenCV ou BBIAVision non disponible",
)
def test_mediapipe_pose_webcam_real() -> None:
    """Test MediaPipe Pose sur flux webcam réel."""
    camera_index = int(os.environ.get("BBIA_CAMERA_INDEX", "0"))

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        pytest.skip(f"Webcam index {camera_index} non disponible")

    try:
        vision = BBIAVision(robot_api=None)
        if not vision.pose_detector:
            pytest.skip("MediaPipe Pose non disponible")

        # Capturer 5 frames et tester pose detection
        for _ in range(5):
            ret, frame = cap.read()
            if not ret:
                break

            pose_result = vision.pose_detector.detect_pose(frame)
            assert isinstance(pose_result, dict), "Résultat pose doit être un dict"
            # Accepte aucun point détecté (pas forcément quelqu'un devant caméra)
    finally:
        cap.release()


@pytest.mark.integration
@pytest.mark.skipif(
    not CV2_AVAILABLE or not BBIA_VISION_AVAILABLE,
    reason="OpenCV ou BBIAVision non disponible",
)
def test_yolo_webcam_real() -> None:
    """Test YOLO détection objets sur webcam réelle."""
    camera_index = int(os.environ.get("BBIA_CAMERA_INDEX", "0"))

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        pytest.skip(f"Webcam index {camera_index} non disponible")

    try:
        vision = BBIAVision(robot_api=None)
        if not vision.yolo_detector:
            pytest.skip("YOLO non disponible")

        ret, frame = cap.read()
        if not ret:
            pytest.skip("Impossible de capturer depuis webcam")

        # Détecter objets
        objects = vision.yolo_detector.detect(frame)
        assert isinstance(objects, list), "Résultat YOLO doit être une liste"
        # Accepte 0 objets (environnement peut être vide)
    finally:
        cap.release()


@pytest.mark.integration
@pytest.mark.hardware
def test_webcam_fallback_graceful() -> None:
    """Test que fallback fonctionne si webcam indisponible."""
    vision = BBIAVision(robot_api=None)

    # Simuler webcam indisponible (index invalide)
    original_index = os.environ.get("BBIA_CAMERA_INDEX")
    os.environ["BBIA_CAMERA_INDEX"] = "999"  # Index probablement inexistant

    try:
        # Doit retourner None gracieusement, pas crasher
        image = vision._capture_from_opencv_camera()
        # Accepte None si webcam indisponible
        assert image is None or isinstance(
            image, type(vision.capture_image())
        ), "Fallback doit gérer webcam indisponible gracieusement"
    finally:
        if original_index:
            os.environ["BBIA_CAMERA_INDEX"] = original_index
        else:
            os.environ.pop("BBIA_CAMERA_INDEX", None)
