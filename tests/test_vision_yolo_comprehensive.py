#!/usr/bin/env python3
"""
Tests complets pour vision_yolo.py - Amélioration coverage 49% → 70%+
"""

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from bbia_sim.vision_yolo import (
    FaceDetector,
    YOLODetector,
    create_face_detector,
    create_yolo_detector,
)


@pytest.mark.unit
@pytest.mark.fast
class TestYOLODetector:
    """Tests pour YOLODetector."""

    def test_init_without_yolo(self):
        """Test initialisation sans YOLO disponible."""
        with patch("bbia_sim.vision_yolo.YOLO_AVAILABLE", False):
            detector = YOLODetector(model_size="n", confidence_threshold=0.25)
            assert detector.model_size == "n"
            assert detector.confidence_threshold == 0.25
            assert detector.model is None
            assert detector.is_loaded is False
            assert isinstance(detector.target_classes, dict)

    def test_init_with_yolo(self):
        """Test initialisation avec YOLO disponible."""
        with patch("bbia_sim.vision_yolo.YOLO_AVAILABLE", True):
            detector = YOLODetector(model_size="s", confidence_threshold=0.5)
            assert detector.model_size == "s"
            assert detector.confidence_threshold == 0.5
            assert detector.model is None
            assert detector.is_loaded is False

    @patch("bbia_sim.vision_yolo.YOLO")
    def test_load_model_success(self, mock_yolo_class):
        """Test chargement modèle réussi."""
        with patch("bbia_sim.vision_yolo.YOLO_AVAILABLE", True):
            mock_model = MagicMock()
            mock_yolo_class.return_value = mock_model

            detector = YOLODetector(model_size="n", confidence_threshold=0.25)
            result = detector.load_model()

            assert result is True
            assert detector.is_loaded is True
            assert detector.model == mock_model
            mock_yolo_class.assert_called_once_with("yolov8n.pt")

    @patch("bbia_sim.vision_yolo.YOLO")
    def test_load_model_failure(self, mock_yolo_class):
        """Test chargement modèle échoué."""
        with patch("bbia_sim.vision_yolo.YOLO_AVAILABLE", True):
            mock_yolo_class.side_effect = Exception("Erreur chargement")

            detector = YOLODetector(model_size="n", confidence_threshold=0.25)
            result = detector.load_model()

            assert result is False
            assert detector.is_loaded is False

    def test_load_model_without_yolo(self):
        """Test load_model sans YOLO."""
        with patch("bbia_sim.vision_yolo.YOLO_AVAILABLE", False):
            detector = YOLODetector(model_size="n", confidence_threshold=0.25)
            result = detector.load_model()
            assert result is False

    @patch("bbia_sim.vision_yolo.YOLO")
    def test_detect_objects_success(self, mock_yolo_class):
        """Test détection objets réussie."""
        with patch("bbia_sim.vision_yolo.YOLO_AVAILABLE", True):
            # Mock résultats YOLO
            mock_box = MagicMock()
            mock_box.xyxy = [[[10, 20, 100, 120]]]
            mock_box.conf = [[0.8]]
            mock_box.cls = [[0]]

            mock_result = MagicMock()
            mock_result.boxes = mock_box

            mock_model = MagicMock()
            mock_model.return_value = [mock_result]
            mock_model.names = {0: "person"}

            mock_yolo_instance = MagicMock()
            mock_yolo_instance.side_effect = lambda *args, **kwargs: [mock_result]
            mock_yolo_class.return_value = mock_yolo_instance

            detector = YOLODetector(model_size="n", confidence_threshold=0.25)
            detector.model = mock_yolo_instance
            detector.model.names = {0: "person"}
            detector.is_loaded = True

            # Image mock
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            detections = detector.detect_objects(image)

            assert len(detections) > 0
            assert "bbox" in detections[0]
            assert "confidence" in detections[0]
            assert "class_name" in detections[0]

    @patch("bbia_sim.vision_yolo.YOLO")
    def test_detect_objects_with_auto_load(self, mock_yolo_class):
        """Test détection avec chargement automatique."""
        with patch("bbia_sim.vision_yolo.YOLO_AVAILABLE", True):
            mock_model = MagicMock()
            mock_model.return_value = []
            mock_yolo_class.return_value = mock_model

            detector = YOLODetector(model_size="n", confidence_threshold=0.25)
            assert detector.is_loaded is False

            image = np.zeros((480, 640, 3), dtype=np.uint8)
            detections = detector.detect_objects(image)

            assert isinstance(detections, list)
            assert detector.is_loaded is True

    def test_get_best_detection_with_relevant(self):
        """Test get_best_detection avec détections pertinentes."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        detections = [
            {"class_name": "person", "confidence": 0.9, "area": 50000},
            {"class_name": "book", "confidence": 0.7, "area": 30000},
            {"class_name": "unknown", "confidence": 0.8, "area": 40000},
        ]

        best = detector.get_best_detection(detections)
        assert best is not None
        assert best["class_name"] in ["person", "book"]

    def test_get_best_detection_empty(self):
        """Test get_best_detection avec liste vide."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        result = detector.get_best_detection([])
        assert result is None

    def test_get_best_detection_no_relevant(self):
        """Test get_best_detection sans détections pertinentes."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        detections = [
            {"class_name": "unknown", "confidence": 0.8, "area": 40000},
        ]
        result = detector.get_best_detection(detections)
        assert result is None

    def test_map_detection_to_action_valid(self):
        """Test mapping détection vers action valide."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        detection = {
            "class_name": "person",
            "confidence": 0.9,
            "center": [200, 300],
            "bbox": [10, 20, 100, 120],
        }

        action = detector.map_detection_to_action(detection)
        assert action is not None
        assert action["action"] == "look_at"
        assert action["confidence"] == 0.9
        assert "direction" in action

    def test_map_detection_to_action_invalid(self):
        """Test mapping détection invalide."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        result = detector.map_detection_to_action(None)
        assert result is None

        detection = {"class_name": "unknown", "confidence": 0.8}
        result = detector.map_detection_to_action(detection)
        assert result is None

    def test_map_detection_to_action_direction_left(self):
        """Test direction gauche."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        detection = {
            "class_name": "person",
            "confidence": 0.9,
            "center": [200, 300],  # < 320 → gauche
            "bbox": [10, 20, 100, 120],
        }
        action = detector.map_detection_to_action(detection)
        assert action is not None
        assert action["direction"] == "left"

    def test_map_detection_to_action_direction_right(self):
        """Test direction droite."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        detection = {
            "class_name": "person",
            "confidence": 0.9,
            "center": [400, 300],  # >= 320 → droite
            "bbox": [10, 20, 100, 120],
        }
        action = detector.map_detection_to_action(detection)
        assert action is not None
        assert action["direction"] == "right"


@pytest.mark.unit
@pytest.mark.fast
class TestFaceDetector:
    """Tests pour FaceDetector."""

    @patch("bbia_sim.vision_yolo.mp")
    def test_init_with_mediapipe(self, mock_mp):
        """Test initialisation avec MediaPipe."""
        mock_mp.solutions.face_detection.FaceDetection = MagicMock(
            return_value=MagicMock()
        )

        detector = FaceDetector()
        assert detector.face_detection is not None

    @patch("bbia_sim.vision_yolo.mp", None)
    def test_init_without_mediapipe(self):
        """Test initialisation sans MediaPipe."""
        with patch.dict("sys.modules", {"mediapipe": None}):
            # Mock ImportError
            detector = FaceDetector()
            assert detector.face_detection is None

    @patch("bbia_sim.vision_yolo.cv2")
    def test_detect_faces_success(self, mock_cv2):
        """Test détection visages réussie."""
        mock_cv2.cvtColor = lambda img, code: img  # Mock conversion

        mock_detection = MagicMock()
        mock_detection.location_data.relative_bounding_box.xmin = 0.1
        mock_detection.location_data.relative_bounding_box.ymin = 0.2
        mock_detection.location_data.relative_bounding_box.width = 0.3
        mock_detection.location_data.relative_bounding_box.height = 0.4
        mock_detection.score = [0.9]

        mock_results = MagicMock()
        mock_results.detections = [mock_detection]

        mock_face_detection = MagicMock()
        mock_face_detection.process.return_value = mock_results

        detector = FaceDetector()
        detector.face_detection = mock_face_detection

        image = np.zeros((480, 640, 3), dtype=np.uint8)
        faces = detector.detect_faces(image)

        assert len(faces) == 1
        assert "bbox" in faces[0]
        assert "confidence" in faces[0]

    def test_detect_faces_no_detector(self):
        """Test détection sans détecteur."""
        detector = FaceDetector()
        detector.face_detection = None

        image = np.zeros((480, 640, 3), dtype=np.uint8)
        faces = detector.detect_faces(image)
        assert faces == []

    def test_get_best_face(self):
        """Test get_best_face."""
        detector = FaceDetector()
        faces = [
            {"confidence": 0.8, "area": 40000},
            {"confidence": 0.9, "area": 50000},
            {"confidence": 0.7, "area": 60000},
        ]

        best = detector.get_best_face(faces)
        assert best is not None
        # Devrait être celui avec meilleur score (confidence * (1 + area/50000))
        assert best["confidence"] >= 0.7

    def test_get_best_face_empty(self):
        """Test get_best_face avec liste vide."""
        detector = FaceDetector()
        result = detector.get_best_face([])
        assert result is None


@pytest.mark.unit
@pytest.mark.fast
class TestFactoryFunctions:
    """Tests pour fonctions factory."""

    @patch("bbia_sim.vision_yolo.YOLO_AVAILABLE", True)
    def test_create_yolo_detector_with_yolo(self):
        """Test création détecteur YOLO."""
        detector = create_yolo_detector(model_size="n", confidence_threshold=0.3)
        assert detector is not None
        assert isinstance(detector, YOLODetector)
        assert detector.model_size == "n"
        assert detector.confidence_threshold == 0.3

    @patch("bbia_sim.vision_yolo.YOLO_AVAILABLE", False)
    def test_create_yolo_detector_without_yolo(self):
        """Test création sans YOLO."""
        detector = create_yolo_detector(model_size="n", confidence_threshold=0.3)
        assert detector is None

    @patch("bbia_sim.vision_yolo.mp")
    def test_create_face_detector_with_mediapipe(self, mock_mp):
        """Test création détecteur visages avec MediaPipe."""
        mock_mp.solutions.face_detection.FaceDetection = MagicMock(
            return_value=MagicMock()
        )
        detector = create_face_detector()
        assert detector is not None
        assert isinstance(detector, FaceDetector)

    def test_create_face_detector_without_mediapipe(self):
        """Test création sans MediaPipe."""
        with patch.dict("sys.modules", {"mediapipe": None}):
            # Mock ImportError
            import sys

            if "mediapipe" in sys.modules:
                del sys.modules["mediapipe"]

            detector = create_face_detector()
            assert detector is None
