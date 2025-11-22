#!/usr/bin/env python3
"""Tests pour le batch processing YOLO."""

import gc

import numpy as np
import pytest

try:
    from bbia_sim.vision_yolo import YOLO_AVAILABLE, YOLODetector
except ImportError:
    YOLO_AVAILABLE = False
    YOLODetector = None


@pytest.mark.skipif(not YOLO_AVAILABLE, reason="YOLO non disponible")
class TestYOLOBatchProcessing:
    """Tests pour le batch processing YOLO."""

    def teardown_method(self):
        """OPTIMISATION RAM: Nettoyer mémoire après chaque test."""
        gc.collect()

    def test_detect_objects_batch_empty_list(self):
        """Test batch processing avec liste vide."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        result = detector.detect_objects_batch([])
        assert result == []

    def test_detect_objects_batch_single_image(self):
        """Test batch processing avec une seule image."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        if not detector.load_model():
            pytest.skip("YOLO modèle non chargé")

        # OPTIMISATION RAM: Image réduite (320x240 au lieu de 640x480 = 4x moins de mémoire)
        test_image = np.zeros((240, 320, 3), dtype=np.uint8)
        result = detector.detect_objects_batch([test_image])

        assert len(result) == 1
        assert isinstance(result[0], list)

    def test_detect_objects_batch_multiple_images(self):
        """Test batch processing avec plusieurs images."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        if not detector.load_model():
            pytest.skip("YOLO modèle non chargé")

        # OPTIMISATION RAM: Images réduites (320x240) et seulement 2 images au lieu de 3
        test_images = [np.zeros((240, 320, 3), dtype=np.uint8) for _ in range(2)]
        result = detector.detect_objects_batch(test_images)

        assert len(result) == 2  # OPTIMISATION RAM: 2 images au lieu de 3
        for detections in result:
            assert isinstance(detections, list)

    def test_detect_objects_batch_returns_correct_format(self):
        """Test que le format de retour est correct."""
        detector = YOLODetector(model_size="n", confidence_threshold=0.25)
        if not detector.load_model():
            pytest.skip("YOLO modèle non chargé")

        # OPTIMISATION RAM: Image réduite (320x240 au lieu de 640x480)
        test_image = np.zeros((240, 320, 3), dtype=np.uint8)
        result = detector.detect_objects_batch([test_image])

        # Vérifier que chaque détection a les champs attendus
        for detections in result:
            for detection in detections:
                assert "bbox" in detection
                assert "confidence" in detection
                assert "class_id" in detection
                assert "class_name" in detection
                assert "center" in detection
                assert "area" in detection
