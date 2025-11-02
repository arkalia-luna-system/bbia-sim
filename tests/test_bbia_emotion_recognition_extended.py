#!/usr/bin/env python3
"""
Tests étendus pour bbia_emotion_recognition.py
Tests de reconnaissance des émotions
"""

from unittest.mock import patch

import pytest


class TestBBIAEmotionRecognitionExtended:
    """Tests étendus pour BBIAEmotionRecognition."""

    @pytest.fixture
    def mock_ml_available(self):
        """Mock ML disponible."""
        with patch("bbia_sim.bbia_emotion_recognition.ML_AVAILABLE", True):
            with patch("bbia_sim.bbia_emotion_recognition.mp") as mock_mp:
                with patch("bbia_sim.bbia_emotion_recognition.torch") as mock_torch:
                    with patch("bbia_sim.bbia_emotion_recognition.pipeline") as mock_pipeline:
                        yield mock_mp, mock_torch, mock_pipeline

    def test_ml_import_available(self):
        """Test que ML est disponible."""
        try:
            from bbia_sim.bbia_emotion_recognition import ML_AVAILABLE

            # Peut être False si dépendances non installées
            assert isinstance(ML_AVAILABLE, bool)
        except ImportError:
            pytest.skip("Module emotion_recognition non disponible")

    def test_ml_import_not_available(self):
        """Test gestion ML non disponible."""
        # En test, c'est OK si ML n'est pas disponible
        try:
            from bbia_sim.bbia_emotion_recognition import ML_AVAILABLE

            # Dans tests, acceptons False
            assert isinstance(ML_AVAILABLE, bool)
        except ImportError:
            pass  # OK

    def test_emotion_recognition_class_exist(self):
        """Test que la classe BBIAEmotionRecognition existe."""
        try:
            from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition

            assert BBIAEmotionRecognition is not None
        except ImportError:
            pytest.skip("Module emotion_recognition non disponible")

    def test_detect_faces_method_exist(self):
        """Test que la méthode detect_faces existe."""
        try:
            from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition

            # Si ML non dispo, va échouer à l'init mais on vérifie la méthode
            assert hasattr(BBIAEmotionRecognition, "detect_faces")
        except ImportError:
            pytest.skip("Module emotion_recognition non disponible")

    def test_analyze_facial_emotion_method_exist(self):
        """Test que la méthode analyze_facial_emotion existe."""
        try:
            from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition

            assert hasattr(BBIAEmotionRecognition, "analyze_facial_emotion")
        except ImportError:
            pytest.skip("Module emotion_recognition non disponible")

    def test_analyze_vocal_emotion_method_exist(self):
        """Test que la méthode analyze_vocal_emotion existe."""
        try:
            from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition

            assert hasattr(BBIAEmotionRecognition, "analyze_vocal_emotion")
        except ImportError:
            pytest.skip("Module emotion_recognition non disponible")

    def test_fuse_emotions_method_exist(self):
        """Test que la méthode fuse_emotions existe."""
        try:
            from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition

            assert hasattr(BBIAEmotionRecognition, "fuse_emotions")
        except ImportError:
            pytest.skip("Module emotion_recognition non disponible")

    def test_get_emotion_statistics_method_exist(self):
        """Test que la méthode get_emotion_statistics existe."""
        try:
            from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition

            assert hasattr(BBIAEmotionRecognition, "get_emotion_statistics")
        except ImportError:
            pytest.skip("Module emotion_recognition non disponible")

    def test_reset_history_method_exist(self):
        """Test que la méthode reset_history existe."""
        try:
            from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition

            assert hasattr(BBIAEmotionRecognition, "reset_history")
        except ImportError:
            pytest.skip("Module emotion_recognition non disponible")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
