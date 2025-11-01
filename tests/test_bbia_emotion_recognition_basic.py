#!/usr/bin/env python3
"""
Tests basiques pour bbia_emotion_recognition.py - Amélioration coverage 15.5% → 50%+
"""

import pytest

from bbia_sim.bbia_emotion_recognition import ML_AVAILABLE, BBIAEmotionRecognition


@pytest.mark.unit
@pytest.mark.fast
class TestBBIAEmotionRecognitionBasic:
    """Tests basiques pour BBIAEmotionRecognition (sans dépendances ML)."""

    def test_ml_available_check(self):
        """Test vérification disponibilité ML."""
        # Test que ML_AVAILABLE est booléen
        assert isinstance(ML_AVAILABLE, bool)

    @pytest.mark.skipif(not ML_AVAILABLE, reason="ML dependencies not available")
    def test_init_with_ml_available(self):
        """Test initialisation avec ML disponible."""
        recognition = BBIAEmotionRecognition(device="cpu")
        assert recognition.device == "cpu"
        assert recognition.is_initialized is False
        assert len(recognition.supported_emotions) > 0
        assert "happy" in recognition.supported_emotions

    @pytest.mark.skipif(ML_AVAILABLE, reason="ML dependencies available")
    def test_init_without_ml_raises_error(self):
        """Test initialisation sans ML lève ImportError."""
        with pytest.raises(ImportError):
            BBIAEmotionRecognition(device="cpu")

    @pytest.mark.skipif(not ML_AVAILABLE, reason="ML dependencies not available")
    def test_supported_emotions(self):
        """Test émotions supportées."""
        recognition = BBIAEmotionRecognition(device="cpu")
        assert "happy" in recognition.supported_emotions
        assert "sad" in recognition.supported_emotions
        assert "neutral" in recognition.supported_emotions

    @pytest.mark.skipif(not ML_AVAILABLE, reason="ML dependencies not available")
    def test_detection_config(self):
        """Test configuration détection."""
        recognition = BBIAEmotionRecognition(device="cpu")
        assert "face_detection_confidence" in recognition.detection_config
        assert "emotion_confidence_threshold" in recognition.detection_config
        assert recognition.detection_config["face_detection_confidence"] == 0.7
