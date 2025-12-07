#!/usr/bin/env python3
"""
Tests de gestion d'erreurs pour pose_detection.

Vérifie que les erreurs sont gérées correctement avec les nouveaux logs ERROR.
"""

import logging
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from bbia_sim.pose_detection import BBIAPoseDetection

logger = logging.getLogger(__name__)


class TestPoseDetectionErrorHandling:
    """Tests de gestion d'erreurs pour BBIAPoseDetection."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_pose_detection_init_error_handling(self):
        """Test que l'initialisation gère les erreurs correctement."""
        # Simuler une erreur lors de l'initialisation MediaPipe
        with patch("bbia_sim.pose_detection.mp", None):
            detector = BBIAPoseDetection()
            # Doit initialiser sans crasher même si MediaPipe non disponible
            assert detector is not None
            assert detector.is_initialized is False

    @pytest.mark.unit
    @pytest.mark.fast
    def test_pose_detection_init_with_exception(self):
        """Test que l'initialisation gère les exceptions correctement."""
        # Simuler une exception lors de l'initialisation en patchant directement
        # le module mediapipe importé
        # Créer un mock mediapipe qui lève une exception
        mock_mp = MagicMock()
        mock_mp.solutions.pose.Pose.side_effect = RuntimeError("Erreur initialisation")

        # Patcher directement bbia_sim.pose_detection.mp car mp est déjà importé
        with patch("bbia_sim.pose_detection.mp", mock_mp):
        with patch("bbia_sim.pose_detection.MEDIAPIPE_POSE_AVAILABLE", True):
                detector = BBIAPoseDetection()
                # Doit initialiser sans crasher
                assert detector is not None
                # is_initialized devrait être False en cas d'erreur
                assert detector.is_initialized is False

    @pytest.mark.unit
    @pytest.mark.fast
    def test_pose_detection_detect_error_handling(self):
        """Test que detect_pose() gère les erreurs correctement."""
        detector = BBIAPoseDetection()
        # Image plus petite pour accélérer le test (240x320 au lieu de 480x640)
        image = np.zeros((240, 320, 3), dtype=np.uint8)

        # Si MediaPipe non disponible, doit retourner None sans crasher
        result = detector.detect_pose(image)
        assert result is None or isinstance(result, dict)

    @pytest.mark.unit
    @pytest.mark.fast
    def test_pose_detection_detect_with_exception(self):
        """Test que detect_pose() gère les exceptions correctement."""
        detector = BBIAPoseDetection()
        # Simuler une exception lors de la détection
        if detector.pose_detector is not None:
            with patch.object(
                detector.pose_detector, "process", side_effect=RuntimeError("Erreur")
            ):
                # Image plus petite pour accélérer le test
                image = np.zeros((240, 320, 3), dtype=np.uint8)
                result = detector.detect_pose(image)
                # Doit retourner None sans crasher
                assert result is None

    @pytest.mark.unit
    @pytest.mark.fast
    def test_pose_detection_logs_error_level(self):
        """Test que les erreurs critiques sont loggées en ERROR."""
        mock_logger = MagicMock(spec=logging.Logger)
        # Créer un mock mediapipe qui lève une exception
        mock_mp = MagicMock()
        mock_mp.solutions.pose.Pose.side_effect = RuntimeError("Erreur critique")

        # Patcher directement bbia_sim.pose_detection.mp car mp est déjà importé
        with patch("bbia_sim.pose_detection.mp", mock_mp):
        with patch("bbia_sim.pose_detection.logger", mock_logger):
            with patch("bbia_sim.pose_detection.MEDIAPIPE_POSE_AVAILABLE", True):
                    BBIAPoseDetection()
                    # Vérifier que logger.error a été appelé
                    mock_logger.error.assert_called()
                    assert "critique" in str(mock_logger.error.call_args).lower()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
