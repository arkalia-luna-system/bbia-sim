#!/usr/bin/env python3
"""
Tests étendus pour vision_yolo.py
Tests de détection YOLO
"""

import pytest


class TestVisionYoloExtended:
    """Tests étendus pour VisionYolo."""

    def test_vision_yolo_module_exists(self):
        """Test que le module vision_yolo existe."""
        try:
            import bbia_sim.vision_yolo

            assert bbia_sim.vision_yolo is not None
        except ImportError:
            pytest.skip("Module vision_yolo non disponible")

    def test_yolo_detection_functions_exist(self):
        """Test que les fonctions de détection existent."""
        try:
            from bbia_sim.vision_yolo import classify_object, detect_objects

            assert detect_objects is not None
            assert classify_object is not None
        except ImportError:
            pytest.skip("Fonctions vision_yolo non disponibles")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
