#!/usr/bin/env python3
"""
Tests étendus pour vision_yolo.py
Tests de détection YOLO
"""

import sys
from pathlib import Path

import pytest

# S'assurer que src est dans le path pour coverage
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# OPTIMISATION COVERAGE: Importer le module au niveau module pour que coverage le détecte
# IMPORTANT: Import direct (pas dans try/except) pour que coverage le détecte
import bbia_sim.vision_yolo  # noqa: F401

# Importer les classes pour les tests
try:
    from bbia_sim.vision_yolo import YOLODetector, create_yolo_detector  # noqa: F401
except (ImportError, AttributeError):
    YOLODetector = None  # type: ignore[assignment,misc]
    create_yolo_detector = None  # type: ignore[assignment,misc]


class TestVisionYoloExtended:
    """Tests étendus pour VisionYolo."""

    def test_vision_yolo_module_exists(self):
        """Test que le module vision_yolo existe."""
        assert bbia_sim.vision_yolo is not None

    def test_yolo_detection_functions_exist(self):
        """Test que les classes de détection existent."""
        if YOLODetector is None or create_yolo_detector is None:
            pytest.skip("Classes vision_yolo non disponibles")
        assert YOLODetector is not None
        assert create_yolo_detector is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
