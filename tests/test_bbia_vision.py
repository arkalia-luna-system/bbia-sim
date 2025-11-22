#!/usr/bin/env python3
"""Tests unitaires Vision (pipeline simulé + skips propres)."""

import os

import pytest

from bbia_sim.bbia_vision import BBIAVision


@pytest.mark.unit
@pytest.mark.fast
def test_scan_environment_simulation() -> None:
    if os.environ.get("BBIA_DISABLE_VISION", "0") == "1":
        pytest.skip("Vision désactivée par BBIA_DISABLE_VISION=1")

    vision = BBIAVision(robot_api=None)
    result = vision.scan_environment()

    assert "objects" in result and isinstance(result["objects"], list)
    assert "faces" in result and isinstance(result["faces"], list)
    assert result.get("source") in {"simulation", "camera_sdk"}


@pytest.mark.unit
@pytest.mark.fast
def test_track_untrack_object() -> None:
    """Test tracking/untracking objet avec mock objets détectés."""
    vision = BBIAVision(robot_api=None)
    vision.scan_environment()

    # Simuler un objet détecté pour le test (car recognize_object cherche dans objects_detected)
    # En mode simulation, scan_environment peut ne pas détecter d'objets réels
    from collections import deque

    vision.objects_detected = deque(
        [
            {"name": "livre", "confidence": 0.8, "bbox": [100, 100, 200, 200]},
        ]
    )

    # Maintenant track_object devrait fonctionner
    result = vision.track_object("livre")
    assert result is True, "track_object devrait réussir avec objet mocké"
    status = vision.get_focus_status()
    assert status["tracking_active"] is True
    assert status["current_focus"] is not None
    assert status["current_focus"]["name"] == "livre"

    vision.stop_tracking()
    status2 = vision.get_focus_status()
    assert status2["tracking_active"] is False
    assert status2["current_focus"] is None


#!/usr/bin/env python3
"""Tests pour le module BBIA Vision."""

import gc


class TestBBIAVision:
    """Tests pour BBIAVision."""

    def teardown_method(self):
        """OPTIMISATION RAM: Décharger modèles vision après chaque test."""
        try:
            # Vider cache YOLO
            import bbia_sim.vision_yolo as vision_yolo_module
            with vision_yolo_module._yolo_cache_lock:
                vision_yolo_module._yolo_model_cache.clear()
        except (AttributeError, ImportError):
            pass
        gc.collect()

    @pytest.mark.fast  # OPTIMISATION RAM: Test rapide avec robot_api=None (simulation)
    def test_vision_creation(self):
        """Test création d'une instance BBIAVision."""
        vision = BBIAVision(
            robot_api=None
        )  # OPTIMISATION RAM: Pas de chargement caméra
        assert vision.camera_active
        assert vision.vision_quality == "HD"
        assert vision.detection_range == 3.0
        assert not vision.tracking_active

    @pytest.mark.fast
    def test_vision_specs(self):
        """Test des spécifications hardware."""
        vision = BBIAVision(
            robot_api=None
        )  # OPTIMISATION RAM: Pas de chargement caméra

        specs = vision.specs
        assert specs is not None
        assert specs["camera"] == "Grand angle"
        # Resolution a été clarifiée avec simulation/réel
        resolution = str(specs["resolution"]) if specs["resolution"] is not None else ""
        assert "1280x720" in resolution or "HD" in resolution
        # FOV a été clarifié avec simulation/réel
        fov = str(specs["fov"]) if specs["fov"] is not None else ""
        assert "80°" in fov or "120°" in fov
        assert specs["focus"] == "Auto"

    @pytest.mark.fast
    def test_detection_methods(self):
        """Test des méthodes de détection."""
        vision = BBIAVision(
            robot_api=None
        )  # OPTIMISATION RAM: Pas de chargement caméra

        # Test que les listes de détection existent (maintenant deque pour optimisation RAM)
        from collections import deque

        assert isinstance(vision.objects_detected, deque)
        assert isinstance(vision.faces_detected, deque)

    @pytest.mark.fast
    def test_tracking_control(self):
        """Test contrôle du suivi."""
        vision = BBIAVision(
            robot_api=None
        )  # OPTIMISATION RAM: Pas de chargement caméra

        # Test état initial
        assert not vision.tracking_active

        # Test changement d'état
        vision.tracking_active = True
        assert vision.tracking_active

    @pytest.mark.fast
    def test_focus_control(self):
        """Test contrôle du focus."""
        vision = BBIAVision(
            robot_api=None
        )  # OPTIMISATION RAM: Pas de chargement caméra

        # Test état initial
        assert vision.current_focus is None

        # Test changement de focus (current_focus est un dict, pas un str)
        test_object = {"name": "test_object", "position": (0.5, 0.5)}
        vision.current_focus = test_object
        assert vision.current_focus == test_object
