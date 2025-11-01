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
    vision.objects_detected = [
        {"name": "livre", "confidence": 0.8, "bbox": [100, 100, 200, 200]},
    ]

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


class TestBBIAVision:
    """Tests pour BBIAVision."""

    def test_vision_creation(self):
        """Test création d'une instance BBIAVision."""
        vision = BBIAVision()
        assert vision.camera_active
        assert vision.vision_quality == "HD"
        assert vision.detection_range == 3.0
        assert not vision.tracking_active

    def test_vision_specs(self):
        """Test des spécifications hardware."""
        vision = BBIAVision()

        specs = vision.specs
        assert specs["camera"] == "Grand angle"
        # Resolution a été clarifiée avec simulation/réel
        assert "1280x720" in specs["resolution"] or "HD" in specs["resolution"]
        # FOV a été clarifié avec simulation/réel
        assert "80°" in specs["fov"] or "120°" in specs["fov"]
        assert specs["focus"] == "Auto"

    def test_detection_methods(self):
        """Test des méthodes de détection."""
        vision = BBIAVision()

        # Test que les listes de détection existent
        assert isinstance(vision.objects_detected, list)
        assert isinstance(vision.faces_detected, list)

    def test_tracking_control(self):
        """Test contrôle du suivi."""
        vision = BBIAVision()

        # Test état initial
        assert not vision.tracking_active

        # Test changement d'état
        vision.tracking_active = True
        assert vision.tracking_active

    def test_focus_control(self):
        """Test contrôle du focus."""
        vision = BBIAVision()

        # Test état initial
        assert vision.current_focus is None

        # Test changement de focus
        vision.current_focus = "test_object"
        assert vision.current_focus == "test_object"
