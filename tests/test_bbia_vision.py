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
    vision = BBIAVision(robot_api=None)
    vision.scan_environment()

    assert vision.track_object("livre") is True
    status = vision.get_focus_status()
    assert status["tracking_active"] is True
    assert status["current_focus"] is not None

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
        assert specs["resolution"] == "1080p"
        assert specs["fov"] == "120°"
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
