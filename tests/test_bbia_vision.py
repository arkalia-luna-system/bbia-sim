#!/usr/bin/env python3
"""
Tests pour le module BBIA Vision
"""

from src.bbia_sim.bbia_vision import BBIAVision


class TestBBIAVision:
    """Tests pour BBIAVision"""

    def test_vision_creation(self):
        """Test création d'une instance BBIAVision"""
        vision = BBIAVision()
        assert vision.camera_active
        assert vision.vision_quality == "HD"
        assert vision.detection_range == 3.0
        assert not vision.tracking_active

    def test_vision_specs(self):
        """Test des spécifications hardware"""
        vision = BBIAVision()

        specs = vision.specs
        assert specs["camera"] == "Grand angle"
        assert specs["resolution"] == "1080p"
        assert specs["fov"] == "120°"
        assert specs["focus"] == "Auto"

    def test_detection_methods(self):
        """Test des méthodes de détection"""
        vision = BBIAVision()

        # Test que les listes de détection existent
        assert isinstance(vision.objects_detected, list)
        assert isinstance(vision.faces_detected, list)

    def test_tracking_control(self):
        """Test contrôle du suivi"""
        vision = BBIAVision()

        # Test état initial
        assert not vision.tracking_active

        # Test changement d'état
        vision.tracking_active = True
        assert vision.tracking_active

    def test_focus_control(self):
        """Test contrôle du focus"""
        vision = BBIAVision()

        # Test état initial
        assert vision.current_focus is None

        # Test changement de focus
        vision.current_focus = "test_object"
        assert vision.current_focus == "test_object"
