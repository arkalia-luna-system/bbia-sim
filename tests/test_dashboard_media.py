#!/usr/bin/env python3
"""Tests pour les endpoints API media du dashboard.

Tests des endpoints pour contrôler le volume des haut-parleurs,
microphone, et activer/désactiver la caméra.
"""

import pytest
from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app

client = TestClient(app)


class TestDashboardMediaEndpoints:
    """Tests pour les endpoints media."""

    def test_set_speaker_volume_valid(self):
        """Test définition volume haut-parleur valide."""
        response = client.post(
            "/development/api/media/speaker/volume",
            json={"volume": 0.7},
        )
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "success"
        assert data["volume"] == 0.7

    def test_set_speaker_volume_invalid_high(self):
        """Test volume invalide (trop élevé)."""
        response = client.post(
            "/development/api/media/speaker/volume",
            json={"volume": 1.5},  # > 1.0
        )
        assert response.status_code == 422  # Validation error

    def test_set_speaker_volume_invalid_low(self):
        """Test volume invalide (trop bas)."""
        response = client.post(
            "/development/api/media/speaker/volume",
            json={"volume": -0.1},  # < 0.0
        )
        assert response.status_code == 422  # Validation error

    def test_set_microphone_volume_valid(self):
        """Test définition volume microphone valide."""
        response = client.post(
            "/development/api/media/microphone/volume",
            json={"volume": 0.8},
        )
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "success"
        assert data["volume"] == 0.8

    def test_toggle_camera_enable(self):
        """Test activation caméra."""
        response = client.post(
            "/development/api/media/camera/toggle",
            json={"enabled": True},
        )
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "success"
        assert data["enabled"] is True

    def test_toggle_camera_disable(self):
        """Test désactivation caméra."""
        response = client.post(
            "/development/api/media/camera/toggle",
            json={"enabled": False},
        )
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "success"
        assert data["enabled"] is False

    def test_get_media_status(self):
        """Test récupération statut media."""
        response = client.get("/development/api/media/status")
        assert response.status_code == 200
        data = response.json()
        assert "speaker_volume" in data
        assert "microphone_volume" in data
        assert "camera_enabled" in data
        assert "speaker_active" in data
        assert "microphone_active" in data
        assert 0.0 <= data["speaker_volume"] <= 1.0
        assert 0.0 <= data["microphone_volume"] <= 1.0
        assert isinstance(data["camera_enabled"], bool)

    def test_volume_clamping(self):
        """Test que volumes sont clampés entre 0.0 et 1.0."""
        # Tester volume 0.0
        response = client.post(
            "/development/api/media/speaker/volume",
            json={"volume": 0.0},
        )
        assert response.status_code == 200

        # Tester volume 1.0
        response = client.post(
            "/development/api/media/speaker/volume",
            json={"volume": 1.0},
        )
        assert response.status_code == 200
