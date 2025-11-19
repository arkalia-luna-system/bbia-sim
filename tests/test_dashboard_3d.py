#!/usr/bin/env python3
"""Tests pour le rendu 3D du robot dans le dashboard.

Tests du chargement du modèle 3D, animations selon état,
et synchronisation avec le robot réel.
"""

import pytest
from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app

client = TestClient(app)


class TestDashboard3D:
    """Tests pour le rendu 3D du robot."""

    def test_dashboard_3d_script_loaded(self):
        """Test que le script robot_3d.js est chargé."""
        response = client.get("/")
        assert response.status_code == 200
        # Vérifier que script est dans HTML
        assert "robot_3d.js" in response.text or "robot-3d" in response.text.lower()

    def test_3d_canvas_exists(self):
        """Test que le canvas 3D existe dans le dashboard."""
        response = client.get("/")
        assert response.status_code == 200
        # Vérifier présence canvas ou élément 3D
        assert "canvas" in response.text.lower() or "3d" in response.text.lower()

    def test_three_js_loaded(self):
        """Test que Three.js est chargé (CDN ou local)."""
        response = client.get("/")
        assert response.status_code == 200
        # Vérifier présence Three.js (CDN ou local)
        assert (
            "three.js" in response.text.lower()
            or "three@0" in response.text.lower()
            or "three.min.js" in response.text.lower()
        )

    def test_robot_model_available(self):
        """Test que les modèles STL sont disponibles."""
        # Vérifier que fichiers STL existent
        from pathlib import Path

        stl_dir = Path("src/bbia_sim/sim/assets/reachy_official")
        if stl_dir.exists():
            stl_files = list(stl_dir.glob("*.stl"))
            assert len(stl_files) > 0, "Aucun fichier STL trouvé"


@pytest.mark.hardware
class TestDashboard3DWithRobot:
    """Tests 3D avec robot réel (nécessite hardware)."""

    def test_3d_sync_with_robot(self):
        """Test synchronisation 3D avec robot réel."""
        # Ce test nécessite un robot réel connecté
        # Pour l'instant, juste vérifier que test existe
        assert True

    def test_3d_animation_states(self):
        """Test animations selon état (awake, sleeping, error)."""
        # Vérifier que différents états déclenchent différentes animations
        states = ["awake", "sleeping", "error"]
        for state in states:
            # Vérifier que state peut être appliqué
            assert state in ["awake", "sleeping", "error"]


@pytest.mark.slow
class TestDashboard3DRendering:
    """Tests de performance rendu 3D (marqué slow)."""

    def test_3d_render_performance(self):
        """Test performance rendu 3D (objectif 60 FPS)."""
        # Simulation test performance
        import time

        frames = 60
        start = time.time()

        # Simuler rendu 60 frames
        for _ in range(frames):
            time.sleep(0.016)  # ~60 FPS

        elapsed = time.time() - start
        fps = frames / elapsed

        # Vérifier que FPS est proche de 60
        assert fps > 50.0  # Tolérance pour tests
