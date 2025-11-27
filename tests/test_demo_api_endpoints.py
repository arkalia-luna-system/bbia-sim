#!/usr/bin/env python3
"""Tests pour les nouveaux exemples d'endpoints API.

Tests unitaires pour vérifier que les exemples API fonctionnent correctement.
"""

import sys
from pathlib import Path
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestDemoAPIEndpoints:
    """Tests pour les exemples d'endpoints API."""

    @patch("examples.demo_motors.httpx")
    def test_demo_motors_import(self, mock_httpx):
        """Test import demo_motors."""
        import examples.demo_motors as demo_motors

        assert hasattr(demo_motors, "main")

    @patch("examples.demo_daemon.httpx")
    def test_demo_daemon_import(self, mock_httpx):
        """Test import demo_daemon."""
        import examples.demo_daemon as demo_daemon

        assert hasattr(demo_daemon, "main")

    @patch("examples.demo_kinematics.httpx")
    def test_demo_kinematics_import(self, mock_httpx):
        """Test import demo_kinematics."""
        import examples.demo_kinematics as demo_kinematics

        assert hasattr(demo_kinematics, "main")

    @patch("examples.demo_media.httpx")
    def test_demo_media_import(self, mock_httpx):
        """Test import demo_media."""
        import examples.demo_media as demo_media

        assert hasattr(demo_media, "main")

    @patch("examples.demo_apps.httpx")
    def test_demo_apps_import(self, mock_httpx):
        """Test import demo_apps."""
        import examples.demo_apps as demo_apps

        assert hasattr(demo_apps, "main")

    @patch("examples.demo_metrics.httpx")
    def test_demo_metrics_import(self, mock_httpx):
        """Test import demo_metrics."""
        import examples.demo_metrics as demo_metrics

        assert hasattr(demo_metrics, "main")

    @patch("examples.demo_state_ws.websockets")
    def test_demo_state_ws_import(self, mock_websockets):
        """Test import demo_state_ws."""
        import examples.demo_state_ws as demo_state_ws

        assert hasattr(demo_state_ws, "main")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
