#!/usr/bin/env python3
"""Tests pour sanity router - Routes FastAPI de vérification système."""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# S'assurer que src est dans le path pour coverage
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# Importer le module directement - coverage doit le détecter
# IMPORTANT: Import direct (pas dans try/except) pour que coverage le détecte
import bbia_sim.daemon.app.routers.sanity  # noqa: F401
from bbia_sim.daemon.app.routers.sanity import (
    _reachy_alive,
    router,
    sanity_emergency_stop,
    sanity_status,
)


class TestSanityRouter:
    """Tests pour le router sanity."""

    @patch("bbia_sim.daemon.app.routers.sanity.ReachyMiniBackend")
    @patch("bbia_sim.daemon.app.routers.sanity.simulation_service")
    @pytest.mark.asyncio
    async def test_sanity_status_success(self, mock_sim_service, mock_backend_class):
        """Test route GET /api/sanity/status avec succès."""
        # Mock simulation service
        mock_sim_service.is_simulation_ready.return_value = True

        # Mock backend
        mock_backend = MagicMock()
        mock_backend.connect.return_value = True
        mock_backend.get_available_joints.return_value = ["joint1", "joint2"]
        mock_backend_class.return_value = mock_backend

        result = await sanity_status()

        assert "timestamp" in result
        assert result["simulation_ready"] is True
        assert "reachy" in result
        assert result["reachy"]["ok"] is True
        assert "status" in result["reachy"]
        assert result["warnings"] == []
        mock_backend.connect.assert_called_once()
        mock_backend.disconnect.assert_called_once()

    @patch("bbia_sim.daemon.app.routers.sanity.ReachyMiniBackend")
    @patch("bbia_sim.daemon.app.routers.sanity.simulation_service")
    @pytest.mark.asyncio
    async def test_sanity_status_backend_error(
        self, mock_sim_service, mock_backend_class
    ):
        """Test route GET /api/sanity/status avec erreur backend."""
        mock_sim_service.is_simulation_ready.return_value = False

        mock_backend = MagicMock()
        mock_backend.connect.side_effect = Exception("Connection failed")
        mock_backend_class.return_value = mock_backend

        result = await sanity_status()

        assert result["simulation_ready"] is False
        assert result["reachy"]["ok"] is False
        assert "error" in result["reachy"]
        mock_backend.disconnect.assert_called_once()

    @patch("bbia_sim.daemon.app.routers.sanity.ReachyMiniBackend")
    @pytest.mark.asyncio
    async def test_sanity_emergency_stop_success(self, mock_backend_class):
        """Test route POST /api/sanity/emergency_stop avec succès."""
        mock_backend = MagicMock()
        mock_backend.connect.return_value = True
        mock_backend.emergency_stop.return_value = True
        mock_backend_class.return_value = mock_backend

        result = await sanity_emergency_stop()

        assert result["ok"] is True
        assert "ts" in result
        mock_backend.connect.assert_called_once()
        mock_backend.emergency_stop.assert_called_once()
        mock_backend.disconnect.assert_called_once()

    @patch("bbia_sim.daemon.app.routers.sanity.ReachyMiniBackend")
    @pytest.mark.asyncio
    async def test_sanity_emergency_stop_error(self, mock_backend_class):
        """Test route POST /api/sanity/emergency_stop avec erreur."""
        mock_backend = MagicMock()
        mock_backend.connect.side_effect = Exception("Connection failed")
        mock_backend_class.return_value = mock_backend

        result = await sanity_emergency_stop()

        assert result["ok"] is False
        assert "error" in result
        mock_backend.disconnect.assert_called_once()

    @patch("bbia_sim.daemon.app.routers.sanity.ReachyMiniBackend")
    def test_reachy_alive_success(self, mock_backend_class):
        """Test fonction _reachy_alive avec succès."""
        mock_backend = MagicMock()
        mock_backend.connect.return_value = True
        mock_backend.get_available_joints.return_value = ["joint1", "joint2"]
        mock_backend_class.return_value = mock_backend

        result = _reachy_alive()

        assert result["ok"] is True
        assert "status" in result
        assert result["status"]["connected"] is True
        assert "available_joints" in result["status"]
        mock_backend.connect.assert_called_once()
        mock_backend.disconnect.assert_called_once()

    @patch("bbia_sim.daemon.app.routers.sanity.ReachyMiniBackend")
    def test_reachy_alive_error(self, mock_backend_class):
        """Test fonction _reachy_alive avec erreur."""
        mock_backend = MagicMock()
        mock_backend.connect.side_effect = Exception("Connection failed")
        mock_backend_class.return_value = mock_backend

        result = _reachy_alive()

        assert result["ok"] is False
        assert "error" in result
        mock_backend.disconnect.assert_called_once()

    def test_router_configured(self):
        """Test que le router est correctement configuré."""
        assert router.prefix == "/api/sanity"
        assert "sanity" in router.tags
