#!/usr/bin/env python3
"""
Tests pour les fonctionnalités priorité haute ecosystem.py
- Tracking WebSocket actif
- Logique démarrage démo
"""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

# Import direct pour éviter problèmes de mock
# Les fonctions seront importées dans chaque test


class TestWebSocketTracking:
    """Tests pour le tracking WebSocket actif."""

    def test_get_ws_manager_import_success(self):
        """Test récupération gestionnaire WS avec import réussi."""
        from bbia_sim.daemon.app.routers.ecosystem import get_ws_manager

        mock_manager = MagicMock()
        mock_manager.active_connections = [MagicMock(), MagicMock()]
        with patch(
            "bbia_sim.daemon.app.routers.ecosystem.get_ws_manager",
            return_value=mock_manager,
        ):
            result = get_ws_manager()
            assert result is not None

    def test_get_active_connections_with_manager(self):
        """Test comptage connexions actives avec gestionnaire."""
        from bbia_sim.daemon.app.routers.ecosystem import get_active_connections

        mock_manager = MagicMock()
        mock_manager.active_connections = [MagicMock(), MagicMock(), MagicMock()]

        # Mock get_ws_manager qui est maintenant utilisé par get_active_connections
        with patch(
            "bbia_sim.daemon.app.routers.ecosystem.get_ws_manager",
            return_value=mock_manager,
        ):
            count = get_active_connections()
            assert count == 3

    def test_get_active_connections_no_manager(self):
        """Test comptage sans gestionnaire (retourne 0)."""
        from bbia_sim.daemon.app.routers.ecosystem import get_active_connections

        # Invalider le cache avant le test
        import bbia_sim.daemon.app.routers.ecosystem as ecosystem_module

        ecosystem_module._active_connections_cache = None
        ecosystem_module._active_connections_cache_time = 0.0

        with patch(
            "bbia_sim.daemon.app.routers.ecosystem.get_ws_manager", return_value=None
        ):
            count = get_active_connections()
            assert count == 0

    def test_get_active_connections_empty(self):
        """Test comptage avec liste vide."""
        from bbia_sim.daemon.app.routers.ecosystem import get_active_connections

        # Invalider le cache avant le test
        import bbia_sim.daemon.app.routers.ecosystem as ecosystem_module

        ecosystem_module._active_connections_cache = None
        ecosystem_module._active_connections_cache_time = 0.0

        mock_manager = MagicMock()
        mock_manager.active_connections = []

        with patch(
            "bbia_sim.daemon.app.routers.ecosystem.get_ws_manager",
            return_value=mock_manager,
        ):
            count = get_active_connections()
            assert count == 0


@pytest.mark.asyncio
class TestDemoLogic:
    """Tests pour la logique démarrage démo."""

    @patch("bbia_sim.daemon.app.routers.ecosystem.RobotFactory")
    @patch("bbia_sim.daemon.app.routers.ecosystem.BBIAEmotions")
    async def test_start_demo_simulation_mode(self, mock_emotions, mock_robot_factory):
        """Test démarrage démo mode simulation."""
        from bbia_sim.daemon.app.routers.ecosystem import start_demo_mode

        # Mock simulation service - patcher le module importé localement
        mock_sim_service = MagicMock()
        mock_sim_service.is_simulation_ready.return_value = False
        mock_sim_service.start_simulation = AsyncMock(return_value=True)

        # Patcher le module avant l'appel de la fonction
        with patch(
            "bbia_sim.daemon.simulation_service.simulation_service", mock_sim_service
        ):
            # Mock robot factory
            mock_robot = MagicMock()
            mock_robot.connect = MagicMock()
            mock_robot.disconnect = MagicMock()
            mock_robot_factory.create_backend.return_value = mock_robot

            # Mock emotions
            mock_emotions_instance = MagicMock()
            mock_emotions_instance.emotions = {
                "happy": {},
                "sad": {},
                "excited": {},
                "calm": {},
                "curious": {},
                "sleepy": {},
            }
            mock_emotions_instance.set_emotion = MagicMock()
            mock_emotions.return_value = mock_emotions_instance

            # Appel
            result = await start_demo_mode(mode="simulation", duration=5.0)

            # Vérifications
            assert result["mode"] == "simulation"
            assert result["duration"] == 5.0
            assert result["status"] == "started"
            mock_sim_service.start_simulation.assert_called_once()

    async def test_start_demo_invalid_mode(self):
        """Test démarrage démo avec mode invalide."""
        from fastapi import HTTPException

        from bbia_sim.daemon.app.routers.ecosystem import start_demo_mode

        with pytest.raises(HTTPException) as exc_info:
            await start_demo_mode(mode="invalid", duration=10.0)
        assert exc_info.value.status_code == 400

    @patch("bbia_sim.daemon.app.routers.ecosystem.RobotFactory")
    @patch("bbia_sim.daemon.app.routers.ecosystem.BBIAEmotions")
    async def test_start_demo_with_emotion(self, mock_emotions, mock_robot_factory):
        """Test démarrage démo avec émotion."""
        from bbia_sim.daemon.app.routers.ecosystem import start_demo_mode

        # Mock robot factory
        mock_robot = MagicMock()
        mock_robot.connect = MagicMock()
        mock_robot.disconnect = MagicMock()
        mock_robot.set_emotion = MagicMock()
        mock_robot_factory.create_backend.return_value = mock_robot

        # Mock emotions
        mock_emotions_instance = MagicMock()
        mock_emotions_instance.emotions = {
            "happy": {},
            "sad": {},
            "excited": {},
        }
        mock_emotions_instance.set_emotion = MagicMock()
        mock_emotions.return_value = mock_emotions_instance

        # Mock simulation (déjà ready)
        mock_sim_service = MagicMock()
        mock_sim_service.is_simulation_ready.return_value = True

        # Patch l'import dans la fonction
        import sys

        mock_module = type(sys)("sim_service_mock")
        mock_module.simulation_service = mock_sim_service

        with patch.dict(
            "sys.modules", {"bbia_sim.daemon.simulation_service": mock_module}
        ):
            result = await start_demo_mode(
                mode="simulation", duration=10.0, emotion="happy"
            )

            assert result["emotion"] == "happy"
            assert "emotion_applied" in result or "emotion_error" in result


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
