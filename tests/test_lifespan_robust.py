#!/usr/bin/env python3
"""Tests pour lifespan context manager robuste."""

from unittest.mock import AsyncMock, patch

import pytest

from bbia_sim.daemon.app.main import app, app_state, lifespan


class TestLifespanRobust:
    """Tests pour lifespan avec retry et fallback."""

    @pytest.mark.asyncio
    async def test_lifespan_retries_on_startup_failure(self):
        """Test retry si startup échoue."""
        with patch("bbia_sim.daemon.app.main.simulation_service") as mock_sim:
            # Simuler 2 échecs puis succès
            mock_sim.start_simulation = AsyncMock(side_effect=[False, False, True])
            mock_sim.stop_simulation = AsyncMock()

            async with lifespan(app):
                # Vérifier que retry a été fait
                assert mock_sim.start_simulation.call_count == 3
                assert app_state["is_running"] is True

    @pytest.mark.asyncio
    async def test_lifespan_fallback_if_sim_unavailable(self):
        """Test fallback si sim non disponible."""
        with patch("bbia_sim.daemon.app.main.simulation_service") as mock_sim:
            # Simuler échec permanent
            mock_sim.start_simulation = AsyncMock(return_value=False)
            mock_sim.stop_simulation = AsyncMock()

            async with lifespan(app):
                # App devrait démarrer même si sim échoue
                assert app is not None
                assert app_state["is_running"] is False
                assert app_state["simulator"] is None

    @pytest.mark.asyncio
    async def test_lifespan_continues_without_sim(self):
        """Test app démarre même sans sim."""
        with patch("bbia_sim.daemon.app.main.simulation_service") as mock_sim:
            mock_sim.start_simulation = AsyncMock(return_value=False)
            mock_sim.stop_simulation = AsyncMock()

            async with lifespan(app):
                # Vérifier que app_state est correct
                assert app_state["simulator"] is None
                assert app_state["is_running"] is False

    @pytest.mark.asyncio
    async def test_lifespan_handles_exception_during_startup(self):
        """Test gestion exception lors startup."""
        with patch("bbia_sim.daemon.app.main.simulation_service") as mock_sim:
            # Simuler exception puis succès
            mock_sim.start_simulation = AsyncMock(
                side_effect=[Exception("Erreur simulation"), True]
            )
            mock_sim.stop_simulation = AsyncMock()

            async with lifespan(app):
                # Vérifier que retry a été fait après exception
                assert mock_sim.start_simulation.call_count == 2
                assert app_state["is_running"] is True

    @pytest.mark.asyncio
    async def test_lifespan_retry_delay(self):
        """Test que retry attend avant nouvelle tentative."""
        with patch("bbia_sim.daemon.app.main.simulation_service") as mock_sim:
            with patch("asyncio.sleep") as mock_sleep:
                # Simuler 2 échecs puis succès
                mock_sim.start_simulation = AsyncMock(side_effect=[False, False, True])
                mock_sim.stop_simulation = AsyncMock()

                async with lifespan(app):
                    # Vérifier que sleep a été appelé 2 fois (pour 2 retries)
                    assert mock_sleep.call_count == 2

    @pytest.mark.asyncio
    async def test_lifespan_max_retries(self):
        """Test que max retries est respecté."""
        with patch("bbia_sim.daemon.app.main.simulation_service") as mock_sim:
            # Simuler échec permanent (3 tentatives)
            mock_sim.start_simulation = AsyncMock(return_value=False)
            mock_sim.stop_simulation = AsyncMock()

            async with lifespan(app):
                # Vérifier que max_retries (3) a été respecté
                assert mock_sim.start_simulation.call_count == 3
                assert app_state["is_running"] is False
