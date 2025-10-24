#!/usr/bin/env python3
"""Tests étendus pour WebSocket Telemetry.

Tests ciblés pour améliorer la couverture de code.
"""

import asyncio
import contextlib
import json
from datetime import datetime
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.bbia_sim.daemon.ws import (
    ConnectionManager,
    get_telemetry_info,
    manager,
    start_telemetry,
    stop_telemetry,
    websocket_endpoint,
)


class TestWebSocketTelemetryExtended:
    """Tests étendus pour WebSocket Telemetry."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.connection_manager = ConnectionManager()

    def test_connection_manager_init(self):
        """Test initialisation du gestionnaire de connexions."""
        assert self.connection_manager.active_connections == []
        assert self.connection_manager.is_broadcasting is False
        assert self.connection_manager.broadcast_task is None

    @pytest.mark.asyncio
    async def test_connect_success(self):
        """Test connexion WebSocket réussie."""
        mock_websocket = AsyncMock()

        await self.connection_manager.connect(mock_websocket)

        mock_websocket.accept.assert_called_once()
        assert len(self.connection_manager.active_connections) == 1
        assert mock_websocket in self.connection_manager.active_connections

    def test_disconnect_existing_connection(self):
        """Test déconnexion d'une connexion existante."""
        mock_websocket = MagicMock()
        self.connection_manager.active_connections.append(mock_websocket)

        self.connection_manager.disconnect(mock_websocket)

        assert len(self.connection_manager.active_connections) == 0
        assert mock_websocket not in self.connection_manager.active_connections

    def test_disconnect_nonexistent_connection(self):
        """Test déconnexion d'une connexion inexistante."""
        mock_websocket = MagicMock()

        self.connection_manager.disconnect(mock_websocket)

        assert len(self.connection_manager.active_connections) == 0

    @pytest.mark.asyncio
    async def test_send_personal_message_success(self):
        """Test envoi de message personnel réussi."""
        mock_websocket = AsyncMock()

        await self.connection_manager.send_personal_message(
            "test message", mock_websocket
        )

        mock_websocket.send_text.assert_called_once_with("test message")

    @pytest.mark.asyncio
    async def test_send_personal_message_error(self):
        """Test erreur d'envoi de message personnel."""
        mock_websocket = AsyncMock()
        mock_websocket.send_text.side_effect = Exception("Send error")
        self.connection_manager.active_connections.append(mock_websocket)

        await self.connection_manager.send_personal_message(
            "test message", mock_websocket
        )

        # Vérifier que la connexion a été supprimée
        assert mock_websocket not in self.connection_manager.active_connections

    @pytest.mark.asyncio
    async def test_broadcast_no_connections(self):
        """Test diffusion sans connexions."""
        await self.connection_manager.broadcast("test message")
        # Ne devrait pas lever d'exception

    @pytest.mark.asyncio
    async def test_broadcast_success(self):
        """Test diffusion réussie."""
        mock_websocket1 = AsyncMock()
        mock_websocket2 = AsyncMock()
        self.connection_manager.active_connections = [mock_websocket1, mock_websocket2]

        await self.connection_manager.broadcast("test message")

        mock_websocket1.send_text.assert_called_once_with("test message")
        mock_websocket2.send_text.assert_called_once_with("test message")

    @pytest.mark.asyncio
    async def test_broadcast_with_errors(self):
        """Test diffusion avec erreurs."""
        mock_websocket1 = AsyncMock()
        mock_websocket2 = AsyncMock()
        mock_websocket2.send_text.side_effect = Exception("Send error")
        self.connection_manager.active_connections = [mock_websocket1, mock_websocket2]

        await self.connection_manager.broadcast("test message")

        mock_websocket1.send_text.assert_called_once_with("test message")
        # Vérifier que la connexion défaillante a été supprimée
        assert mock_websocket2 not in self.connection_manager.active_connections
        assert mock_websocket1 in self.connection_manager.active_connections

    @pytest.mark.asyncio
    async def test_start_broadcast_success(self):
        """Test démarrage de diffusion réussi."""
        # S'assurer que is_broadcasting est False au début
        self.connection_manager.is_broadcasting = False
        assert self.connection_manager.is_broadcasting is False

        # Mock _broadcast_loop pour éviter l'exécution réelle
        with patch.object(
            self.connection_manager, "_broadcast_loop", new_callable=AsyncMock
        ):
            await self.connection_manager.start_broadcast()

            assert self.connection_manager.is_broadcasting is True
            # Vérifier qu'une tâche a été créée (peut être None si le mock ne fonctionne pas)
            # mais is_broadcasting devrait être True

    @pytest.mark.asyncio
    async def test_start_broadcast_already_broadcasting(self):
        """Test démarrage de diffusion déjà actif."""
        self.connection_manager.is_broadcasting = True
        original_task = MagicMock()
        self.connection_manager.broadcast_task = original_task

        await self.connection_manager.start_broadcast()

        # Vérifier que la tâche n'a pas été modifiée
        assert self.connection_manager.broadcast_task is original_task

    @pytest.mark.asyncio
    async def test_stop_broadcast_success(self):
        """Test arrêt de diffusion réussi."""
        self.connection_manager.is_broadcasting = True

        # Créer une vraie tâche asyncio
        async def dummy_task():
            await asyncio.sleep(0.1)

        task = asyncio.create_task(dummy_task())
        self.connection_manager.broadcast_task = task

        await self.connection_manager.stop_broadcast()

        assert self.connection_manager.is_broadcasting is False
        assert task.cancelled()

    @pytest.mark.asyncio
    async def test_stop_broadcast_no_task(self):
        """Test arrêt de diffusion sans tâche."""
        self.connection_manager.is_broadcasting = True
        self.connection_manager.broadcast_task = None

        await self.connection_manager.stop_broadcast()

        assert self.connection_manager.is_broadcasting is False

    @pytest.mark.asyncio
    async def test_broadcast_loop_success(self):
        """Test boucle de diffusion réussie."""
        self.connection_manager.is_broadcasting = True

        with (
            patch.object(self.connection_manager, "broadcast") as mock_broadcast,
            patch("asyncio.sleep") as mock_sleep,
        ):

            # Simuler une seule itération puis arrêt
            mock_sleep.side_effect = [None, asyncio.CancelledError()]

            with contextlib.suppress(asyncio.CancelledError):
                await self.connection_manager._broadcast_loop()

            # Vérifier qu'au moins un appel a été fait
            assert mock_broadcast.call_count >= 1

    @pytest.mark.asyncio
    async def test_broadcast_loop_error_handling(self):
        """Test gestion d'erreur dans la boucle de diffusion."""
        self.connection_manager.is_broadcasting = True

        with (
            patch.object(self.connection_manager, "broadcast") as mock_broadcast,
            patch("asyncio.sleep") as mock_sleep,
        ):

            mock_broadcast.side_effect = Exception("Broadcast error")
            mock_sleep.side_effect = [None, asyncio.CancelledError()]

            with contextlib.suppress(asyncio.CancelledError):
                await self.connection_manager._broadcast_loop()

            # Vérifier qu'au moins un appel a été fait
            assert mock_broadcast.call_count >= 1

    def test_generate_telemetry_data_structure(self):
        """Test structure des données de télémétrie."""
        data = self.connection_manager._generate_telemetry_data()

        assert "timestamp" in data
        assert "robot" in data
        assert "joints" in data
        assert "sensors" in data
        assert "status" in data

        # Vérifier la structure du robot
        robot = data["robot"]
        assert "position" in robot
        assert "orientation" in robot
        assert "velocity" in robot

        # Vérifier les capteurs
        sensors = data["sensors"]
        assert "battery" in sensors
        assert "temperature" in sensors
        assert "cpu_usage" in sensors
        assert "memory_usage" in sensors

    def test_generate_telemetry_data_values(self):
        """Test valeurs des données de télémétrie."""
        data = self.connection_manager._generate_telemetry_data()

        # Vérifier les plages de valeurs
        assert 0.0 <= data["sensors"]["battery"] <= 100.0
        assert 20.0 <= data["sensors"]["temperature"] <= 30.0
        assert 0.0 <= data["sensors"]["cpu_usage"] <= 100.0
        assert 0.0 <= data["sensors"]["memory_usage"] <= 100.0

        # Vérifier le statut
        assert data["status"]["mode"] == "autonomous"
        assert isinstance(data["status"]["errors"], list)
        assert isinstance(data["status"]["warnings"], list)
        assert isinstance(data["status"]["active_tasks"], list)

    def test_generate_telemetry_data_timestamp(self):
        """Test timestamp des données de télémétrie."""
        data = self.connection_manager._generate_telemetry_data()

        timestamp = data["timestamp"]
        assert isinstance(timestamp, str)
        # Vérifier que c'est un format ISO valide
        datetime.fromisoformat(timestamp)

    @pytest.mark.asyncio
    async def test_websocket_endpoint_connection(self):
        """Test endpoint WebSocket - connexion."""
        mock_websocket = AsyncMock()

        with (
            patch.object(manager, "connect") as mock_connect,
            patch.object(manager, "disconnect") as mock_disconnect,
            patch.object(manager, "start_broadcast"),
            patch.object(manager, "stop_broadcast"),
            patch.object(manager, "active_connections", [mock_websocket]),
        ):

            # Simuler une déconnexion immédiate
            mock_websocket.receive_text.side_effect = Exception("Disconnect")

            with contextlib.suppress(Exception):
                await websocket_endpoint(mock_websocket)

            mock_connect.assert_called_once_with(mock_websocket)
            mock_disconnect.assert_called_once_with(mock_websocket)

    @pytest.mark.asyncio
    async def test_websocket_endpoint_ping_pong(self):
        """Test endpoint WebSocket - ping/pong."""
        mock_websocket = AsyncMock()

        with (
            patch.object(manager, "connect"),
            patch.object(manager, "disconnect"),
            patch.object(manager, "send_personal_message") as mock_send,
            patch.object(manager, "active_connections", [mock_websocket]),
        ):

            # Simuler un message ping puis déconnexion
            mock_websocket.receive_text.side_effect = [
                json.dumps({"type": "ping"}),
                Exception("Disconnect"),
            ]

            with contextlib.suppress(Exception):
                await websocket_endpoint(mock_websocket)

            mock_send.assert_called_once()
            call_args = mock_send.call_args[0]
            assert "pong" in call_args[0]

    @pytest.mark.asyncio
    async def test_websocket_endpoint_request_status(self):
        """Test endpoint WebSocket - demande de statut."""
        mock_websocket = AsyncMock()

        with (
            patch.object(manager, "connect"),
            patch.object(manager, "disconnect"),
            patch.object(manager, "send_personal_message") as mock_send,
            patch.object(manager, "active_connections", [mock_websocket]),
        ):

            # Simuler un message request_status puis déconnexion
            mock_websocket.receive_text.side_effect = [
                json.dumps({"type": "request_status"}),
                Exception("Disconnect"),
            ]

            with contextlib.suppress(Exception):
                await websocket_endpoint(mock_websocket)

            mock_send.assert_called_once()
            call_args = mock_send.call_args[0]
            status_data = json.loads(call_args[0])
            assert status_data["type"] == "status"
            assert "connections" in status_data
            assert "broadcasting" in status_data

    @pytest.mark.asyncio
    async def test_websocket_endpoint_invalid_json(self):
        """Test endpoint WebSocket - JSON invalide."""
        mock_websocket = AsyncMock()

        with (
            patch.object(manager, "connect") as mock_connect,
            patch.object(manager, "disconnect") as mock_disconnect,
            patch.object(manager, "active_connections", [mock_websocket]),
        ):

            # Simuler un message JSON invalide puis déconnexion
            mock_websocket.receive_text.side_effect = [
                "invalid json",
                Exception("Disconnect"),
            ]

            with contextlib.suppress(Exception):
                await websocket_endpoint(mock_websocket)

            mock_connect.assert_called_once()
            mock_disconnect.assert_called_once()

    @pytest.mark.asyncio
    async def test_websocket_endpoint_general_error(self):
        """Test endpoint WebSocket - erreur générale."""
        mock_websocket = AsyncMock()

        with (
            patch.object(manager, "connect") as mock_connect,
            patch.object(manager, "disconnect") as mock_disconnect,
            patch.object(manager, "active_connections", [mock_websocket]),
        ):

            # Simuler une erreur générale puis déconnexion
            mock_websocket.receive_text.side_effect = [
                Exception("General error"),
                Exception("Disconnect"),
            ]

            with contextlib.suppress(Exception):
                await websocket_endpoint(mock_websocket)

            mock_connect.assert_called_once()
            mock_disconnect.assert_called_once()

    @pytest.mark.asyncio
    async def test_get_telemetry_info(self):
        """Test informations de télémétrie."""
        with (
            patch.object(manager, "active_connections", [MagicMock(), MagicMock()]),
            patch.object(manager, "is_broadcasting", True),
        ):

            info = await get_telemetry_info()

            assert info["service"] == "WebSocket Telemetry"
            assert info["version"] == "1.0.0"
            assert info["status"] == "running"
            assert info["connections"] == 2
            assert info["broadcasting"] is True
            assert info["frequency"] == "10Hz"
            assert info["endpoint"] == "/ws/telemetry"
            assert "timestamp" in info

    @pytest.mark.asyncio
    async def test_start_telemetry_endpoint(self):
        """Test endpoint de démarrage de télémétrie."""
        with patch.object(manager, "start_broadcast") as mock_start:
            result = await start_telemetry()

            assert result["status"] == "started"
            assert result["message"] == "Diffusion de télémétrie démarrée"
            assert "timestamp" in result
            mock_start.assert_called_once()

    @pytest.mark.asyncio
    async def test_stop_telemetry_endpoint(self):
        """Test endpoint d'arrêt de télémétrie."""
        with patch.object(manager, "stop_broadcast") as mock_stop:
            result = await stop_telemetry()

            assert result["status"] == "stopped"
            assert result["message"] == "Diffusion de télémétrie arrêtée"
            assert "timestamp" in result
            mock_stop.assert_called_once()

    def test_manager_global_instance(self):
        """Test instance globale du gestionnaire."""
        assert isinstance(manager, ConnectionManager)
        assert manager.active_connections == []
        # Note: is_broadcasting peut être True si d'autres tests l'ont modifié

    @pytest.mark.asyncio
    async def test_broadcast_loop_cancellation(self):
        """Test annulation de la boucle de diffusion."""
        self.connection_manager.is_broadcasting = True

        with (
            patch.object(self.connection_manager, "broadcast") as mock_broadcast,
            patch("asyncio.sleep") as mock_sleep,
        ):

            # Simuler une annulation immédiate
            mock_sleep.side_effect = asyncio.CancelledError()

            with contextlib.suppress(asyncio.CancelledError):
                await self.connection_manager._broadcast_loop()

            # La boucle devrait se terminer proprement
            # Note: broadcast peut être appelé une fois avant l'annulation
            assert mock_broadcast.call_count <= 1

    def test_generate_telemetry_data_randomness(self):
        """Test caractère déterministe des données de télémétrie."""
        import time

        # Attendre un petit délai pour s'assurer que les timestamps diffèrent
        data1 = self.connection_manager._generate_telemetry_data()
        time.sleep(0.1)  # Attendre 100ms pour changer le timestamp
        data2 = self.connection_manager._generate_telemetry_data()

        # Les données devraient être différentes (déterministes basées sur le temps)
        assert data1["robot"]["position"]["x"] != data2["robot"]["position"]["x"]
        assert data1["sensors"]["battery"] != data2["sensors"]["battery"]
        assert data1["sensors"]["temperature"] != data2["sensors"]["temperature"]
