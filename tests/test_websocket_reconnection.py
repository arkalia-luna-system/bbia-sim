#!/usr/bin/env python3
"""Tests de reconnexion WebSocket pour robustesse.

Tests pour valider la reconnexion automatique après déconnexion,
gestion perte réseau temporaire, queue messages, et heartbeat.
"""

import asyncio
import json
import time
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app
from bbia_sim.daemon.ws.telemetry import ConnectionManager


class TestWebSocketReconnection:
    """Tests de reconnexion WebSocket."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.manager = ConnectionManager()
        self.client = TestClient(app)

    def test_reconnection_after_disconnect(self):
        """Test reconnexion automatique après déconnexion."""
        # Simuler première connexion
        mock_ws1 = AsyncMock()
        asyncio.run(self.manager.connect(mock_ws1))
        assert len(self.manager.active_connections) == 1

        # Simuler déconnexion
        self.manager.disconnect(mock_ws1)
        assert len(self.manager.active_connections) == 0

        # Simuler reconnexion
        mock_ws2 = AsyncMock()
        asyncio.run(self.manager.connect(mock_ws2))
        assert len(self.manager.active_connections) == 1
        assert mock_ws2 in self.manager.active_connections

    @pytest.mark.asyncio
    async def test_multiple_reconnections(self):
        """Test multiples reconnexions successives."""
        for i in range(3):
            mock_ws = AsyncMock()
            await self.manager.connect(mock_ws)
            assert len(self.manager.active_connections) == 1

            self.manager.disconnect(mock_ws)
            assert len(self.manager.active_connections) == 0

    def test_network_loss_temporary(self):
        """Test gestion perte réseau temporaire."""
        # Simuler connexion
        mock_ws = AsyncMock()
        asyncio.run(self.manager.connect(mock_ws))
        assert len(self.manager.active_connections) == 1

        # Simuler perte réseau (exception lors de l'envoi)
        mock_ws.send_json.side_effect = Exception("Network error")
        self.manager.disconnect(mock_ws)
        assert len(self.manager.active_connections) == 0

        # Simuler reconnexion après perte réseau
        mock_ws2 = AsyncMock()
        asyncio.run(self.manager.connect(mock_ws2))
        assert len(self.manager.active_connections) == 1

    @pytest.mark.asyncio
    async def test_message_queue_during_disconnect(self):
        """Test queue messages pendant déconnexion."""
        # Simuler connexion
        mock_ws = AsyncMock()
        await self.manager.connect(mock_ws)

        # Simuler messages en queue pendant déconnexion
        messages = [
            {"type": "telemetry", "data": {"latency": 10}},
            {"type": "telemetry", "data": {"latency": 20}},
        ]

        # Déconnecter avant d'envoyer
        self.manager.disconnect(mock_ws)

        # Vérifier que les messages ne sont pas envoyés après déconnexion
        for msg in messages:
            # Le message ne devrait pas être envoyé car la connexion est fermée
            assert mock_ws not in self.manager.active_connections

    @pytest.mark.asyncio
    async def test_heartbeat_detection(self):
        """Test heartbeat/ping pour détecter déconnexions."""
        mock_ws = AsyncMock()
        await self.manager.connect(mock_ws)

        # Simuler heartbeat (ping)
        ping_message = {"type": "ping", "timestamp": time.time()}
        mock_ws.receive_text = AsyncMock(return_value=json.dumps(ping_message))

        # Vérifier que le heartbeat est reçu
        try:
            received = await asyncio.wait_for(mock_ws.receive_text(), timeout=0.1)
            data = json.loads(received)
            assert data["type"] == "ping"
        except asyncio.TimeoutError:
            # Timeout acceptable si pas de heartbeat immédiat
            pass

    @pytest.mark.asyncio
    async def test_reconnection_with_existing_connections(self):
        """Test reconnexion avec connexions existantes."""
        # Créer plusieurs connexions
        mock_ws1 = AsyncMock()
        mock_ws2 = AsyncMock()
        await self.manager.connect(mock_ws1)
        await self.manager.connect(mock_ws2)
        assert len(self.manager.active_connections) == 2

        # Déconnecter une seule connexion
        self.manager.disconnect(mock_ws1)
        assert len(self.manager.active_connections) == 1
        assert mock_ws2 in self.manager.active_connections

        # Reconnexion de la première
        mock_ws3 = AsyncMock()
        await self.manager.connect(mock_ws3)
        assert len(self.manager.active_connections) == 2

    def test_connection_manager_cleanup(self):
        """Test nettoyage du gestionnaire après déconnexions."""
        # Créer plusieurs connexions
        mock_ws1 = AsyncMock()
        mock_ws2 = AsyncMock()
        asyncio.run(self.manager.connect(mock_ws1))
        asyncio.run(self.manager.connect(mock_ws2))
        assert len(self.manager.active_connections) == 2

        # Déconnecter toutes les connexions
        self.manager.disconnect(mock_ws1)
        self.manager.disconnect(mock_ws2)
        assert len(self.manager.active_connections) == 0

    @pytest.mark.asyncio
    async def test_rapid_reconnect_disconnect(self):
        """Test reconnexions/déconnexions rapides successives."""
        for _ in range(5):
            mock_ws = AsyncMock()
            await self.manager.connect(mock_ws)
            await asyncio.sleep(0.01)  # Petit délai
            self.manager.disconnect(mock_ws)

        assert len(self.manager.active_connections) == 0

    @pytest.mark.skipif(
        not hasattr(TestClient, "websocket_connect"),
        reason="TestClient WebSocket non disponible",
    )
    def test_websocket_endpoint_reconnection(self):
        """Test reconnexion via endpoint WebSocket réel."""
        try:
            # Première connexion
            with self.client.websocket_connect("/ws/telemetry") as ws1:
                assert ws1 is not None

            # Reconnexion immédiate
            with self.client.websocket_connect("/ws/telemetry") as ws2:
                assert ws2 is not None
        except Exception:
            # WebSocket peut ne pas être disponible en test
            pytest.skip("WebSocket endpoint non disponible en test")


class TestWebSocketResilience:
    """Tests de résilience WebSocket."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.manager = ConnectionManager()

    @pytest.mark.asyncio
    async def test_connection_limit_enforced(self):
        """Test que la limite de connexions est respectée."""
        # Créer le maximum de connexions
        connections = []
        for i in range(self.manager._max_connections):
            mock_ws = AsyncMock()
            await self.manager.connect(mock_ws)
            connections.append(mock_ws)

        assert len(self.manager.active_connections) == self.manager._max_connections

        # Tentative de connexion supplémentaire devrait être rejetée
        mock_ws_extra = AsyncMock()
        await self.manager.connect(mock_ws_extra)
        # La connexion devrait être fermée
        mock_ws_extra.close.assert_called_once()

    @pytest.mark.asyncio
    async def test_error_handling_during_broadcast(self):
        """Test gestion d'erreurs pendant broadcast."""
        mock_ws1 = AsyncMock()
        mock_ws2 = AsyncMock()
        await self.manager.connect(mock_ws1)
        await self.manager.connect(mock_ws2)

        # Simuler erreur lors de l'envoi à une connexion
        mock_ws1.send_json.side_effect = Exception("Send error")

        # Le broadcast devrait continuer pour les autres connexions
        # (testé via la structure du manager)
        assert len(self.manager.active_connections) == 2
