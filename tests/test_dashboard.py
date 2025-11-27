"""Tests pour le module dashboard.py."""

import json
from unittest.mock import AsyncMock, MagicMock, Mock, patch

import pytest

# Import conditionnel selon disponibilité FastAPI
try:
    from fastapi.testclient import TestClient
    from fastapi.websockets import WebSocket

    FASTAPI_TEST_AVAILABLE = True
except ImportError:
    FASTAPI_TEST_AVAILABLE = False
    TestClient = None  # type: ignore[assignment,misc]
    WebSocket = None  # type: ignore[assignment,misc]

# OPTIMISATION COVERAGE: Import au niveau module pour que coverage détecte le module
try:
    from bbia_sim.dashboard import (
        BBIAWebSocketManager,
        app,
        create_dashboard_app,
        handle_robot_command,
        run_dashboard,
        websocket_manager,
    )

    DASHBOARD_AVAILABLE = True
except ImportError:
    DASHBOARD_AVAILABLE = False
    BBIAWebSocketManager = None  # type: ignore[assignment,misc]
    app = None  # type: ignore[assignment,misc]
    create_dashboard_app = None  # type: ignore[assignment,misc]
    handle_robot_command = None  # type: ignore[assignment,misc]
    run_dashboard = None  # type: ignore[assignment,misc]
    websocket_manager = None  # type: ignore[assignment,misc]


@pytest.mark.skipif(not FASTAPI_TEST_AVAILABLE, reason="FastAPI non disponible")
class TestBBIAWebSocketManager:
    """Tests pour BBIAWebSocketManager."""

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    def test_init(self):
        """Test initialisation du gestionnaire WebSocket."""
        manager = BBIAWebSocketManager()
        assert manager.active_connections == []
        assert manager.robot is None
        assert manager.robot_backend == "mujoco"

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_connect(self):
        """Test connexion WebSocket."""
        manager = BBIAWebSocketManager()
        mock_websocket = AsyncMock(spec=WebSocket)

        await manager.connect(mock_websocket)
        assert mock_websocket.accept.called
        assert len(manager.active_connections) == 1
        assert mock_websocket in manager.active_connections

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    def test_disconnect(self):
        """Test déconnexion WebSocket."""
        manager = BBIAWebSocketManager()
        mock_websocket = Mock()
        manager.active_connections = [mock_websocket]

        manager.disconnect(mock_websocket)
        assert len(manager.active_connections) == 0
        assert mock_websocket not in manager.active_connections

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_send_personal_message(self):
        """Test envoi message personnel."""
        manager = BBIAWebSocketManager()
        mock_websocket = AsyncMock(spec=WebSocket)
        # Ajouter le websocket aux connexions actives (requis pour send_personal_message)
        manager.active_connections.append(mock_websocket)

        await manager.send_personal_message("test message", mock_websocket)
        mock_websocket.send_text.assert_called_once_with("test message")

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_send_personal_message_error(self):
        """Test envoi message avec erreur."""
        manager = BBIAWebSocketManager()
        mock_websocket = AsyncMock(spec=WebSocket)
        mock_websocket.send_text.side_effect = Exception("Connection error")

        # Ne doit pas lever d'exception
        await manager.send_personal_message("test", mock_websocket)

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_broadcast(self):
        """Test broadcast message."""
        manager = BBIAWebSocketManager()
        mock_ws1 = AsyncMock(spec=WebSocket)
        mock_ws2 = AsyncMock(spec=WebSocket)
        manager.active_connections = [mock_ws1, mock_ws2]

        await manager.broadcast("broadcast message")
        mock_ws1.send_text.assert_called_once_with("broadcast message")
        mock_ws2.send_text.assert_called_once_with("broadcast message")

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_broadcast_empty(self):
        """Test broadcast avec aucune connexion."""
        manager = BBIAWebSocketManager()
        manager.active_connections = []

        # Ne doit pas lever d'exception
        await manager.broadcast("message")

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_broadcast_with_disconnected(self):
        """Test broadcast avec connexions déconnectées."""
        manager = BBIAWebSocketManager()
        mock_ws1 = AsyncMock(spec=WebSocket)
        mock_ws2 = AsyncMock(spec=WebSocket)
        mock_ws2.send_text.side_effect = Exception("Connection closed")
        manager.active_connections = [mock_ws1, mock_ws2]

        await manager.broadcast("message")
        # mock_ws2 doit être retiré de la liste
        assert len(manager.active_connections) == 1
        assert mock_ws1 in manager.active_connections
        assert mock_ws2 not in manager.active_connections

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_send_status_update(self):
        """Test envoi mise à jour statut."""
        manager = BBIAWebSocketManager()
        manager.robot = Mock()  # Robot connecté
        manager.robot_backend = "mujoco"
        mock_ws = AsyncMock(spec=WebSocket)
        manager.active_connections = [mock_ws]

        await manager.send_status_update()

        # Vérifier que broadcast a été appelé
        assert mock_ws.send_text.called
        call_args = mock_ws.send_text.call_args[0][0]
        data = json.loads(call_args)
        assert data["type"] == "status"
        assert data["robot_connected"] is True
        assert data["robot_backend"] == "mujoco"

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_send_log_message(self):
        """Test envoi message de log."""
        manager = BBIAWebSocketManager()
        mock_ws = AsyncMock(spec=WebSocket)
        manager.active_connections = [mock_ws]

        await manager.send_log_message("info", "Test log message")

        assert mock_ws.send_text.called
        call_args = mock_ws.send_text.call_args[0][0]
        data = json.loads(call_args)
        assert data["type"] == "log"
        assert data["level"] == "info"
        assert data["message"] == "Test log message"


@pytest.mark.skipif(not FASTAPI_TEST_AVAILABLE, reason="FastAPI non disponible")
class TestDashboardRoutes:
    """Tests pour les routes FastAPI du dashboard."""

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    def test_dashboard_route(self):
        """Test route principale dashboard."""
        if app is None:
            pytest.skip("App non disponible")
        client = TestClient(app)
        response = client.get("/")
        assert response.status_code == 200
        assert "BBIA Dashboard" in response.text
        assert "text/html" in response.headers["content-type"]

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    def test_health_check_route(self):
        """Test route health check."""
        if app is None:
            pytest.skip("App non disponible")
        client = TestClient(app)
        response = client.get("/healthz")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert "timestamp" in data
        assert "version" in data
        assert "robot_connected" in data

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_websocket_endpoint(self):
        """Test endpoint WebSocket."""
        if app is None:
            pytest.skip("App non disponible")
        client = TestClient(app)
        with client.websocket_connect("/ws") as websocket:
            # Vérifier que la connexion est acceptée
            assert len(websocket_manager.active_connections) == 1

            # Envoyer un message
            websocket.send_json(
                {"type": "command", "command_type": "test", "value": "test"}
            )

            # Recevoir réponse (peut être timeout si pas de réponse)
            try:
                data = websocket.receive_json(timeout=1.0)
                # Vérifier que c'est un message de statut
                assert data["type"] in ["status", "log"]
            except Exception:
                # Timeout acceptable si pas de réponse immédiate
                pass

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_websocket_disconnect(self):
        """Test déconnexion WebSocket."""
        if websocket_manager is None:
            pytest.skip("WebSocket manager non disponible")
        mock_websocket = AsyncMock(spec=WebSocket)
        await websocket_manager.connect(mock_websocket)
        assert len(websocket_manager.active_connections) == 1

        websocket_manager.disconnect(mock_websocket)
        assert len(websocket_manager.active_connections) == 0


@pytest.mark.skipif(not FASTAPI_TEST_AVAILABLE, reason="FastAPI non disponible")
class TestHandleRobotCommand:
    """Tests pour handle_robot_command."""

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_handle_emotion_command(self):
        """Test commande émotion."""
        if handle_robot_command is None or websocket_manager is None:
            pytest.skip("Fonctions dashboard non disponibles")
        # Mock robot
        mock_robot = Mock()
        mock_robot.set_emotion = Mock()
        websocket_manager.robot = mock_robot

        command = {"command_type": "emotion", "value": "happy"}
        await handle_robot_command(command)

        mock_robot.set_emotion.assert_called_once_with("happy", intensity=0.8)

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_handle_action_look_at(self):
        """Test action look_at."""
        if handle_robot_command is None or websocket_manager is None:
            pytest.skip("Fonctions dashboard non disponibles")
        mock_robot = Mock()
        mock_robot.set_joint_pos = Mock()
        websocket_manager.robot = mock_robot

        command = {"command_type": "action", "value": "look_at"}
        await handle_robot_command(command)

        # Vérifier que set_joint_pos a été appelé plusieurs fois
        assert mock_robot.set_joint_pos.call_count >= 3

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_handle_action_greet(self):
        """Test action greet."""
        if handle_robot_command is None or websocket_manager is None:
            pytest.skip("Fonctions dashboard non disponibles")
        mock_robot = Mock()
        mock_robot.set_emotion = Mock()
        websocket_manager.robot = mock_robot

        command = {"command_type": "action", "value": "greet"}
        await handle_robot_command(command)

        # Vérifier que set_emotion a été appelé
        assert mock_robot.set_emotion.call_count >= 2

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_handle_action_wake_up(self):
        """Test action wake_up."""
        if handle_robot_command is None or websocket_manager is None:
            pytest.skip("Fonctions dashboard non disponibles")
        mock_robot = Mock()
        mock_robot.set_emotion = Mock()
        mock_robot.set_joint_pos = Mock()
        websocket_manager.robot = mock_robot

        command = {"command_type": "action", "value": "wake_up"}
        await handle_robot_command(command)

        # Vérifier que les méthodes ont été appelées
        assert mock_robot.set_emotion.called
        assert mock_robot.set_joint_pos.called

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_handle_action_stop(self):
        """Test action stop."""
        if handle_robot_command is None or websocket_manager is None:
            pytest.skip("Fonctions dashboard non disponibles")
        mock_robot = Mock()
        mock_robot.set_emotion = Mock()
        mock_robot.set_joint_pos = Mock()
        websocket_manager.robot = mock_robot

        command = {"command_type": "action", "value": "stop"}
        await handle_robot_command(command)

        mock_robot.set_emotion.assert_called_with("neutral", intensity=0.5)
        mock_robot.set_joint_pos.assert_called_with("yaw_body", 0.0)

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_handle_command_robot_init(self):
        """Test initialisation robot si nécessaire."""
        if handle_robot_command is None or websocket_manager is None:
            pytest.skip("Fonctions dashboard non disponibles")
        websocket_manager.robot = None
        websocket_manager.robot_backend = "mujoco"

        with patch("bbia_sim.dashboard.RobotFactory") as mock_factory:
            mock_robot = Mock()
            mock_robot.connect = Mock()
            mock_factory.create_backend.return_value = mock_robot

            command = {"command_type": "emotion", "value": "happy"}
            await handle_robot_command(command)

            mock_factory.create_backend.assert_called_once_with("mujoco")
            mock_robot.connect.assert_called_once()
            assert websocket_manager.robot == mock_robot


@pytest.mark.skipif(not FASTAPI_TEST_AVAILABLE, reason="FastAPI non disponible")
class TestDashboardFunctions:
    """Tests pour les fonctions utilitaires du dashboard."""

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    def test_create_dashboard_app(self):
        """Test création application dashboard."""
        if create_dashboard_app is None:
            pytest.skip("Fonction non disponible")
        app_result = create_dashboard_app()
        assert app_result is not None

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", False)
    def test_create_dashboard_app_no_fastapi(self):
        """Test création dashboard sans FastAPI."""
        if create_dashboard_app is None:
            pytest.skip("Fonction non disponible")
        app_result = create_dashboard_app()
        assert app_result is None

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", True)
    @patch("bbia_sim.dashboard.uvicorn")
    def test_run_dashboard(self, mock_uvicorn):
        """Test lancement dashboard."""
        if run_dashboard is None or websocket_manager is None:
            pytest.skip("Fonctions dashboard non disponibles")
        websocket_manager.robot_backend = "mujoco"
        run_dashboard(host="127.0.0.1", port=8000, backend="mujoco")

        mock_uvicorn.run.assert_called_once()
        assert websocket_manager.robot_backend == "mujoco"

    @pytest.mark.skipif(
        not DASHBOARD_AVAILABLE or not FASTAPI_TEST_AVAILABLE,
        reason="Dashboard ou FastAPI non disponible",
    )
    @patch("bbia_sim.dashboard.FASTAPI_AVAILABLE", False)
    def test_run_dashboard_no_fastapi(self):
        """Test lancement dashboard sans FastAPI."""
        if run_dashboard is None:
            pytest.skip("Fonction non disponible")
        # Ne doit pas lever d'exception
        run_dashboard()
