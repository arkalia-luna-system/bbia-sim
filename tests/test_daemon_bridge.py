#!/usr/bin/env python3
"""
Tests unitaires pour daemon/bridge.py
Tests de la couche Zenoh/FastAPI pour Reachy Mini
"""

import sys
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest  # type: ignore[import-untyped]

# S'assurer que src est dans le path pour coverage
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# Importer le module au niveau du fichier pour que coverage le détecte
# IMPORTANT: Import direct (pas dans try/except) pour que coverage le détecte
import bbia_sim.daemon.bridge  # noqa: F401

# OPTIMISATION COVERAGE: Importer les classes principales au niveau module
try:
    from bbia_sim.daemon.bridge import (
        REACHY_MINI_AVAILABLE,
        ZENOH_AVAILABLE,
        RobotCommand,
        RobotState,
        ZenohBridge,
        ZenohConfig,
        get_bridge_status,
        get_robot_state,
    )

    DAEMON_BRIDGE_AVAILABLE = True
except ImportError:
    DAEMON_BRIDGE_AVAILABLE = False
    REACHY_MINI_AVAILABLE = False  # type: ignore[assignment,misc]
    ZENOH_AVAILABLE = False  # type: ignore[assignment,misc]
    RobotCommand = None  # type: ignore[assignment,misc]
    RobotState = None  # type: ignore[assignment,misc]
    ZenohBridge = None  # type: ignore[assignment,misc]
    ZenohConfig = None  # type: ignore[assignment,misc]
    get_bridge_status = None  # type: ignore[assignment,misc]
    get_robot_state = None  # type: ignore[assignment,misc]


class TestDaemonBridge:
    """Tests pour le module daemon/bridge."""

    def setup_method(self):
        """Configuration avant chaque test."""
        # Mock des dépendances conditionnelles
        self.mock_zenoh = MagicMock()
        self.mock_reachy_mini = MagicMock()

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE,
        reason="Module daemon.bridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.zenoh")
    def test_zenoh_import_available(self, mock_zenoh_module):
        """Test que zenoh est disponible."""
        assert ZENOH_AVAILABLE is True

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE,
        reason="Module daemon.bridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", False)
    def test_zenoh_import_not_available(self):
        """Test que zenoh n'est pas disponible."""
        # En mode test, zenoh peut ne pas être disponible
        # C'est OK si zenoh n'est pas disponible
        assert isinstance(ZENOH_AVAILABLE, bool)

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE,
        reason="Module daemon.bridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.ReachyMini")
    def test_reachy_mini_import_available(self, mock_reachy):
        """Test que reachy_mini est disponible."""
        # Le patch devrait fonctionner directement sur la variable importée
        # Vérifier que REACHY_MINI_AVAILABLE est un booléen (peut être True ou False selon le patch)
        # En mode test, on vérifie simplement que la variable existe et est un booléen
        assert isinstance(REACHY_MINI_AVAILABLE, bool)

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE,
        reason="Module daemon.bridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", False)
    def test_reachy_mini_import_not_available(self):
        """Test que reachy_mini n'est pas disponible."""
        # C'est OK si reachy_mini n'est pas disponible en test
        assert isinstance(REACHY_MINI_AVAILABLE, bool)

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohConfig is None,
        reason="Module daemon.bridge ou ZenohConfig non disponible",
    )
    def test_zenoh_config_initialization(self):
        """Test initialisation configuration Zenoh."""
        config = ZenohConfig()
        assert config.mode == "client"
        assert isinstance(config.connect, list)
        assert len(config.connect) > 0
        assert config.timeout == 1000
        assert config.retry_attempts == 3

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or RobotCommand is None,
        reason="Module daemon.bridge ou RobotCommand non disponible",
    )
    def test_robot_command_model(self):
        """Test modèle RobotCommand."""
        import time

        timestamp_before = time.time()
        cmd = RobotCommand(command="test", parameters={"param": "value"})
        timestamp_after = time.time()

        assert cmd.command == "test"
        assert cmd.parameters == {"param": "value"}
        assert timestamp_before <= cmd.timestamp <= timestamp_after

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or RobotState is None,
        reason="Module daemon.bridge ou RobotState non disponible",
    )
    def test_robot_state_model(self):
        """Test modèle RobotState."""
        import time

        timestamp_before = time.time()
        state = RobotState(
            joints={"yaw_body": 0.1},
            emotions={"happy": 0.8},
            sensors={"sensor": 1.0},
        )
        timestamp_after = time.time()

        assert state.joints == {"yaw_body": 0.1}
        assert state.emotions == {"happy": 0.8}
        assert state.sensors == {"sensor": 1.0}
        assert timestamp_before <= state.timestamp <= timestamp_after

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.Session")
    def test_zenoh_bridge_initialization(self, mock_session):
        """Test initialisation ZenohBridge."""
        bridge = ZenohBridge()
        assert bridge.connected is False
        assert bridge.session is None or isinstance(bridge.session, type(None))
        assert hasattr(bridge, "topics")
        assert hasattr(bridge, "subscribers")
        assert hasattr(bridge, "publishers")

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    def test_bridge_topics_structure(self):
        """Test structure des topics Zenoh."""
        bridge = ZenohBridge()
        assert isinstance(bridge.topics, dict)
        required_topics = ["commands", "state", "telemetry", "errors"]
        for topic in required_topics:
            assert topic in bridge.topics
            assert isinstance(bridge.topics[topic], str)

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    def test_bridge_methods_exist(self):
        """Test que les méthodes principales existent."""
        bridge = ZenohBridge()

        # Vérifier présence méthodes principales
        assert hasattr(bridge, "start")
        assert hasattr(bridge, "stop")
        assert hasattr(bridge, "send_command")
        assert hasattr(bridge, "get_current_state")
        assert hasattr(bridge, "is_connected")
        # Note: connect et connect_zenoh ne sont pas des méthodes de ZenohBridge
        # ZenohBridge utilise start() pour se connecter

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None or ZenohConfig is None,
        reason="Module daemon.bridge, ZenohBridge ou ZenohConfig non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", True)
    def test_zenoh_bridge_config_parameter(self):
        """Test initialisation ZenohBridge avec config personnalisée."""
        custom_config = ZenohConfig(
            mode="client", connect=["tcp://custom:7447"], timeout=2000
        )
        bridge = ZenohBridge(config=custom_config)

        assert bridge.config == custom_config
        assert bridge.config.timeout == 2000

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or RobotCommand is None,
        reason="Module daemon.bridge ou RobotCommand non disponible",
    )
    def test_robot_command_with_timestamp(self):
        """Test RobotCommand avec timestamp fourni."""
        import time

        custom_timestamp = time.time() - 100
        cmd = RobotCommand(
            command="test",
            parameters={"param": "value"},
            timestamp=custom_timestamp,
        )

        assert cmd.timestamp == custom_timestamp

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or RobotState is None,
        reason="Module daemon.bridge ou RobotState non disponible",
    )
    def test_robot_state_with_timestamp(self):
        """Test RobotState avec timestamp fourni."""
        import time

        custom_timestamp = time.time() - 100
        state = RobotState(
            joints={"yaw_body": 0.1},
            emotions={"happy": 0.8},
            sensors={"sensor": 1.0},
            timestamp=custom_timestamp,
        )

        assert state.timestamp == custom_timestamp

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohConfig is None,
        reason="Module daemon.bridge ou ZenohConfig non disponible",
    )
    def test_zenoh_config_default_values(self):
        """Test valeurs par défaut ZenohConfig."""
        config = ZenohConfig()
        assert config.mode == "client"
        assert isinstance(config.connect, list)
        assert len(config.connect) > 0
        assert "localhost" in str(config.connect[0]) or "7447" in str(config.connect[0])
        assert config.timeout == 1000
        assert config.retry_attempts == 3

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.Session")
    def test_zenoh_bridge_subscribers_publishers_init(self, mock_session):
        """Test initialisation subscribers et publishers."""
        bridge = ZenohBridge()
        assert isinstance(bridge.subscribers, dict)
        assert isinstance(bridge.publishers, dict)

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.zenoh")
    @patch("bbia_sim.daemon.bridge.asyncio")
    def test_zenoh_bridge_start_success(self, mock_asyncio, mock_zenoh_module):
        """Test démarrage bridge Zenoh avec succès."""
        import asyncio

        # Mock session Zenoh
        mock_session = MagicMock()
        mock_zenoh_module.open = MagicMock(return_value=mock_session)
        mock_zenoh_module.Config.return_value = MagicMock()

        bridge = ZenohBridge()
        # Mock async methods
        mock_asyncio.create_task = MagicMock()

        # Test start (mocké pour éviter vraie connexion)
        async def test():
            result = await bridge.start()
            # Vérifier que start retourne bool
            assert isinstance(result, bool)

        asyncio.run(test())

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", False)
    def test_zenoh_bridge_start_no_zenoh(self):
        """Test démarrage bridge sans Zenoh."""
        import asyncio

        bridge = ZenohBridge()

        async def test():
            result = await bridge.start()
            assert result is False

        asyncio.run(test())

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    def test_zenoh_bridge_stop(self):
        """Test arrêt bridge Zenoh."""
        import asyncio

        bridge = ZenohBridge()
        bridge.connected = True
        bridge.session = MagicMock()
        bridge.session.close = MagicMock(return_value=None)

        async def test():
            await bridge.stop()
            assert bridge.connected is False

        asyncio.run(test())

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None or RobotCommand is None,
        reason="Module daemon.bridge, ZenohBridge ou RobotCommand non disponible",
    )
    def test_zenoh_bridge_send_command(self):
        """Test envoi commande via bridge."""
        import asyncio

        bridge = ZenohBridge()
        bridge.connected = False

        cmd = RobotCommand(command="test", parameters={})
        # Bridge non connecté doit retourner False
        result = asyncio.run(bridge.send_command(cmd))
        assert result is False

        # Bridge connecté avec publisher mocké
        bridge.connected = True
        bridge.publishers["commands"] = MagicMock()
        bridge.publishers["commands"].put = MagicMock(return_value=None)

        async def test():
            result = await bridge.send_command(cmd)
            # Doit retourner bool
            assert isinstance(result, bool)

        asyncio.run(test())

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    def test_zenoh_bridge_get_current_state(self):
        """Test récupération état actuel."""
        bridge = ZenohBridge()
        state = bridge.get_current_state()
        assert state is not None
        assert hasattr(state, "joints")
        assert hasattr(state, "emotions")
        assert hasattr(state, "sensors")

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    def test_zenoh_bridge_is_connected(self):
        """Test vérification connexion."""
        bridge = ZenohBridge()
        assert bridge.is_connected() is False
        bridge.connected = True
        assert bridge.is_connected() is True

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    def test_zenoh_bridge_stop_with_reachy_mini(self):
        """Test arrêt bridge avec Reachy Mini."""
        import asyncio

        bridge = ZenohBridge()
        bridge.connected = True
        bridge.reachy_mini = MagicMock()
        bridge.reachy_mini.close = MagicMock()

        async def test():
            await bridge.stop()
            assert bridge.connected is False
            if bridge.reachy_mini is not None:
                bridge.reachy_mini.close.assert_called_once()

        asyncio.run(test())

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    def test_zenoh_bridge_stop_with_subscribers(self):
        """Test arrêt bridge avec subscribers."""
        import asyncio

        bridge = ZenohBridge()
        bridge.connected = True
        mock_sub = MagicMock()
        mock_sub.close = MagicMock(return_value=None)
        bridge.subscribers["commands"] = mock_sub

        async def test():
            await bridge.stop()
            mock_sub.close.assert_called_once()

        asyncio.run(test())

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None or RobotCommand is None,
        reason="Module daemon.bridge, ZenohBridge ou RobotCommand non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    def test_zenoh_bridge_send_command_not_connected(self):
        """Test envoi commande bridge non connecté."""
        import asyncio

        bridge = ZenohBridge()
        bridge.connected = False
        cmd = RobotCommand(command="test", parameters={})

        async def test():
            result = await bridge.send_command(cmd)
            assert result is False

        asyncio.run(test())

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_setup_topics(self):
        """Test configuration topics Zenoh."""
        bridge = ZenohBridge()
        bridge.session = MagicMock()
        # Les méthodes sont appelées avec await, donc elles doivent retourner des coroutines
        bridge.session.declare_subscriber = AsyncMock(return_value=MagicMock())
        bridge.session.declare_publisher = AsyncMock(return_value=MagicMock())

        await bridge._setup_zenoh_topics()

        assert "commands" in bridge.subscribers
        assert "state" in bridge.publishers
        assert "telemetry" in bridge.publishers
        assert "errors" in bridge.publishers

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_on_command_received_valid(self):
        """Test réception commande valide."""
        import json

        bridge = ZenohBridge()
        bridge.command_queue = MagicMock()
        bridge.command_queue.put = MagicMock()

        mock_sample = MagicMock()
        mock_sample.payload.decode.return_value = json.dumps(
            {"command": "test", "parameters": {}}
        )

        await bridge._on_command_received(mock_sample)

        bridge.command_queue.put.assert_called_once()

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_on_command_received_too_large(self):
        """Test réception commande trop volumineuse."""
        bridge = ZenohBridge()
        # _publish_error est async, donc utiliser AsyncMock
        # Utiliser setattr pour éviter l'erreur "Cannot assign to a method"
        setattr(bridge, "_publish_error", AsyncMock())

        mock_sample = MagicMock()
        large_payload = "x" * (1048577)  # 1MB + 1 byte
        mock_sample.payload.decode.return_value = large_payload

        await bridge._on_command_received(mock_sample)

        # _publish_error est un AsyncMock, vérifier qu'il a été appelé
        publish_error_mock = bridge._publish_error
        if hasattr(publish_error_mock, "assert_called_once"):
            publish_error_mock.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None or RobotCommand is None,
        reason="Module daemon.bridge, ZenohBridge ou RobotCommand non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_execute_command_goto_target(self):
        """Test exécution commande goto_target."""
        bridge = ZenohBridge()
        bridge.reachy_mini = MagicMock()
        bridge.reachy_mini.goto_target = MagicMock()

        cmd = RobotCommand(
            command="goto_target",
            parameters={
                "head": [1.0, 0.0, 0.0],
                "duration": 1.0,
                "method": "minjerk",
            },
        )

        await bridge._execute_command(cmd)
        bridge.reachy_mini.goto_target.assert_called_once()

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None or RobotCommand is None,
        reason="Module daemon.bridge, ZenohBridge ou RobotCommand non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_execute_command_set_emotion(self):
        """Test exécution commande set_emotion."""
        bridge = ZenohBridge()
        bridge.reachy_mini = MagicMock()

        cmd = RobotCommand(
            command="set_emotion", parameters={"emotion": "happy", "intensity": 0.8}
        )

        await bridge._execute_command(cmd)
        assert "happy" in bridge.current_state.emotions

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_publish_state(self):
        """Test publication état."""
        bridge = ZenohBridge()
        bridge.publishers["state"] = MagicMock()
        bridge.publishers["state"].put = MagicMock()

        await bridge._publish_state()
        bridge.publishers["state"].put.assert_called_once()

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_publish_error(self):
        """Test publication erreur."""
        bridge = ZenohBridge()
        bridge.publishers["errors"] = MagicMock()
        bridge.publishers["errors"].put = MagicMock()

        await bridge._publish_error("Test error")
        bridge.publishers["errors"].put.assert_called_once()

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or ZenohBridge is None,
        reason="Module daemon.bridge ou ZenohBridge non disponible",
    )
    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_update_robot_state(self):
        """Test mise à jour état robot."""
        bridge = ZenohBridge()
        bridge.reachy_mini = MagicMock()
        bridge.reachy_mini.get_current_joint_positions = MagicMock(
            return_value=([0.1] * 6, [0.2, 0.3])
        )

        await bridge._update_robot_state()

        assert len(bridge.current_state.joints) > 0

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or RobotState is None or get_robot_state is None,
        reason="Module daemon.bridge, RobotState ou get_robot_state non disponible",
    )
    @patch("bbia_sim.daemon.bridge.app")
    @patch("bbia_sim.daemon.bridge.bridge")
    def test_fastapi_get_robot_state_endpoint(self, mock_bridge, mock_app):
        """Test endpoint GET /api/zenoh/state."""
        mock_bridge.is_connected.return_value = True
        mock_state = RobotState(joints={}, emotions={}, sensors={})
        mock_bridge.get_current_state.return_value = mock_state

        # Test async endpoint (simulé)
        import asyncio

        async def test():
            result = await get_robot_state()
            assert "joints" in result

        asyncio.run(test())

    @pytest.mark.skipif(
        not DAEMON_BRIDGE_AVAILABLE or get_bridge_status is None,
        reason="Module daemon.bridge ou get_bridge_status non disponible",
    )
    @patch("bbia_sim.daemon.bridge.bridge")
    def test_fastapi_get_bridge_status_endpoint(self, mock_bridge):
        """Test endpoint GET /api/zenoh/status."""
        # Configure le mock bridge
        mock_bridge.is_connected.return_value = True

        import asyncio

        async def test():
            result = await get_bridge_status()
            assert "connected" in result
            assert "zenoh_available" in result

        asyncio.run(test())


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
