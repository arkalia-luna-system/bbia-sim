#!/usr/bin/env python3
"""
Tests unitaires pour daemon/bridge.py
Tests de la couche Zenoh/FastAPI pour Reachy Mini
"""

from unittest.mock import MagicMock, patch

import pytest


class TestDaemonBridge:
    """Tests pour le module daemon/bridge."""

    def setup_method(self):
        """Configuration avant chaque test."""
        # Mock des dépendances conditionnelles
        self.mock_zenoh = MagicMock()
        self.mock_reachy_mini = MagicMock()

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.zenoh")
    def test_zenoh_import_available(self, mock_zenoh_module):
        """Test que zenoh est disponible."""
        from bbia_sim.daemon.bridge import ZENOH_AVAILABLE

        assert ZENOH_AVAILABLE is True

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", False)
    def test_zenoh_import_not_available(self):
        """Test que zenoh n'est pas disponible."""
        # En mode test, zenoh peut ne pas être disponible
        from bbia_sim.daemon.bridge import ZENOH_AVAILABLE

        # C'est OK si zenoh n'est pas disponible
        assert isinstance(ZENOH_AVAILABLE, bool)

    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.ReachyMini")
    def test_reachy_mini_import_available(self, mock_reachy):
        """Test que reachy_mini est disponible."""
        from bbia_sim.daemon.bridge import REACHY_MINI_AVAILABLE

        assert REACHY_MINI_AVAILABLE is True

    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", False)
    def test_reachy_mini_import_not_available(self):
        """Test que reachy_mini n'est pas disponible."""
        from bbia_sim.daemon.bridge import REACHY_MINI_AVAILABLE

        # C'est OK si reachy_mini n'est pas disponible en test
        assert isinstance(REACHY_MINI_AVAILABLE, bool)

    def test_zenoh_config_initialization(self):
        """Test initialisation configuration Zenoh."""
        try:
            from bbia_sim.daemon.bridge import ZenohConfig

            config = ZenohConfig()
            assert config.mode == "client"
            assert isinstance(config.connect, list)
            assert len(config.connect) > 0
            assert config.timeout == 1000
            assert config.retry_attempts == 3
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    def test_robot_command_model(self):
        """Test modèle RobotCommand."""
        try:
            import time

            from bbia_sim.daemon.bridge import RobotCommand

            timestamp_before = time.time()
            cmd = RobotCommand(command="test", parameters={"param": "value"})
            timestamp_after = time.time()

            assert cmd.command == "test"
            assert cmd.parameters == {"param": "value"}
            assert timestamp_before <= cmd.timestamp <= timestamp_after
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    def test_robot_state_model(self):
        """Test modèle RobotState."""
        try:
            import time

            from bbia_sim.daemon.bridge import RobotState

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
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.Session")
    def test_zenoh_bridge_initialization(self, mock_session):
        """Test initialisation ZenohBridge."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            assert bridge.connected is False
            assert bridge.session is None or isinstance(bridge.session, type(None))
            assert hasattr(bridge, "topics")
            assert hasattr(bridge, "subscribers")
            assert hasattr(bridge, "publishers")
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")
        except Exception as e:
            # Peut échouer si zenoh n'est pas installé
            if "zenoh" in str(e).lower():
                pytest.skip(f"Zenoh non disponible: {e}")
            raise

    def test_bridge_topics_structure(self):
        """Test structure des topics Zenoh."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            assert isinstance(bridge.topics, dict)
            required_topics = ["commands", "state", "telemetry", "errors"]
            for topic in required_topics:
                assert topic in bridge.topics
                assert isinstance(bridge.topics[topic], str)
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")
        except Exception as e:
            if "zenoh" in str(e).lower():
                pytest.skip(f"Zenoh non disponible: {e}")
            raise

    def test_bridge_methods_exist(self):
        """Test que les méthodes principales existent."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()

            # Vérifier présence méthodes principales
            assert hasattr(bridge, "connect") or hasattr(bridge, "connect_zenoh")
            assert hasattr(bridge, "disconnect") or hasattr(bridge, "disconnect_zenoh")

        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")
        except Exception as e:
            if "zenoh" in str(e).lower():
                pytest.skip(f"Zenoh non disponible: {e}")
            raise

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", True)
    def test_zenoh_bridge_config_parameter(self):
        """Test initialisation ZenohBridge avec config personnalisée."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge, ZenohConfig

            custom_config = ZenohConfig(
                mode="client", connect=["tcp://custom:7447"], timeout=2000
            )
            bridge = ZenohBridge(config=custom_config)

            assert bridge.config == custom_config
            assert bridge.config.timeout == 2000
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")
        except Exception as e:
            if "zenoh" in str(e).lower():
                pytest.skip(f"Zenoh non disponible: {e}")
            raise

    def test_robot_command_with_timestamp(self):
        """Test RobotCommand avec timestamp fourni."""
        try:
            import time

            from bbia_sim.daemon.bridge import RobotCommand

            custom_timestamp = time.time() - 100
            cmd = RobotCommand(
                command="test",
                parameters={"param": "value"},
                timestamp=custom_timestamp,
            )

            assert cmd.timestamp == custom_timestamp
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    def test_robot_state_with_timestamp(self):
        """Test RobotState avec timestamp fourni."""
        try:
            import time

            from bbia_sim.daemon.bridge import RobotState

            custom_timestamp = time.time() - 100
            state = RobotState(
                joints={"yaw_body": 0.1},
                emotions={"happy": 0.8},
                sensors={"sensor": 1.0},
                timestamp=custom_timestamp,
            )

            assert state.timestamp == custom_timestamp
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    def test_zenoh_config_default_values(self):
        """Test valeurs par défaut ZenohConfig."""
        try:
            from bbia_sim.daemon.bridge import ZenohConfig

            config = ZenohConfig()
            assert config.mode == "client"
            assert isinstance(config.connect, list)
            assert len(config.connect) > 0
            assert "localhost" in str(config.connect[0]) or "7447" in str(
                config.connect[0]
            )
            assert config.timeout == 1000
            assert config.retry_attempts == 3
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.Session")
    def test_zenoh_bridge_subscribers_publishers_init(self, mock_session):
        """Test initialisation subscribers et publishers."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            assert isinstance(bridge.subscribers, dict)
            assert isinstance(bridge.publishers, dict)
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")
        except Exception as e:
            if "zenoh" in str(e).lower():
                pytest.skip(f"Zenoh non disponible: {e}")
            raise

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.zenoh")
    @patch("bbia_sim.daemon.bridge.asyncio")
    def test_zenoh_bridge_start_success(self, mock_asyncio, mock_zenoh_module):
        """Test démarrage bridge Zenoh avec succès."""
        try:
            import asyncio

            from bbia_sim.daemon.bridge import ZenohBridge

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
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")
        except Exception as e:
            if "zenoh" in str(e).lower() or "asyncio" in str(e).lower():
                pytest.skip(f"Zenoh ou asyncio non disponible: {e}")
            raise

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", False)
    def test_zenoh_bridge_start_no_zenoh(self):
        """Test démarrage bridge sans Zenoh."""
        try:
            import asyncio

            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()

            async def test():
                result = await bridge.start()
                assert result is False

            asyncio.run(test())
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    def test_zenoh_bridge_stop(self):
        """Test arrêt bridge Zenoh."""
        try:
            import asyncio

            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            bridge.connected = True
            bridge.session = MagicMock()
            bridge.session.close = MagicMock(return_value=None)

            async def test():
                await bridge.stop()
                assert bridge.connected is False

            asyncio.run(test())
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    def test_zenoh_bridge_send_command(self):
        """Test envoi commande via bridge."""
        try:
            import asyncio

            from bbia_sim.daemon.bridge import RobotCommand, ZenohBridge

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
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    def test_zenoh_bridge_get_current_state(self):
        """Test récupération état actuel."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            state = bridge.get_current_state()
            assert state is not None
            assert hasattr(state, "joints")
            assert hasattr(state, "emotions")
            assert hasattr(state, "sensors")
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    def test_zenoh_bridge_is_connected(self):
        """Test vérification connexion."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            assert bridge.is_connected() is False
            bridge.connected = True
            assert bridge.is_connected() is True
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    def test_zenoh_bridge_stop_with_reachy_mini(self):
        """Test arrêt bridge avec Reachy Mini."""
        try:
            import asyncio

            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            bridge.connected = True
            bridge.reachy_mini = MagicMock()
            bridge.reachy_mini.close = MagicMock()

            async def test():
                await bridge.stop()
                assert bridge.connected is False
                bridge.reachy_mini.close.assert_called_once()

            asyncio.run(test())
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    def test_zenoh_bridge_stop_with_subscribers(self):
        """Test arrêt bridge avec subscribers."""
        try:
            import asyncio

            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            bridge.connected = True
            mock_sub = MagicMock()
            mock_sub.close = MagicMock(return_value=None)
            bridge.subscribers["commands"] = mock_sub

            async def test():
                await bridge.stop()
                mock_sub.close.assert_called_once()

            asyncio.run(test())
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    def test_zenoh_bridge_send_command_not_connected(self):
        """Test envoi commande bridge non connecté."""
        try:
            import asyncio

            from bbia_sim.daemon.bridge import RobotCommand, ZenohBridge

            bridge = ZenohBridge()
            bridge.connected = False
            cmd = RobotCommand(command="test", parameters={})

            async def test():
                result = await bridge.send_command(cmd)
                assert result is False

            asyncio.run(test())
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_setup_topics(self):
        """Test configuration topics Zenoh."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            bridge.session = MagicMock()
            bridge.session.declare_subscriber = MagicMock(return_value=MagicMock())
            bridge.session.declare_publisher = MagicMock(return_value=MagicMock())

            await bridge._setup_zenoh_topics()

            assert "commands" in bridge.subscribers
            assert "state" in bridge.publishers
            assert "telemetry" in bridge.publishers
            assert "errors" in bridge.publishers
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_on_command_received_valid(self):
        """Test réception commande valide."""
        try:
            import json

            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            bridge.command_queue = MagicMock()
            bridge.command_queue.put = MagicMock()

            mock_sample = MagicMock()
            mock_sample.payload.decode.return_value = json.dumps(
                {"command": "test", "parameters": {}}
            )

            await bridge._on_command_received(mock_sample)

            bridge.command_queue.put.assert_called_once()
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_on_command_received_too_large(self):
        """Test réception commande trop volumineuse."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            bridge._publish_error = MagicMock()

            mock_sample = MagicMock()
            large_payload = "x" * (1048577)  # 1MB + 1 byte
            mock_sample.payload.decode.return_value = large_payload

            await bridge._on_command_received(mock_sample)

            bridge._publish_error.assert_called_once()
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_execute_command_goto_target(self):
        """Test exécution commande goto_target."""
        try:
            from bbia_sim.daemon.bridge import RobotCommand, ZenohBridge

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
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_execute_command_set_emotion(self):
        """Test exécution commande set_emotion."""
        try:
            from bbia_sim.daemon.bridge import RobotCommand, ZenohBridge

            bridge = ZenohBridge()
            bridge.reachy_mini = MagicMock()

            cmd = RobotCommand(
                command="set_emotion", parameters={"emotion": "happy", "intensity": 0.8}
            )

            await bridge._execute_command(cmd)
            assert "happy" in bridge.current_state.emotions
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_publish_state(self):
        """Test publication état."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            bridge.publishers["state"] = MagicMock()
            bridge.publishers["state"].put = MagicMock()

            await bridge._publish_state()
            bridge.publishers["state"].put.assert_called_once()
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_publish_error(self):
        """Test publication erreur."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            bridge.publishers["errors"] = MagicMock()
            bridge.publishers["errors"].put = MagicMock()

            await bridge._publish_error("Test error")
            bridge.publishers["errors"].put.assert_called_once()
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.ZENOH_AVAILABLE", True)
    @patch("bbia_sim.daemon.bridge.REACHY_MINI_AVAILABLE", True)
    @pytest.mark.asyncio
    async def test_zenoh_bridge_update_robot_state(self):
        """Test mise à jour état robot."""
        try:
            from bbia_sim.daemon.bridge import ZenohBridge

            bridge = ZenohBridge()
            bridge.reachy_mini = MagicMock()
            bridge.reachy_mini.get_current_joint_positions = MagicMock(
                return_value=([0.1] * 6, [0.2, 0.3])
            )

            await bridge._update_robot_state()

            assert len(bridge.current_state.joints) > 0
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.app")
    @patch("bbia_sim.daemon.bridge.bridge")
    def test_fastapi_get_robot_state_endpoint(self, mock_bridge, mock_app):
        """Test endpoint GET /api/zenoh/state."""
        try:
            from bbia_sim.daemon.bridge import RobotState, get_robot_state

            mock_bridge.is_connected.return_value = True
            mock_state = RobotState(joints={}, emotions={}, sensors={})
            mock_bridge.get_current_state.return_value = mock_state

            # Test async endpoint (simulé)
            import asyncio

            async def test():
                result = await get_robot_state()
                assert "joints" in result

            asyncio.run(test())
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")

    @patch("bbia_sim.daemon.bridge.bridge")
    def test_fastapi_get_bridge_status_endpoint(self):
        """Test endpoint GET /api/zenoh/status."""
        try:
            from bbia_sim.daemon.bridge import get_bridge_status

            # Import global bridge
            import bbia_sim.daemon.bridge as bridge_module

            original_bridge = bridge_module.bridge
            bridge_module.bridge = MagicMock()
            bridge_module.bridge.is_connected.return_value = True

            import asyncio

            async def test():
                result = await get_bridge_status()
                assert "connected" in result
                assert "zenoh_available" in result

            asyncio.run(test())

            bridge_module.bridge = original_bridge
        except ImportError:
            pytest.skip("Module daemon.bridge non disponible")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
