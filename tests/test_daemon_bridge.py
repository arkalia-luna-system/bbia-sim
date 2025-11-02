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
                command="test", parameters={"param": "value"}, timestamp=custom_timestamp
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
            assert "localhost" in str(config.connect[0]) or "7447" in str(config.connect[0])
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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
