#!/usr/bin/env python3
"""Tests pour intégration RobotFactory avec RobotRegistry."""

from unittest.mock import MagicMock, patch

from bbia_sim.robot_factory import RobotFactory


class TestRobotFactoryRegistryIntegration:
    """Tests pour intégration RobotFactory avec RobotRegistry."""

    def test_auto_mode_uses_registry_for_discovery(self):
        """Test que mode 'auto' utilise RobotRegistry pour découverte."""
        with patch("bbia_sim.robot_registry.RobotRegistry") as mock_registry_class:
            mock_registry = MagicMock()
            mock_registry.discover_robots.return_value = [
                {
                    "id": "robot-1",
                    "hostname": "192.168.1.100",
                    "port": 8080,
                    "status": "available",
                }
            ]
            mock_registry_class.return_value = mock_registry

            with patch("bbia_sim.robot_factory.ReachyMiniBackend") as mock_backend:
                mock_instance = MagicMock()
                mock_instance.is_connected = False
                mock_backend.return_value = mock_instance

                # Mode auto devrait utiliser registry
                RobotFactory.create_backend("auto")

                # Vérifier que registry a été utilisé
                mock_registry.discover_robots.assert_called_once()

    def test_auto_mode_fallback_if_registry_fails(self):
        """Test que mode 'auto' fallback si registry échoue."""
        with patch("bbia_sim.robot_registry.RobotRegistry") as mock_registry_class:
            mock_registry_class.side_effect = Exception("Registry error")

            with patch("bbia_sim.robot_factory.ReachyMiniBackend") as mock_backend:
                mock_instance = MagicMock()
                mock_instance.is_connected = False
                mock_backend.return_value = mock_instance

                with patch("bbia_sim.robot_factory.MuJoCoBackend") as mock_sim:
                    mock_sim_instance = MagicMock()
                    mock_sim.return_value = mock_sim_instance

                    # Mode auto devrait fallback vers sim si registry échoue
                    backend = RobotFactory.create_backend("auto")

                    # Devrait retourner sim
                    assert backend is not None
                    mock_sim.assert_called_once()

    def test_auto_mode_uses_discovered_robot_info(self):
        """Test que mode 'auto' utilise infos robot découvert."""
        with patch("bbia_sim.robot_registry.RobotRegistry") as mock_registry_class:
            mock_registry = MagicMock()
            mock_registry.discover_robots.return_value = [
                {
                    "id": "robot-1",
                    "hostname": "192.168.1.100",
                    "port": 8080,
                    "status": "available",
                }
            ]
            mock_registry_class.return_value = mock_registry

            with patch("bbia_sim.robot_factory.ReachyMiniBackend") as mock_backend:
                mock_instance = MagicMock()
                mock_instance.is_connected = True
                mock_instance.robot = MagicMock()  # Robot réel
                mock_backend.return_value = mock_instance

                # Mode auto devrait utiliser infos robot découvert
                RobotFactory.create_backend("auto")

                # Vérifier que backend a été créé avec use_sim=False
                mock_backend.assert_called()
                call_kwargs = (
                    mock_backend.call_args[1] if mock_backend.call_args else {}
                )
                assert call_kwargs.get("use_sim") is False
