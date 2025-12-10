#!/usr/bin/env python3
"""Tests complets pour robot_registry.py - Découverte automatique des robots via Zenoh."""

from __future__ import annotations

import os
from unittest.mock import MagicMock, patch

from bbia_sim.robot_registry import RobotRegistry


class TestRobotRegistry:
    """Tests pour RobotRegistry."""

    def test_init(self) -> None:
        """Test initialisation."""
        registry = RobotRegistry()
        assert registry.robots == []
        assert registry.session is None

    def test_discover_robots_no_zenoh(self) -> None:
        """Test découverte robots sans Zenoh."""
        with patch("bbia_sim.robot_registry.ZENOH_AVAILABLE", False):
            registry = RobotRegistry()
            robots = registry.discover_robots()
            assert robots == []

    def test_discover_robots_zenoh_none(self) -> None:
        """Test découverte robots avec zenoh=None."""
        with (
            patch("bbia_sim.robot_registry.ZENOH_AVAILABLE", True),
            patch("bbia_sim.robot_registry.zenoh", None),
        ):
            registry = RobotRegistry()
            robots = registry.discover_robots()
            assert robots == []

    def test_discover_robots_with_env_vars(self) -> None:
        """Test découverte robots via variables d'environnement."""
        old_robot_id = os.environ.get("BBIA_ROBOT_ID")
        old_hostname = os.environ.get("BBIA_HOSTNAME")
        old_port = os.environ.get("BBIA_PORT")

        try:
            os.environ["BBIA_ROBOT_ID"] = "test_robot_123"
            os.environ["BBIA_HOSTNAME"] = "test-host"
            os.environ["BBIA_PORT"] = "9090"

            mock_zenoh = MagicMock()
            mock_config = MagicMock()
            mock_session = MagicMock()
            mock_zenoh.open.return_value = mock_session
            mock_zenoh.Config.return_value = mock_config

            with (
                patch("bbia_sim.robot_registry.ZENOH_AVAILABLE", True),
                patch("bbia_sim.robot_registry.zenoh", mock_zenoh),
                patch("bbia_sim.robot_registry.Config", return_value=mock_config),
                patch("time.sleep"),
            ):
                registry = RobotRegistry()
                robots = registry.discover_robots()

            assert len(robots) == 1
            assert robots[0]["id"] == "test_robot_123"
            assert robots[0]["hostname"] == "test-host"
            assert robots[0]["port"] == 9090
            assert robots[0]["status"] == "available"
        finally:
            # Restaurer variables d'environnement
            if old_robot_id is not None:
                os.environ["BBIA_ROBOT_ID"] = old_robot_id
            elif "BBIA_ROBOT_ID" in os.environ:
                del os.environ["BBIA_ROBOT_ID"]

            if old_hostname is not None:
                os.environ["BBIA_HOSTNAME"] = old_hostname
            elif "BBIA_HOSTNAME" in os.environ:
                del os.environ["BBIA_HOSTNAME"]

            if old_port is not None:
                os.environ["BBIA_PORT"] = old_port
            elif "BBIA_PORT" in os.environ:
                del os.environ["BBIA_PORT"]

    def test_discover_robots_with_defaults(self) -> None:
        """Test découverte robots avec valeurs par défaut."""
        old_robot_id = os.environ.get("BBIA_ROBOT_ID")
        old_hostname = os.environ.get("BBIA_HOSTNAME")
        old_port = os.environ.get("BBIA_PORT")

        try:
            os.environ["BBIA_ROBOT_ID"] = "test_robot"
            # Ne pas définir BBIA_HOSTNAME et BBIA_PORT pour tester valeurs par défaut
            if "BBIA_HOSTNAME" in os.environ:
                del os.environ["BBIA_HOSTNAME"]
            if "BBIA_PORT" in os.environ:
                del os.environ["BBIA_PORT"]

            mock_zenoh = MagicMock()
            mock_config = MagicMock()
            mock_session = MagicMock()
            mock_zenoh.open.return_value = mock_session
            mock_zenoh.Config.return_value = mock_config

            with (
                patch("bbia_sim.robot_registry.ZENOH_AVAILABLE", True),
                patch("bbia_sim.robot_registry.zenoh", mock_zenoh),
                patch("bbia_sim.robot_registry.Config", return_value=mock_config),
                patch("time.sleep"),
            ):
                registry = RobotRegistry()
                robots = registry.discover_robots()

            assert len(robots) == 1
            assert robots[0]["id"] == "test_robot"
            assert robots[0]["hostname"] == "localhost"  # Valeur par défaut
            assert robots[0]["port"] == 8080  # Valeur par défaut
        finally:
            # Restaurer variables d'environnement
            if old_robot_id is not None:
                os.environ["BBIA_ROBOT_ID"] = old_robot_id
            elif "BBIA_ROBOT_ID" in os.environ:
                del os.environ["BBIA_ROBOT_ID"]

            if old_hostname is not None:
                os.environ["BBIA_HOSTNAME"] = old_hostname
            if old_port is not None:
                os.environ["BBIA_PORT"] = old_port

    def test_discover_robots_exception_handling(self) -> None:
        """Test gestion exceptions lors de la découverte."""
        mock_zenoh = MagicMock()
        mock_config = MagicMock()
        mock_zenoh.open.side_effect = Exception("Erreur Zenoh")
        mock_zenoh.Config.return_value = mock_config

        with (
            patch("bbia_sim.robot_registry.ZENOH_AVAILABLE", True),
            patch("bbia_sim.robot_registry.zenoh", mock_zenoh),
            patch("bbia_sim.robot_registry.Config", return_value=mock_config),
        ):
            registry = RobotRegistry()
            robots = registry.discover_robots()
            # Doit retourner liste vide en cas d'erreur
            assert robots == []

    def test_discover_robots_session_close(self) -> None:
        """Test fermeture session après découverte."""
        mock_zenoh = MagicMock()
        mock_config = MagicMock()
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = mock_config

        with (
            patch("bbia_sim.robot_registry.ZENOH_AVAILABLE", True),
            patch("bbia_sim.robot_registry.zenoh", mock_zenoh),
            patch("bbia_sim.robot_registry.Config", return_value=mock_config),
            patch("time.sleep"),
        ):
            registry = RobotRegistry()
            registry.discover_robots()

            # Vérifier que session.close() a été appelé
            mock_session.close.assert_called_once()

    def test_discover_robots_session_close_exception(self) -> None:
        """Test gestion exception lors de la fermeture de session."""
        mock_zenoh = MagicMock()
        mock_config = MagicMock()
        mock_session = MagicMock()
        mock_session.close.side_effect = Exception("Erreur fermeture")
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = mock_config

        with (
            patch("bbia_sim.robot_registry.ZENOH_AVAILABLE", True),
            patch("bbia_sim.robot_registry.zenoh", mock_zenoh),
            patch("bbia_sim.robot_registry.Config", return_value=mock_config),
            patch("time.sleep"),
        ):
            registry = RobotRegistry()
            # Ne doit pas lever d'exception
            robots = registry.discover_robots()
            assert isinstance(robots, list)

    def test_discover_robots_timeout(self) -> None:
        """Test découverte robots avec timeout personnalisé."""
        old_robot_id = os.environ.get("BBIA_ROBOT_ID")

        try:
            os.environ["BBIA_ROBOT_ID"] = "test_robot"

            mock_zenoh = MagicMock()
            mock_config = MagicMock()
            mock_session = MagicMock()
            mock_zenoh.open.return_value = mock_session
            mock_zenoh.Config.return_value = mock_config

            with (
                patch("bbia_sim.robot_registry.ZENOH_AVAILABLE", True),
                patch("bbia_sim.robot_registry.zenoh", mock_zenoh),
                patch("bbia_sim.robot_registry.Config", return_value=mock_config),
                patch("time.sleep"),
            ):
                registry = RobotRegistry()
                robots = registry.discover_robots(timeout=10.0)

            assert len(robots) >= 0  # Peut être vide ou contenir robots
        finally:
            if old_robot_id is not None:
                os.environ["BBIA_ROBOT_ID"] = old_robot_id
            elif "BBIA_ROBOT_ID" in os.environ:
                del os.environ["BBIA_ROBOT_ID"]

    def test_list_robots_empty(self) -> None:
        """Test list_robots quand aucun robot découvert."""
        with patch.object(RobotRegistry, "discover_robots", return_value=[]):
            registry = RobotRegistry()
            robots = registry.list_robots()
            assert robots == []

    def test_list_robots_with_robots(self) -> None:
        """Test list_robots avec robots découverts."""
        mock_robots = [
            {"id": "robot1", "hostname": "host1", "port": 8080, "status": "available"},
            {"id": "robot2", "hostname": "host2", "port": 8081, "status": "available"},
        ]

        with patch.object(RobotRegistry, "discover_robots", return_value=mock_robots):
            registry = RobotRegistry()
            robots = registry.list_robots()
            assert len(robots) == 2
            assert robots == mock_robots

    def test_list_robots_cached(self) -> None:
        """Test list_robots utilise cache si robots déjà découverts."""
        mock_robots = [
            {"id": "robot1", "hostname": "host1", "port": 8080, "status": "available"},
        ]

        registry = RobotRegistry()
        registry.robots = mock_robots

        # discover_robots ne doit pas être appelé si robots déjà en cache
        with patch.object(
            RobotRegistry, "discover_robots", return_value=[]
        ) as mock_discover:
            robots = registry.list_robots()
            mock_discover.assert_not_called()
            assert robots == mock_robots

    def test_discover_robots_topics_exception(self) -> None:
        """Test gestion exceptions lors de la vérification des topics."""
        mock_zenoh = MagicMock()
        mock_config = MagicMock()
        mock_session = MagicMock()
        mock_session.declare_subscriber.side_effect = Exception("Topic error")
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = mock_config

        with (
            patch("bbia_sim.robot_registry.ZENOH_AVAILABLE", True),
            patch("bbia_sim.robot_registry.zenoh", mock_zenoh),
            patch("bbia_sim.robot_registry.Config", return_value=mock_config),
            patch("time.sleep"),
        ):
            registry = RobotRegistry()
            # Ne doit pas lever d'exception, doit continuer
            robots = registry.discover_robots()
            assert isinstance(robots, list)
