#!/usr/bin/env python3
"""Tests pour fallback automatique sim → robot."""

from unittest.mock import MagicMock, patch

from bbia_sim.robot_factory import RobotFactory


class TestAutoFallback:
    """Tests pour mode 'auto' avec fallback automatique."""

    def test_auto_defaults_to_mujoco_without_hardware_flag(self, monkeypatch):
        """Sans BBIA_ALLOW_HARDWARE, auto → MuJoCo (pas de tentative robot)."""
        monkeypatch.delenv("BBIA_ALLOW_HARDWARE", raising=False)
        with patch("bbia_sim.robot_factory.ReachyMiniBackend") as mock_rm:
            with patch("bbia_sim.robot_factory.MuJoCoBackend") as mock_sim:
                mock_sim.return_value = MagicMock(name="mujoco")
                backend = RobotFactory.create_backend("auto")
                assert backend is not None
                mock_rm.assert_not_called()
                mock_sim.assert_called_once()

    def test_auto_detects_robot_when_available(self, monkeypatch):
        """Test détection robot si disponible (flag hardware requis)."""
        monkeypatch.setenv("BBIA_ALLOW_HARDWARE", "1")
        with patch("bbia_sim.robot_factory.ReachyMiniBackend") as mock_backend_class:
            mock_instance = MagicMock()
            mock_instance.is_connected = True
            mock_instance.robot = MagicMock()  # Robot réel présent
            mock_backend_class.return_value = mock_instance

            backend = RobotFactory.create_backend("auto")

            assert backend is not None
            assert backend.is_connected is True
            assert backend.robot is not None
            mock_backend_class.assert_called_once()

    def test_auto_fallback_to_sim_when_robot_unavailable(self, monkeypatch):
        """Test fallback sim si robot absent (avec flag hardware)."""
        monkeypatch.setenv("BBIA_ALLOW_HARDWARE", "1")
        with patch(
            "bbia_sim.robot_factory.ReachyMiniBackend",
            side_effect=Exception("No robot"),
        ):
            with patch("bbia_sim.robot_factory.MuJoCoBackend") as mock_sim:
                mock_instance = MagicMock()
                mock_sim.return_value = mock_instance

                backend = RobotFactory.create_backend("auto")

                assert backend is not None
                assert isinstance(backend, type(mock_instance))
                mock_sim.assert_called_once()

    def test_auto_handles_connection_errors(self, monkeypatch):
        """Test gestion erreurs connexion."""
        monkeypatch.setenv("BBIA_ALLOW_HARDWARE", "1")
        with patch("bbia_sim.robot_factory.ReachyMiniBackend") as mock_backend_class:
            mock_instance = MagicMock()
            mock_instance.is_connected = True
            mock_instance.robot = None  # Robot non connecté (mode sim)
            mock_backend_class.return_value = mock_instance

            with patch("bbia_sim.robot_factory.MuJoCoBackend") as mock_sim:
                mock_sim_instance = MagicMock()
                mock_sim.return_value = mock_sim_instance

                backend = RobotFactory.create_backend("auto")

                # Devrait fallback vers sim car robot est None (mode sim)
                assert backend is not None
                mock_sim.assert_called_once()

    def test_auto_preserves_kwargs(self, monkeypatch):
        """Test préservation kwargs lors fallback."""
        monkeypatch.setenv("BBIA_ALLOW_HARDWARE", "1")
        with patch(
            "bbia_sim.robot_factory.ReachyMiniBackend",
            side_effect=Exception("Connection failed"),
        ):
            with patch("bbia_sim.robot_factory.MuJoCoBackend") as mock_sim:
                mock_instance = MagicMock()
                mock_sim.return_value = mock_instance

                backend = RobotFactory.create_backend("auto", fast=True)

                assert backend is not None
                # Vérifier que kwargs sont passés (fast=True devient model_path dans MuJoCoBackend)
                # Le kwargs fast=True est bien passé, mais transformé en model_path
                assert mock_sim.call_count == 1
                call_kwargs = mock_sim.call_args[1] if mock_sim.call_args else {}
                assert "fast" in call_kwargs or "model_path" in call_kwargs

    def test_auto_case_insensitive(self, monkeypatch):
        """Test mode auto insensible à la casse."""
        monkeypatch.delenv("BBIA_ALLOW_HARDWARE", raising=False)
        with patch(
            "bbia_sim.robot_factory.ReachyMiniBackend",
            side_effect=Exception("No robot"),
        ):
            with patch("bbia_sim.robot_factory.MuJoCoBackend") as mock_sim:
                mock_instance = MagicMock()
                mock_sim.return_value = mock_instance

                backend1 = RobotFactory.create_backend("AUTO")
                backend2 = RobotFactory.create_backend("Auto")
                backend3 = RobotFactory.create_backend("aUtO")

                assert backend1 is not None
                assert backend2 is not None
                assert backend3 is not None
                assert mock_sim.call_count == 3

    def test_auto_in_available_backends(self):
        """Test que 'auto' est dans la liste des backends disponibles."""
        backends = RobotFactory.get_available_backends()
        assert "auto" in backends

    def test_auto_backend_info(self):
        """Test informations backend 'auto'."""
        info = RobotFactory.get_backend_info("auto")
        assert isinstance(info, dict)
        assert info["name"] == "Auto-Détection"
        assert "BBIA_ALLOW_HARDWARE" in info["description"]
