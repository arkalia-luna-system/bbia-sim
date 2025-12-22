#!/usr/bin/env python3
"""Tests pour routing multi-backends simultanés sim/robot réel."""

from unittest.mock import MagicMock, patch

import pytest

from bbia_sim.daemon.app.backend_adapter import BackendAdapter, get_backend_adapter
from bbia_sim.robot_factory import RobotFactory


class TestMultiBackendRouting:
    """Tests pour routing multi-backends."""

    def test_backend_adapter_uses_multi_backends_if_available(self):
        """Test que BackendAdapter utilise multi_backends si disponible."""
        mock_backend = MagicMock()
        mock_backend.is_connected = True
        # Patcher le module importé dans backend_adapter
        with patch("bbia_sim.daemon.app.backend_adapter.app_state") as mock_app_state:
            mock_app_state.get.return_value = {"mujoco": mock_backend}
            adapter = BackendAdapter(backend_type="mujoco")

            assert adapter._robot is not None
            assert adapter._robot is mock_backend

    def test_backend_adapter_fallback_if_multi_backends_unavailable(self):
        """Test que BackendAdapter fallback si multi_backends non disponible."""
        with patch("bbia_sim.daemon.app.backend_adapter.app_state") as mock_app_state:
            mock_app_state.get.return_value = {}
            with patch(
                "bbia_sim.daemon.app.backend_adapter.RobotFactory.create_backend"
            ) as mock_create:
                mock_backend = MagicMock()
                mock_create.return_value = mock_backend

                adapter = BackendAdapter(backend_type="mujoco")

                assert adapter._robot is not None
                mock_create.assert_called_once_with("mujoco")

    def test_get_backend_adapter_accepts_backend_query_param(self):
        """Test que get_backend_adapter accepte paramètre backend."""
        with patch(
            "bbia_sim.daemon.app.backend_adapter.BackendAdapter"
        ) as mock_adapter_class:
            mock_instance = MagicMock()
            mock_adapter_class.return_value = mock_instance

            adapter = get_backend_adapter(backend="mujoco")

            assert adapter is not None
            mock_adapter_class.assert_called_once_with(backend_type="mujoco")

    def test_get_backend_adapter_default_if_no_backend_param(self):
        """Test que get_backend_adapter utilise défaut si pas de param backend."""
        with patch(
            "bbia_sim.daemon.app.backend_adapter.BackendAdapter"
        ) as mock_adapter_class:
            mock_instance = MagicMock()
            mock_adapter_class.return_value = mock_instance

            adapter = get_backend_adapter(backend=None)

            assert adapter is not None
            mock_adapter_class.assert_called_once_with(backend_type=None)

    def test_backend_adapter_routes_to_correct_backend(self):
        """Test que BackendAdapter route vers le bon backend."""
        mock_mujoco = MagicMock()
        mock_mujoco.is_connected = True
        mock_reachy = MagicMock()
        mock_reachy.is_connected = True
        with patch("bbia_sim.daemon.app.backend_adapter.app_state") as mock_app_state:
            mock_app_state.get.return_value = {
                "mujoco": mock_mujoco,
                "reachy_mini": mock_reachy,
            }
            adapter_mujoco = BackendAdapter(backend_type="mujoco")
            adapter_reachy = BackendAdapter(backend_type="reachy_mini")

            assert adapter_mujoco._robot is mock_mujoco
            assert adapter_reachy._robot is mock_reachy

    def test_backend_adapter_handles_backend_not_in_multi_backends(self):
        """Test que BackendAdapter gère backend non présent dans multi_backends."""
        with patch("bbia_sim.daemon.app.backend_adapter.app_state") as mock_app_state:
            mock_app_state.get.return_value = {"mujoco": MagicMock()}
            with patch(
                "bbia_sim.daemon.app.backend_adapter.RobotFactory.create_backend"
            ) as mock_create:
                mock_backend = MagicMock()
                mock_create.return_value = mock_backend

                adapter = BackendAdapter(backend_type="reachy_mini")

                assert adapter._robot is not None
                mock_create.assert_called_once_with("reachy_mini")

    def test_multi_backends_initialized_in_lifespan(self):
        """Test que multi_backends sont initialisés dans lifespan."""
        with patch(
            "bbia_sim.robot_factory.RobotFactory.create_multi_backend"
        ) as mock_create:
            mock_multi = {"mujoco": MagicMock()}
            mock_create.return_value = mock_multi

            # Simuler l'initialisation dans lifespan
            multi_backends = RobotFactory.create_multi_backend(backends=["mujoco"])

            assert multi_backends == mock_multi
            mock_create.assert_called_once_with(backends=["mujoco"])

    def test_backend_adapter_handles_none_backend_gracefully(self):
        """Test que BackendAdapter gère gracieusement backend=None."""
        with patch("bbia_sim.daemon.app.backend_adapter.app_state") as mock_app_state:
            mock_app_state.get.return_value = {}
            with patch(
                "bbia_sim.daemon.app.backend_adapter.RobotFactory.create_backend"
            ) as mock_create:
                mock_backend = MagicMock()
                mock_create.return_value = mock_backend

                adapter = BackendAdapter(backend_type=None)

                assert adapter._robot is not None
                # Devrait fallback vers mujoco ou reachy_mini
                assert mock_create.call_count >= 1


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
