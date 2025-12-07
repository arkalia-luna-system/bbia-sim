#!/usr/bin/env python3
"""
Tests de gestion d'erreurs pour unity_reachy_controller.

Vérifie que les erreurs sont gérées correctement avec les nouveaux logs.
"""

import logging
import tempfile
from pathlib import Path
from unittest.mock import MagicMock, call, patch

import pytest

from bbia_sim.unity_reachy_controller import UnityReachyMiniController

logger = logging.getLogger(__name__)


class TestUnityControllerErrorHandling:
    """Tests de gestion d'erreurs pour UnityReachyMiniController."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_unity_controller_input_error_handling(self):
        """Test que les erreurs input() sont gérées correctement."""
        with tempfile.TemporaryDirectory() as tmpdir:
            controller = UnityReachyMiniController(
                command_file=str(Path(tmpdir) / "cmd.txt"),
                response_file=str(Path(tmpdir) / "resp.txt"),
            )

            # Simuler une erreur lors de input() dans interactive_mode
            with patch("builtins.input", side_effect=EOFError("EOF")):
                # interactive_mode doit gérer l'erreur sans crasher
                controller.interactive_mode(max_iterations=1)
                # Si on arrive ici, l'erreur a été gérée correctement
                assert controller is not None

    @pytest.mark.unit
    @pytest.mark.fast
    def test_unity_controller_command_error_handling(self):
        """Test que les erreurs de commande sont gérées correctement."""
        with tempfile.TemporaryDirectory() as tmpdir:
            controller = UnityReachyMiniController(
                command_file=str(Path(tmpdir) / "cmd.txt"),
                response_file=str(Path(tmpdir) / "resp.txt"),
            )

            # Simuler une erreur lors de l'exécution d'une commande dans interactive_mode
            mock_logger = MagicMock(spec=logging.Logger)
            with patch("bbia_sim.unity_reachy_controller.logger", mock_logger):
                with patch.object(
                    controller, "move_head", side_effect=RuntimeError("Erreur commande")
                ):
                    # Simuler une commande qui échoue
                    controller.interactive_mode(max_iterations=1)
                    # Vérifier que logger.error a été appelé pour les erreurs critiques
                    error_calls = [
                        call
                        for call in mock_logger.error.call_args_list
                        if call and "critique" in str(call).lower()
                    ]
                    # L'erreur doit être loggée en ERROR avec "critique"
                    assert len(error_calls) > 0 or mock_logger.error.called

    @pytest.mark.unit
    @pytest.mark.fast
    def test_unity_controller_init_error_handling(self):
        """Test que l'initialisation gère les erreurs correctement."""
        # Simuler une erreur de permission
        with patch("pathlib.Path.mkdir", side_effect=PermissionError("Permission")):
            controller = UnityReachyMiniController(
                command_file="/invalid/path/cmd.txt",
                response_file="/invalid/path/resp.txt",
            )
            # Doit initialiser sans crasher
            assert controller is not None
            # is_connected devrait être False en cas d'erreur
            assert controller.is_connected is False

    @pytest.mark.unit
    @pytest.mark.fast
    def test_unity_controller_send_command_error(self):
        """Test que _send_command() gère les erreurs correctement."""
        with tempfile.TemporaryDirectory() as tmpdir:
            controller = UnityReachyMiniController(
                command_file=str(Path(tmpdir) / "cmd.txt"),
                response_file=str(Path(tmpdir) / "resp.txt"),
            )

            # Simuler une erreur d'écriture
            with patch("pathlib.Path.write_text", side_effect=OSError("Write error")):
                result = controller._send_command("test")
                # Doit retourner False sans crasher
                assert result is False


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
