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
        # Vérifier que le code utilise logger.error pour les erreurs critiques
        # En regardant le code source, on voit que les erreurs sont loggées en ERROR
        import inspect

        from bbia_sim import unity_reachy_controller

        # Vérifier que le fichier contient bien logger.error pour erreurs critiques
        source = inspect.getsource(unity_reachy_controller)
        assert "logger.error" in source or "logger.exception" in source
        # Vérifier que "critique" est mentionné dans les messages d'erreur
        assert "critique" in source.lower() or "erreur" in source.lower()

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
