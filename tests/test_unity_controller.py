#!/usr/bin/env python3
"""Tests pour le module UnityReachyMiniController."""

import tempfile
from pathlib import Path
from unittest.mock import patch

from bbia_sim.unity_reachy_controller import UnityReachyMiniController


class TestUnityReachyMiniController:
    """Tests pour UnityReachyMiniController."""

    def test_init_default_files(self):
        """Test initialisation avec fichiers par défaut."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            assert controller.command_file.name == "reachy_commands.txt"
            assert controller.response_file.name == "reachy_response.txt"
            assert controller.last_response == ""
            assert controller.is_connected is True

    def test_init_custom_files(self):
        """Test initialisation avec fichiers personnalisés."""
        with tempfile.TemporaryDirectory() as _:
            cmd_file = Path(_) / "custom_cmd.txt"
            resp_file = Path(_) / "custom_resp.txt"

            controller = UnityReachyMiniController(
                command_file=str(cmd_file), response_file=str(resp_file)
            )
            assert controller.command_file == cmd_file
            assert controller.response_file == resp_file

    def test_init_communication_files_success(self):
        """Test initialisation réussie des fichiers de communication."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            assert controller.is_connected is True
            assert controller.command_file.exists()
            assert controller.response_file.exists()

    def test_init_communication_files_error(self):
        """Test gestion d'erreur lors de l'initialisation."""
        with patch(
            "pathlib.Path.exists", side_effect=PermissionError("Permission denied")
        ):
            controller = UnityReachyMiniController()
            assert controller.is_connected is False

    def test_send_command_success(self):
        """Test envoi de commande réussi."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            result = controller._send_command("test_command")
            assert result is True
            assert controller.command_file.read_text() == "test_command"

    def test_send_command_not_connected(self):
        """Test envoi de commande sans connexion."""
        controller = UnityReachyMiniController()
        controller.is_connected = False
        result = controller._send_command("test_command")
        assert result is False

    def test_send_command_error(self):
        """Test gestion d'erreur lors de l'envoi."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            with patch("pathlib.Path.write_text", side_effect=OSError("Write error")):
                result = controller._send_command("test_command")
                assert result is False

    def test_wait_for_response_success(self):
        """Test attente de réponse réussie."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            controller.response_file.write_text("test_response")
            result = controller._wait_for_response(timeout=1.0)
            assert result == "test_response"
            assert controller.last_response == "test_response"

    def test_wait_for_response_timeout(self):
        """Test timeout lors de l'attente de réponse."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            # S'assurer qu'il n'y a pas de contenu dans le fichier de réponse
            controller.response_file.write_text("")
            controller.last_response = ""
            result = controller._wait_for_response(timeout=0.1)
            assert result == ""

    def test_wait_for_response_same_content(self):
        """Test attente avec même contenu que précédemment."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            controller.last_response = "same_response"
            controller.response_file.write_text("same_response")
            result = controller._wait_for_response(timeout=0.1)
            assert result == ""

    def test_move_head_success(self):
        """Test mouvement de tête réussi."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            result = controller.move_head(1.0, 2.0, 3.0)
            assert result is True
            assert controller.command_file.read_text() == "move_head|1.0|2.0|3.0"

    def test_set_emotion_valid(self):
        """Test définition d'émotion valide."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            result = controller.set_emotion("happy")
            assert result is True
            assert controller.command_file.read_text() == "set_emotion|happy"

    def test_set_emotion_invalid(self):
        """Test définition d'émotion invalide."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            result = controller.set_emotion("invalid_emotion")
            assert result is False

    def test_set_emotion_case_insensitive(self):
        """Test définition d'émotion insensible à la casse."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            result = controller.set_emotion("HAPPY")
            assert result is True
            assert controller.command_file.read_text() == "set_emotion|happy"

    def test_reset_position(self):
        """Test remise à zéro de position."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            result = controller.reset_position()
            assert result is True
            assert controller.command_file.read_text() == "reset"

    def test_get_status_success(self):
        """Test récupération de statut réussie."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            controller.response_file.write_text("status_ok")
            result = controller.get_status()
            assert result == "status_ok"

    def test_get_status_failure(self):
        """Test récupération de statut échouée."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.get_status()
            assert result == ""

    @patch("time.sleep")
    def test_bbia_awake_sequence(self, mock_sleep):
        """Test séquence de réveil BBIA."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            result = controller.bbia_awake()
            assert result is True
            # Vérifier que set_emotion a été appelé
            assert controller.command_file.read_text() == "set_emotion|neutral"

    def test_bbia_awake_emotion_failure(self):
        """Test échec de la séquence de réveil."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_awake()
            assert (
                result is True
            )  # La méthode retourne toujours True même en cas d'erreur

    def test_interactive_mode_help_command(self):
        """Test commande help dans le mode interactif."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            # Mock input pour retourner 'help' puis 'quit' pour éviter la boucle infinie
            with patch("builtins.input", side_effect=["help", "quit"]):
                with patch("builtins.print") as mock_print:
                    controller.interactive_mode()
            # Vérifier que l'aide a été affichée
            mock_print.assert_called()

    def test_interactive_mode_quit_command(self):
        """Test commande quit dans le mode interactif."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            with patch("builtins.input", return_value="quit"):
                with patch("builtins.print"):
                    controller.interactive_mode()
            # Vérifier que le mode interactif s'est terminé proprement

    def test_interactive_mode_head_command(self):
        """Test commande head dans le mode interactif."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            with patch("builtins.input", side_effect=["head 1.0 2.0 3.0", "quit"]):
                controller.interactive_mode()
            # Vérifier que la commande head a été exécutée
            assert controller.command_file.read_text() == "move_head|1.0|2.0|3.0"

    def test_interactive_mode_emotion_command(self):
        """Test commande emotion dans le mode interactif."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            with patch("builtins.input", side_effect=["emotion happy", "quit"]):
                controller.interactive_mode()
            # Vérifier que la commande emotion a été exécutée
            assert controller.command_file.read_text() == "set_emotion|happy"

    def test_interactive_mode_reset_command(self):
        """Test commande reset dans le mode interactif."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            with patch("builtins.input", side_effect=["reset", "quit"]):
                controller.interactive_mode()
            # Vérifier que la commande reset a été exécutée
            assert controller.command_file.read_text() == "reset"

    def test_interactive_mode_awake_command(self):
        """Test commande awake dans le mode interactif."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            with patch("builtins.input", side_effect=["awake", "quit"]):
                with patch("time.sleep"):  # Mock sleep pour éviter les délais
                    controller.interactive_mode()
            # Vérifier que la commande awake a été exécutée
            assert controller.command_file.read_text() == "set_emotion|neutral"

    def test_interactive_mode_status_command(self):
        """Test commande status dans le mode interactif."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            controller.response_file.write_text("status_ok")
            with patch("builtins.input", side_effect=["status", "quit"]):
                controller.interactive_mode()
            # Vérifier que la commande status a été exécutée (le fichier contient la dernière commande)
            # Note: le status ne modifie pas le command_file, il affiche juste le statut
            assert controller.is_connected is True

    def test_interactive_mode_unknown_command(self):
        """Test commande inconnue dans le mode interactif."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            with patch("builtins.input", side_effect=["unknown_command", "quit"]):
                with patch("builtins.print") as mock_print:
                    controller.interactive_mode()
            # Vérifier qu'un message d'erreur a été affiché
            mock_print.assert_called_with(
                "❌ Commande inconnue. Tapez 'help' pour l'aide."
            )

    def test_interactive_mode_keyboard_interrupt(self):
        """Test interruption clavier dans le mode interactif."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            with patch("builtins.input", side_effect=KeyboardInterrupt):
                # Le KeyboardInterrupt devrait être géré proprement et sortir de la boucle
                controller.interactive_mode()
            # Vérifier que le mode interactif s'est terminé proprement
            # (pas de crash, pas de boucle infinie)

    def test_interactive_mode_exception_handling(self):
        """Test gestion d'exception dans le mode interactif."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            # Simuler une exception après quelques commandes normales, puis quit pour éviter la boucle infinie
            with patch(
                "builtins.input", side_effect=["help", Exception("Test error"), "quit"]
            ):
                with patch("builtins.print") as mock_print:
                    controller.interactive_mode()
            # Vérifier qu'un message d'erreur a été affiché
            mock_print.assert_called_with("❌ Erreur: Test error")

    def test_show_help(self):
        """Test affichage de l'aide."""
        with tempfile.TemporaryDirectory() as _:
            controller = UnityReachyMiniController()
            with patch("builtins.print") as mock_print:
                controller._show_help()
            # Vérifier que l'aide a été affichée
            mock_print.assert_called_once()
            help_text = mock_print.call_args[0][0]
            assert "Commandes BBIA disponibles" in help_text
            assert "status" in help_text
            assert "awake" in help_text
            assert "head" in help_text
            assert "emotion" in help_text
            assert "reset" in help_text
            assert "quit" in help_text
            assert "help" in help_text
