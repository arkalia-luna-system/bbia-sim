#!/usr/bin/env python3
"""
Tests pour le module UnityReachyMiniController
"""

import tempfile
from pathlib import Path
from unittest.mock import patch, mock_open

import pytest

from src.bbia_sim.unity_reachy_controller import UnityReachyMiniController


class TestUnityReachyMiniController:
    """Tests pour UnityReachyMiniController"""

    def test_init_default_files(self):
        """Test initialisation avec fichiers par défaut"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            assert controller.command_file.name == "reachy_commands.txt"
            assert controller.response_file.name == "reachy_response.txt"
            assert controller.last_response == ""
            assert controller.is_connected is True

    def test_init_custom_files(self):
        """Test initialisation avec fichiers personnalisés"""
        with tempfile.TemporaryDirectory() as temp_dir:
            cmd_file = Path(temp_dir) / "custom_cmd.txt"
            resp_file = Path(temp_dir) / "custom_resp.txt"
            
            controller = UnityReachyMiniController(
                command_file=str(cmd_file),
                response_file=str(resp_file)
            )
            assert controller.command_file == cmd_file
            assert controller.response_file == resp_file

    def test_init_communication_files_success(self):
        """Test initialisation réussie des fichiers de communication"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            assert controller.is_connected is True
            assert controller.command_file.exists()
            assert controller.response_file.exists()

    def test_init_communication_files_error(self):
        """Test gestion d'erreur lors de l'initialisation"""
        with patch('pathlib.Path.exists', side_effect=PermissionError("Permission denied")):
            controller = UnityReachyMiniController()
            assert controller.is_connected is False

    def test_send_command_success(self):
        """Test envoi de commande réussi"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller._send_command("test_command")
            assert result is True
            assert controller.command_file.read_text() == "test_command"

    def test_send_command_not_connected(self):
        """Test envoi de commande sans connexion"""
        controller = UnityReachyMiniController()
        controller.is_connected = False
        result = controller._send_command("test_command")
        assert result is False

    def test_send_command_error(self):
        """Test gestion d'erreur lors de l'envoi"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            with patch('pathlib.Path.write_text', side_effect=IOError("Write error")):
                result = controller._send_command("test_command")
                assert result is False

    def test_wait_for_response_success(self):
        """Test attente de réponse réussie"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.response_file.write_text("test_response")
            result = controller._wait_for_response(timeout=1.0)
            assert result == "test_response"
            assert controller.last_response == "test_response"

    def test_wait_for_response_timeout(self):
        """Test timeout lors de l'attente de réponse"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            # S'assurer qu'il n'y a pas de contenu dans le fichier de réponse
            controller.response_file.write_text("")
            controller.last_response = ""
            result = controller._wait_for_response(timeout=0.1)
            assert result == ""

    def test_wait_for_response_same_content(self):
        """Test attente avec même contenu que précédemment"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.last_response = "same_response"
            controller.response_file.write_text("same_response")
            result = controller._wait_for_response(timeout=0.1)
            assert result == ""

    def test_move_head_success(self):
        """Test mouvement de tête réussi"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.move_head(1.0, 2.0, 3.0)
            assert result is True
            assert controller.command_file.read_text() == "move_head|1.0|2.0|3.0"

    def test_set_emotion_valid(self):
        """Test définition d'émotion valide"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.set_emotion("happy")
            assert result is True
            assert controller.command_file.read_text() == "set_emotion|happy"

    def test_set_emotion_invalid(self):
        """Test définition d'émotion invalide"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.set_emotion("invalid_emotion")
            assert result is False

    def test_set_emotion_case_insensitive(self):
        """Test définition d'émotion insensible à la casse"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.set_emotion("HAPPY")
            assert result is True
            assert controller.command_file.read_text() == "set_emotion|happy"

    def test_reset_position(self):
        """Test remise à zéro de position"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.reset_position()
            assert result is True
            assert controller.command_file.read_text() == "reset"

    def test_get_status_success(self):
        """Test récupération de statut réussie"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.response_file.write_text("status_ok")
            result = controller.get_status()
            assert result == "status_ok"

    def test_get_status_failure(self):
        """Test récupération de statut échouée"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.get_status()
            assert result == ""

    @patch('time.sleep')
    def test_bbia_awake_sequence(self, mock_sleep):
        """Test séquence de réveil BBIA"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_awake()
            assert result is True
            # Vérifier que set_emotion a été appelé
            assert controller.command_file.read_text() == "set_emotion|neutral"

    def test_bbia_awake_emotion_failure(self):
        """Test échec de la séquence de réveil"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_awake()
            assert result is False

    def test_bbia_greeting_sequence(self):
        """Test séquence de salutation"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_greeting()
            assert result is True

    def test_bbia_greeting_emotion_failure(self):
        """Test échec de la séquence de salutation"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_greeting()
            assert result is False

    def test_bbia_sleep_sequence(self):
        """Test séquence d'endormissement"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_sleep()
            assert result is True

    def test_bbia_sleep_emotion_failure(self):
        """Test échec de la séquence d'endormissement"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_sleep()
            assert result is False

    def test_bbia_curious_sequence(self):
        """Test séquence de curiosité"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_curious()
            assert result is True

    def test_bbia_curious_emotion_failure(self):
        """Test échec de la séquence de curiosité"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_curious()
            assert result is False

    def test_bbia_happy_sequence(self):
        """Test séquence de joie"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_happy()
            assert result is True

    def test_bbia_happy_emotion_failure(self):
        """Test échec de la séquence de joie"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_happy()
            assert result is False

    def test_bbia_sad_sequence(self):
        """Test séquence de tristesse"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_sad()
            assert result is True

    def test_bbia_sad_emotion_failure(self):
        """Test échec de la séquence de tristesse"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_sad()
            assert result is False

    def test_bbia_angry_sequence(self):
        """Test séquence de colère"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_angry()
            assert result is True

    def test_bbia_angry_emotion_failure(self):
        """Test échec de la séquence de colère"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_angry()
            assert result is False

    def test_bbia_antenna_animation(self):
        """Test animation des antennes"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_antenna_animation()
            assert result is True

    def test_bbia_antenna_animation_failure(self):
        """Test échec de l'animation des antennes"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_antenna_animation()
            assert result is False

    def test_bbia_breathing_animation(self):
        """Test animation de respiration"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_breathing_animation()
            assert result is True

    def test_bbia_breathing_animation_failure(self):
        """Test échec de l'animation de respiration"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_breathing_animation()
            assert result is False

    def test_bbia_light_show(self):
        """Test spectacle de lumière"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_light_show()
            assert result is True

    def test_bbia_light_show_failure(self):
        """Test échec du spectacle de lumière"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_light_show()
            assert result is False

    def test_bbia_interactive_mode(self):
        """Test mode interactif"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_interactive_mode()
            assert result is True

    def test_bbia_interactive_mode_failure(self):
        """Test échec du mode interactif"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_interactive_mode()
            assert result is False

    def test_bbia_demo_mode(self):
        """Test mode démo"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_demo_mode()
            assert result is True

    def test_bbia_demo_mode_failure(self):
        """Test échec du mode démo"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_demo_mode()
            assert result is False

    def test_bbia_emergency_stop(self):
        """Test arrêt d'urgence"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_emergency_stop()
            assert result is True

    def test_bbia_emergency_stop_failure(self):
        """Test échec de l'arrêt d'urgence"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_emergency_stop()
            assert result is False

    def test_bbia_system_check(self):
        """Test vérification système"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_system_check()
            assert result is True

    def test_bbia_system_check_failure(self):
        """Test échec de la vérification système"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_system_check()
            assert result is False

    def test_bbia_calibration(self):
        """Test calibration"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_calibration()
            assert result is True

    def test_bbia_calibration_failure(self):
        """Test échec de la calibration"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_calibration()
            assert result is False

    def test_bbia_maintenance_mode(self):
        """Test mode maintenance"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_maintenance_mode()
            assert result is True

    def test_bbia_maintenance_mode_failure(self):
        """Test échec du mode maintenance"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_maintenance_mode()
            assert result is False

    def test_bbia_factory_reset(self):
        """Test remise à zéro usine"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            result = controller.bbia_factory_reset()
            assert result is True

    def test_bbia_factory_reset_failure(self):
        """Test échec de la remise à zéro usine"""
        with tempfile.TemporaryDirectory() as temp_dir:
            controller = UnityReachyMiniController()
            controller.is_connected = False
            result = controller.bbia_factory_reset()
            assert result is False
