#!/usr/bin/env python3
"""
Tests de gestion d'erreurs pour bbia_chat.

Vérifie que les erreurs critiques sont loggées en ERROR.
"""

import logging
from unittest.mock import MagicMock, patch

import pytest

from bbia_sim.bbia_chat import BBIAChat

logger = logging.getLogger(__name__)


class TestBBIAChatErrorHandling:
    """Tests de gestion d'erreurs pour BBIAChat."""

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_chat_generate_error_handling(self):
        """Test que generate() gère les erreurs correctement."""
        # Ne pas charger de modèle LLM pour économiser RAM
        chat = BBIAChat(robot_api=None)
        # S'assurer que le modèle n'est pas chargé
        chat.llm_model = None
        chat.llm_tokenizer = None

        # Simuler une erreur lors de la génération
        mock_logger = MagicMock(spec=logging.Logger)
        with patch("bbia_sim.bbia_chat.logger", mock_logger):
            with patch.object(
                chat,
                "_generate_llm_response",
                side_effect=RuntimeError("Erreur génération"),
            ):
                result = chat.generate("test")
                # Doit retourner un message d'erreur sans crasher
                assert isinstance(result, str)
                assert "erreur" in result.lower() or "désolé" in result.lower()
                # Vérifier que logger.error a été appelé
                mock_logger.error.assert_called()
                assert "critique" in str(mock_logger.error.call_args).lower()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_chat_chat_error_handling(self):
        """Test que chat() gère les erreurs correctement."""
        # Ne pas charger de modèle LLM pour économiser RAM
        chat = BBIAChat(robot_api=None)
        chat.llm_model = None
        chat.llm_tokenizer = None

        # Simuler une erreur lors du chat
        mock_logger = MagicMock(spec=logging.Logger)
        with patch("bbia_sim.bbia_chat.logger", mock_logger):
            with patch.object(
                chat, "generate", side_effect=RuntimeError("Erreur chat")
            ):
                result = chat.chat("test")
                # Doit retourner un message d'erreur sans crasher
                assert isinstance(result, str)
                assert "comprends" in result.lower() or "reformuler" in result.lower()
                # Vérifier que logger.error a été appelé
                mock_logger.error.assert_called()
                assert "critique" in str(mock_logger.error.call_args).lower()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_chat_execute_action_error_handling(self):
        """Test que _execute_action() gère les erreurs correctement."""
        # Ne pas charger de modèle LLM pour économiser RAM
        chat = BBIAChat(robot_api=None)
        chat.llm_model = None
        chat.llm_tokenizer = None

        # Simuler une erreur lors de l'exécution d'une action
        mock_logger = MagicMock(spec=logging.Logger)
        with patch("bbia_sim.bbia_chat.logger", mock_logger):
            with patch.object(
                chat, "_extract_emotion", side_effect=RuntimeError("Erreur action")
            ):
                # Ne doit pas crasher
                chat._execute_action("test_action", "test message")
                # Vérifier que logger.error a été appelé
                mock_logger.error.assert_called()
                assert "critique" in str(mock_logger.error.call_args).lower()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_chat_apply_emotion_error_handling(self):
        """Test que _apply_emotion() gère les erreurs correctement."""
        # Ne pas charger de modèle LLM pour économiser RAM
        chat = BBIAChat(robot_api=None)
        chat.llm_model = None
        chat.llm_tokenizer = None

        # Simuler une erreur lors de l'application d'une émotion
        mock_logger = MagicMock(spec=logging.Logger)
        with patch("bbia_sim.bbia_chat.logger", mock_logger):
            with patch(
                "bbia_sim.bbia_chat.BBIAEmotions", side_effect=RuntimeError("Erreur")
            ):
                # Ne doit pas crasher
                chat._apply_emotion("happy")
                # Vérifier que logger.error a été appelé
                mock_logger.error.assert_called()
                assert "critique" in str(mock_logger.error.call_args).lower()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
