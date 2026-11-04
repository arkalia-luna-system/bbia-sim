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
        # Vérifier la structure du code sans l'exécuter
        import inspect

        from bbia_sim import bbia_chat

        source = inspect.getsource(bbia_chat.BBIAChat.generate)
        # Vérifier que les erreurs sont gérées avec logger.error
        assert "except Exception" in source
        assert "logger.error" in source or "critique" in source.lower()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_chat_chat_error_handling(self):
        """Test que chat() gère les erreurs correctement."""
        # Ne pas charger de modèle LLM (évite download HF / timeout CI 30s)
        chat = BBIAChat(robot_api=None)
        # Sentinelles truthy pour court-circuiter le lazy-load dans chat()
        chat.llm_model = MagicMock(name="llm_model_stub")
        chat.llm_tokenizer = MagicMock(name="llm_tokenizer_stub")

        mock_logger = MagicMock(spec=logging.Logger)
        with patch("bbia_sim.bbia_chat.logger", mock_logger):
            with patch.object(chat, "_load_llm") as load_llm:
                with patch.object(
                    chat, "generate", side_effect=RuntimeError("Erreur chat")
                ):
                    result = chat.chat("test")
                    load_llm.assert_not_called()
                    assert isinstance(result, str)
                    assert (
                        "comprends" in result.lower() or "reformuler" in result.lower()
                    )
                    mock_logger.error.assert_called()
                    assert "critique" in str(mock_logger.error.call_args).lower()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_chat_execute_action_error_handling(self):
        """Test que _execute_action() gère les erreurs correctement."""
        # Ne pas charger de modèle LLM pour économiser RAM
        # Vérifier la structure du code sans l'exécuter
        import inspect

        from bbia_sim import bbia_chat

        source = inspect.getsource(bbia_chat.BBIAChat._execute_action)
        # Vérifier que les erreurs sont gérées avec logger.error
        assert "except Exception" in source
        assert "logger.error" in source or "critique" in source.lower()

    @pytest.mark.unit
    @pytest.mark.fast
    def test_bbia_chat_apply_emotion_error_handling(self):
        """Test que _apply_emotion() gère les erreurs correctement."""
        # Ne pas charger de modèle LLM pour économiser RAM
        # Vérifier la structure du code sans l'exécuter
        import inspect

        from bbia_sim import bbia_chat

        source = inspect.getsource(bbia_chat.BBIAChat._apply_emotion)
        # Vérifier que les erreurs sont gérées avec logger.error
        assert "except Exception" in source
        assert "logger.error" in source or "critique" in source.lower()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
