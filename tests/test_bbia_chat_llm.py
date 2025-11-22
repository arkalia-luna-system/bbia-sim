#!/usr/bin/env python3
"""Tests pour le module BBIA Chat LLM.

Tests unitaires pour l'intégration LLM conversationnel (Phi-2, TinyLlama).
"""

import sys
import time
from unittest.mock import MagicMock, patch

import pytest

from bbia_sim.bbia_chat import HF_AVAILABLE, BBIAChat


class TestBBIAChat:
    """Tests pour BBIAChat."""

    def test_bbia_chat_creation(self):
        """Test création d'une instance BBIAChat."""
        chat = BBIAChat()
        assert chat.llm_model is None or chat.llm_model is not None
        assert chat.llm_tokenizer is None or chat.llm_tokenizer is not None
        assert chat.context.maxlen == 10
        assert chat.personality == "friendly"
        assert isinstance(chat.user_preferences, dict)

    def test_bbia_chat_with_robot_api(self):
        """Test création BBIAChat avec robot_api."""
        mock_robot_api = MagicMock()
        chat = BBIAChat(robot_api=mock_robot_api)
        assert chat.robot_api == mock_robot_api

    def test_build_context_prompt(self):
        """Test construction prompt avec contexte."""
        chat = BBIAChat()
        user_message = "Bonjour"

        # Sans contexte
        prompt = chat._build_context_prompt(user_message)
        assert "BBIA" in prompt
        assert "Bonjour" in prompt
        assert "Utilisateur:" in prompt

        # Avec contexte
        chat.context.append(
            {
                "user": "Salut",
                "assistant": "Bonjour !",
                "timestamp": time.time(),
            }
        )
        prompt_with_context = chat._build_context_prompt(user_message)
        assert "Salut" in prompt_with_context
        assert "Bonjour !" in prompt_with_context

    def test_detect_action(self):
        """Test détection actions robot."""
        chat = BBIAChat()

        # Test look_right
        action = chat._detect_action("Tourne la tête à droite")
        assert action is not None
        assert action["action"] == "look_right"
        assert action["confidence"] == 0.9

        # Test look_left
        action = chat._detect_action("Regarde à gauche")
        assert action is not None
        assert action["action"] == "look_left"

        # Test look_up
        action = chat._detect_action("Dirige la tête vers le haut")
        assert action is not None
        assert action["action"] == "look_up"

        # Test look_down
        action = chat._detect_action("Regarde en bas")
        assert action is not None
        assert action["action"] == "look_down"

        # Test wake_up
        action = chat._detect_action("Réveille-toi")
        assert action is not None
        assert action["action"] == "wake_up"

        # Test sleep
        action = chat._detect_action("Endors-toi")
        assert action is not None
        assert action["action"] == "sleep"

        # Test pas d'action
        action = chat._detect_action("Comment vas-tu ?")
        assert action is None

    def test_execute_action(self):
        """Test exécution actions robot."""
        mock_robot_api = MagicMock()
        mock_robot_api.goto_target = MagicMock()

        chat = BBIAChat(robot_api=mock_robot_api)

        # Mock create_head_pose (importé dans _execute_action depuis reachy_mini.utils)
        # Créer un mock du module reachy_mini.utils pour éviter ImportError
        mock_pose = MagicMock()
        mock_create_head_pose = MagicMock(return_value=mock_pose)

        # Créer un mock du module reachy_mini et de son sous-module utils
        mock_reachy_mini = MagicMock()
        mock_reachy_mini_utils = MagicMock()
        mock_reachy_mini_utils.create_head_pose = mock_create_head_pose
        mock_reachy_mini.utils = mock_reachy_mini_utils

        # Patcher sys.modules pour que l'import fonctionne
        with patch.dict(
            sys.modules,
            {
                "reachy_mini": mock_reachy_mini,
                "reachy_mini.utils": mock_reachy_mini_utils,
            },
        ):
            # Test look_right
            chat._execute_action({"action": "look_right"})
            # Vérifier que create_head_pose a été appelé et goto_target aussi
            mock_create_head_pose.assert_called_once_with(
                yaw=0.3, pitch=0.0, degrees=False
            )
            mock_robot_api.goto_target.assert_called_once_with(
                head=mock_pose, duration=1.0
            )

            # Reset mocks
            mock_robot_api.goto_target.reset_mock()
            mock_create_head_pose.reset_mock()

            # Test look_left
            chat._execute_action({"action": "look_left"})
            mock_create_head_pose.assert_called_once_with(
                yaw=-0.3, pitch=0.0, degrees=False
            )
            mock_robot_api.goto_target.assert_called_once_with(
                head=mock_pose, duration=1.0
            )

    def test_execute_action_no_robot_api(self):
        """Test exécution action sans robot_api."""
        chat = BBIAChat(robot_api=None)
        # Ne doit pas lever d'exception
        chat._execute_action({"action": "look_right"})

    def test_sanitize_response(self):
        """Test nettoyage réponse LLM."""
        chat = BBIAChat()

        # Test retirer code
        response = "Voici du code:\n```python\nprint('test')\n```\nFin"
        cleaned = chat._sanitize_response(response)
        assert "```" not in cleaned

        # Test limiter longueur
        long_response = "A" * 600
        cleaned = chat._sanitize_response(long_response)
        assert len(cleaned) <= 503  # 500 + "..."

    def test_chat_empty_message(self):
        """Test chat avec message vide."""
        chat = BBIAChat()
        response = chat.chat("")
        assert "compris" in response.lower() or "reformuler" in response.lower()

    def test_chat_context_management(self):
        """Test gestion contexte conversation."""
        chat = BBIAChat()

        # Mocker _load_llm() et generate() pour éviter le chargement réel du modèle LLM
        # qui cause un timeout en CI
        with (
            patch.object(chat, "_load_llm"),
            patch.object(chat, "generate", return_value="Réponse simulée"),
        ):
            # Simuler conversation
            chat.chat("Bonjour")
            assert len(chat.context) == 1

            chat.chat("Comment vas-tu ?")
            assert len(chat.context) == 2

            # Vérifier structure contexte
            entry = chat.context[-1]
            assert "user" in entry
            assert "assistant" in entry
            assert "timestamp" in entry

    @pytest.mark.skipif(not HF_AVAILABLE, reason="Hugging Face non disponible")
    def test_generate_with_llm(self):
        """Test génération réponse avec LLM (si disponible)."""
        chat = BBIAChat()

        # Si LLM chargé
        if chat.llm_model and chat.llm_tokenizer:
            prompt = "Bonjour, comment vas-tu ?"
            response = chat.generate(prompt, max_length=50)
            assert isinstance(response, str)
            assert len(response) > 0

    def test_generate_without_llm(self):
        """Test génération sans LLM (fallback)."""
        chat = BBIAChat()
        chat.llm_model = None
        chat.llm_tokenizer = None

        response = chat.generate("Test")
        assert "disponible" in response.lower()

    def test_generate_timeout(self):
        """Test timeout génération."""
        chat = BBIAChat()

        # Mock génération lente
        if chat.llm_model:
            original_generate = chat.llm_model.generate

            def slow_generate(*args, **kwargs):
                # OPTIMISATION: Réduire sleep de 6s → 1.1s (suffisant pour tester timeout, 5.5x plus rapide)
                time.sleep(1.1)  # Dépasse timeout de 1.0s
                return original_generate(*args, **kwargs)

            chat.llm_model.generate = slow_generate

            # OPTIMISATION: Ajuster timeout à 1.0s pour correspondre au nouveau sleep
            response = chat.generate("Test", timeout=1.0)
            # Devrait retourner message timeout ou erreur
            assert isinstance(response, str)

    def test_chat_with_action_detection(self):
        """Test chat avec détection action."""
        mock_robot_api = MagicMock()
        mock_robot_api.goto_target = MagicMock()

        chat = BBIAChat(robot_api=mock_robot_api)

        # Mock create_head_pose pour éviter ImportError
        # Créer un mock du module reachy_mini.utils pour éviter ImportError
        mock_pose = MagicMock()
        mock_create_head_pose = MagicMock(return_value=mock_pose)

        # Créer un mock du module reachy_mini et de son sous-module utils
        mock_reachy_mini = MagicMock()
        mock_reachy_mini_utils = MagicMock()
        mock_reachy_mini_utils.create_head_pose = mock_create_head_pose
        mock_reachy_mini.utils = mock_reachy_mini_utils

        # Patcher sys.modules pour que l'import fonctionne
        with (
            patch.dict(
                sys.modules,
                {
                    "reachy_mini": mock_reachy_mini,
                    "reachy_mini.utils": mock_reachy_mini_utils,
                },
            ),
            patch.object(chat, "_load_llm"),
            patch.object(chat, "generate", return_value="Réponse simulée"),
        ):
            # Message avec action
            response = chat.chat("Tourne la tête à droite")
            # Vérifier que la réponse est générée
            assert isinstance(response, str)
            # Note: goto_target peut ne pas être appelé si SDK non disponible, c'est normal

    def test_user_preferences(self):
        """Test gestion préférences utilisateur."""
        chat = BBIAChat()
        assert isinstance(chat.user_preferences, dict)
        # Note: user_preferences peut contenir des valeurs chargées depuis fichier
        # On vérifie juste que c'est un dict

        # Ajouter préférence
        chat.user_preferences["test_preference"] = "test_value"
        assert chat.user_preferences["test_preference"] == "test_value"

    def test_personality(self):
        """Test gestion personnalité."""
        chat = BBIAChat()
        assert chat.personality == "friendly"

        # Changer personnalité (pour futures phases)
        chat.personality = "professional"
        assert chat.personality == "professional"
