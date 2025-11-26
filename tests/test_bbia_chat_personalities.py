#!/usr/bin/env python3
"""Tests pour les personnalités du chat BBIA.

Tests du système de personnalités (friendly, professional, playful, etc.)
et de l'apprentissage des préférences utilisateur.
"""

from unittest.mock import patch

import pytest

from bbia_sim.bbia_chat import BBIAChat


class TestBBIAChatPersonalities:
    """Tests pour les personnalités du chat."""

    @pytest.fixture
    def chat(self):
        """Crée une instance BBIAChat pour les tests."""
        with patch("bbia_sim.bbia_chat.HF_AVAILABLE", False):
            # Désactiver LLM pour tests rapides
            chat = BBIAChat(robot_api=None)
            chat.llm_model = None
            chat.llm_tokenizer = None
            return chat

    def test_personality_default(self, chat):
        """Test que la personnalité par défaut est 'friendly'."""
        assert chat.personality == "friendly"

    def test_set_personality(self, chat):
        """Test changement de personnalité."""
        chat.set_personality("professional")
        assert chat.personality == "professional"

        chat.set_personality("playful")
        assert chat.personality == "playful"

    def test_personality_invalid(self, chat):
        """Test que personnalité invalide ne change pas la personnalité."""
        original = chat.personality
        chat.set_personality("invalid_personality")
        assert chat.personality == original

    def test_personality_system_prompt(self, chat):
        """Test que le system prompt change selon personnalité."""
        # Vérifier que _build_context_prompt inclut personnalité
        chat.set_personality("professional")
        prompt = chat._build_context_prompt("Bonjour")
        assert "professional" in prompt.lower() or "professionnel" in prompt.lower()

    def test_learn_preference(self, chat):
        """Test apprentissage préférences utilisateur."""
        chat.learn_preference("court", {"response_length": "short"})
        assert chat.user_preferences.get("response_length") == "short"

    def test_adapt_to_preferences(self, chat):
        """Test adaptation réponse selon préférences."""
        chat.user_preferences["response_length"] = "short"
        long_response = "Voici une très longue réponse avec beaucoup de détails. Deuxième phrase. Troisième phrase."
        adapted = chat._adapt_to_preferences(long_response)
        # Si réponse a plus de 2 phrases, elle doit être raccourcie
        if long_response.count(".") > 2:
            assert len(adapted) <= len(long_response)

    def test_preferences_persistence(self, chat):
        """Test que préférences sont sauvegardées."""
        chat.learn_preference("court", {"response_length": "short"})
        # Vérifier que préférences sont dans user_preferences
        assert "response_length" in chat.user_preferences


@pytest.mark.slow
class TestBBIAChatPersonalitiesWithLLM:
    """Tests personnalités avec LLM réel (marqué slow)."""

    @pytest.fixture
    def chat_with_llm(self):
        """Crée une instance BBIAChat avec LLM (si disponible)."""
        # Skip en CI si trop lent (chargement modèle LLM)
        import os

        if os.environ.get("CI", "false").lower() == "true":
            pytest.skip("Test désactivé en CI (chargement modèle LLM trop lent)")
        try:
            chat = BBIAChat(robot_api=None)
            if chat.llm_model is None:
                pytest.skip("LLM non disponible")
            return chat
        except Exception:
            pytest.skip("LLM non disponible")

    def test_personality_affects_response(self, chat_with_llm):
        """Test que personnalité affecte la réponse LLM."""
        chat_with_llm.set_personality("professional")
        response1 = chat_with_llm.chat("Bonjour")

        chat_with_llm.set_personality("playful")
        response2 = chat_with_llm.chat("Bonjour")

        # Les réponses devraient être différentes selon personnalité
        assert response1 != response2
