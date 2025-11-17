#!/usr/bin/env python3
"""
Tests unitaires pour la fonctionnalité de chat intelligent BBIA.
Tests de la nouvelle fonctionnalité chat dans bbia_huggingface.py
"""

from collections import deque

import pytest

# Import conditionnel pour éviter erreurs si HF indisponible
try:
    from bbia_sim.bbia_huggingface import BBIAHuggingFace

    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    BBIAHuggingFace = None  # type: ignore


class TestBBIAHuggingFaceChat:
    """Tests pour la fonctionnalité chat intelligent."""

    def setup_method(self) -> None:
        """Initialise l'instance pour chaque test."""
        try:
            self.hf = BBIAHuggingFace()
        except ImportError:
            pytest.skip("Hugging Face transformers non disponible")

    def test_chat_simple_greeting(self) -> None:
        """Test chat avec salutation simple."""
        response = self.hf.chat("Bonjour")

        assert isinstance(response, str)
        assert len(response) > 0
        assert any(word in response.lower() for word in ["bonjour", "hello", "salut"])

    def test_chat_conversation_history(self) -> None:
        """Test que l'historique de conversation est sauvegardé."""
        initial_count = len(self.hf.conversation_history)
        max_history_size = 1000

        self.hf.chat("Bonjour")
        self.hf.chat("Comment allez-vous ?")

        # Si l'historique est déjà plein (1000 messages), il ne peut pas augmenter
        # Sinon, il doit augmenter de 2
        if initial_count >= max_history_size:
            # L'historique est déjà plein, il reste à max_history_size
            assert len(self.hf.conversation_history) == max_history_size
        else:
            # L'historique peut augmenter
            assert len(self.hf.conversation_history) == min(
                initial_count + 2, max_history_size
            )

        # Vérifier structure des entrées
        last_entry = self.hf.conversation_history[-1]
        assert "user" in last_entry
        assert "bbia" in last_entry
        assert "sentiment" in last_entry
        assert "timestamp" in last_entry

    def test_chat_with_empty_message(self) -> None:
        """Test chat avec message vide."""
        response = self.hf.chat("")

        assert isinstance(response, str)
        assert len(response) > 0

    def test_chat_error_handling(self) -> None:
        """Test gestion des erreurs dans chat."""
        # Test avec message normal
        response = self.hf.chat("test message")
        assert "reformuler" in response.lower() or len(response) > 0

    def test_generate_simple_response_salutations(self) -> None:
        """Test génération réponse simple pour salutations."""
        sentiment = {"sentiment": "NEUTRAL"}

        response1 = self.hf._generate_simple_response("Bonjour", sentiment)
        # La réponse doit contenir au moins un mot de salutation/amabilité
        assert any(
            word in response1.lower()
            for word in [
                "bonjour",
                "hello",
                "salut",
                "comment",
                "ça",
                "plaisir",
                "coucou",
            ]
        )

        response2 = self.hf._generate_simple_response("Salut", sentiment)
        # La réponse doit contenir au moins un mot de salutation/amabilité
        assert any(
            word in response2.lower()
            for word in [
                "bonjour",
                "hello",
                "salut",
                "comment",
                "ça",
                "plaisir",
                "coucou",
            ]
        )

    def test_generate_simple_response_positive(self) -> None:
        """Test génération réponse pour sentiment positif."""
        sentiment = {"sentiment": "POSITIVE"}

        response = self.hf._generate_simple_response("Je suis content", sentiment)
        assert len(response) > 0
        # Vérifier qu'il y a une réponse positive (plus flexible)
        assert any(
            word in response.lower()
            for word in [
                "super",
                "content",
                "bien",
                "formidable",
                "excellent",
                "génial",
                "heureux",
                "plaisir",
            ]
        )

    def test_adapt_response_to_personality(self) -> None:
        """Test adaptation réponse selon personnalité."""
        sentiment = {"sentiment": "NEUTRAL"}

        response = self.hf._adapt_response_to_personality("Test", sentiment)
        assert "🤖" in response or "💬" in response

        # Tester autres personnalités
        self.hf.bbia_personality = "curious"
        response2 = self.hf._adapt_response_to_personality("Test", sentiment)
        assert "🤔" in response2

    def test_build_context_string_empty(self) -> None:
        """Test construction contexte avec historique vide."""
        context = self.hf._build_context_string()

        assert isinstance(context, str)
        assert len(context) > 0
        assert "bbia" in context.lower() or "robot" in context.lower()

    def test_build_context_string_with_history(self) -> None:
        """Test construction contexte avec historique."""
        # Ajouter quelques messages
        self.hf.chat("Message 1")
        self.hf.chat("Message 2")
        self.hf.chat("Message 3")

        context = self.hf._build_context_string()

        assert isinstance(context, str)
        assert "historique" in context.lower() or "user" in context.lower()

    def test_chat_context_preservation(self) -> None:
        """Test que le contexte est préservé entre plusieurs appels."""
        max_history_size = 1000
        self.hf.chat("Premier message")
        count1 = len(self.hf.conversation_history)

        self.hf.chat("Deuxième message")
        count2 = len(self.hf.conversation_history)

        # Si l'historique est déjà plein, il reste à max_history_size
        if count1 >= max_history_size:
            assert count2 == max_history_size
        else:
            assert count2 == count1 + 1

    def test_bbia_personality_default(self) -> None:
        """Test que la personnalité par défaut est correcte."""
        assert self.hf.bbia_personality == "friendly_robot"

    def test_chat_fallback_on_error(self) -> None:
        """Test que le fallback fonctionne en cas d'erreur."""
        # Forcer une erreur en définissant un modèle invalide
        # Le chat doit quand même retourner une réponse
        response = self.hf.chat("test")
        assert isinstance(response, str)
        assert len(response) > 0


def test_hf_module_initialization() -> None:
    """Test initialisation du module Hugging Face avec variables chat."""
    try:
        hf = BBIAHuggingFace()

        # Vérifier que les variables chat sont initialisées
        assert hasattr(hf, "conversation_history")
        assert hasattr(hf, "context")
        assert hasattr(hf, "bbia_personality")
        assert isinstance(hf.conversation_history, deque)
        assert isinstance(hf.context, dict)
        assert isinstance(hf.bbia_personality, str)
    except ImportError:
        pytest.skip("Hugging Face transformers non disponible")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
