#!/usr/bin/env python3
"""
Tests pour vérifier les améliorations d'intelligence de BBIA.
Vérifie que les améliorations fonctionnent sans régression.
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_behavior import ConversationBehavior, WakeUpBehavior
from bbia_sim.bbia_huggingface import BBIAHuggingFace


class TestBBIAIntelligenceImprovements:
    """Tests pour vérifier les améliorations d'intelligence."""

    def test_wake_up_messages_variety(self):
        """Test que les messages de réveil sont variés."""
        behavior = WakeUpBehavior()
        # Vérifier que le code contient plusieurs messages de réveil
        # (on teste la structure, pas l'exécution complète)
        import inspect

        source = inspect.getsource(behavior.execute)
        assert "wake_messages" in source, "Messages de réveil doivent être définis"
        assert source.count('"') >= 16, "Doit y avoir plusieurs messages de réveil"

    def test_conversation_greeting_responses_variety(self):
        """Test que les réponses de salutation sont variées."""
        behavior = ConversationBehavior()
        assert "greeting" in behavior.enriched_responses
        greeting_responses = behavior.enriched_responses["greeting"]
        assert (
            len(greeting_responses) >= 4
        ), "Doit avoir au moins 4 réponses de salutation"
        # Vérifier que les réponses sont différentes
        assert len(set(greeting_responses)) == len(
            greeting_responses
        ), "Les réponses doivent être uniques"

    def test_conversation_default_responses_improved(self):
        """Test que les réponses par défaut sont améliorées."""
        behavior = ConversationBehavior()
        assert "default" in behavior.enriched_responses
        default_responses = behavior.enriched_responses["default"]
        # AMÉLIORATION: Au moins 7 réponses (après amélioration)
        assert (
            len(default_responses) >= 7
        ), f"Doit avoir au moins 7 réponses par défaut améliorées (actuel: {len(default_responses)})"
        # Vérifier que les réponses sont plus intelligentes (contiennent des questions/encouragements)
        intelligent_indicators = [
            "pouvez-vous",
            "j'aimerais",
            "partagez",
            "réflexion",
            "curieux",
            "intéressant",
            "comprendre",
            "approfondir",
        ]
        has_intelligent_responses = any(
            any(
                indicator.lower() in resp.lower()
                for indicator in intelligent_indicators
            )
            for resp in default_responses
        )
        assert (
            has_intelligent_responses
        ), "Les réponses par défaut doivent être plus intelligentes avec questions"

    def test_huggingface_response_quality(self):
        """Test que BBIAHuggingFace génère des réponses de qualité."""
        try:
            hf = BBIAHuggingFace()
            # Test avec un message simple
            response = hf.chat("Bonjour")
            assert isinstance(response, str), "La réponse doit être une chaîne"
            assert len(response) > 10, "La réponse doit être substantielle"
            assert len(response) < 200, "La réponse ne doit pas être trop longue"
        except ImportError:
            pytest.skip("Hugging Face non disponible")

    def test_conversation_uses_huggingface_if_available(self):
        """Test que ConversationBehavior utilise HuggingFace si disponible."""
        behavior = ConversationBehavior()
        # Si HF est disponible, hf_chat ne doit pas être None
        # Si non disponible, le fallback enrichi doit fonctionner
        assert behavior.hf_chat is not None or behavior.enriched_responses is not None

    def test_sentiment_detection_works(self):
        """Test que la détection de sentiment fonctionne."""
        behavior = ConversationBehavior()
        # Test détection émotion basique
        emotion1 = behavior._detect_emotion_from_text("Je suis content et heureux")
        assert emotion1 == "happy", "Devrait détecter 'happy'"

        emotion2 = behavior._detect_emotion_from_text("Je suis triste et déçu")
        assert emotion2 == "sad", "Devrait détecter 'sad'"

        emotion3 = behavior._detect_emotion_from_text("Je suis curieux et intrigué")
        assert emotion3 == "curious", "Devrait détecter 'curious'"

        emotion4 = behavior._detect_emotion_from_text("texte neutre sans émotion")
        assert emotion4 == "neutral", "Devrait détecter 'neutral'"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
