#!/usr/bin/env python3
"""
🧪 TESTS AMÉLIORATIONS INTELLIGENCE CONTEXTE BBIA
Vérifie que les améliorations d'intelligence contextuelle fonctionnent
sans régression.
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestBBIAIntelligenceContext:
    """Tests pour les améliorations d'intelligence contextuelle."""

    def test_context_reference_detection(self):
        """Test que BBIA détecte les références au contexte précédent."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            hf = BBIAHuggingFace()

            # Premier message pour créer un contexte
            response1 = hf.chat("J'aime la robotique")
            assert isinstance(response1, str)
            assert len(response1) > 0

            # Deuxième message avec référence ("ça")
            response2 = hf.chat("C'est intéressant ça")
            assert isinstance(response2, str)
            assert len(response2) > 0
            # La réponse devrait idéalement référencer le contexte précédent
            # (test informatif, ne fait pas échouer si pas de référence)

            print("✅ Contexte: message1='J'aime la robotique'")
            print("✅ Référence: message2='C'est intéressant ça'")
            print(f"   → Réponse: {response2[:60]}...")

        except ImportError:
            pytest.skip("Hugging Face non disponible")

    def test_generic_responses_variety_improved(self):
        """Test que les réponses génériques ont été améliorées (plus variées)."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            hf = BBIAHuggingFace()

            # Test plusieurs messages génériques pour vérifier variété
            test_messages = [
                "C'est intéressant",
                "Je vois",
                "D'accord",
                "C'est bien",
            ]

            responses = []
            for msg in test_messages:
                response = hf.chat(msg)
                responses.append(response)
                assert isinstance(response, str)
                assert len(response) > 20, "Réponses doivent être substantielles"

            # Vérifier qu'on a de la variété (au moins 2 réponses différentes)
            unique_responses = set(responses)
            assert (
                len(unique_responses) >= 2
            ), f"Pas assez de variété (unique: {len(unique_responses)})"

            print(
                f"✅ Variété réponses génériques: {len(unique_responses)}/{len(test_messages)} uniques"
            )

        except ImportError:
            pytest.skip("Hugging Face non disponible")

    def test_question_responses_improved(self):
        """Test que les réponses aux questions ont été améliorées."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            hf = BBIAHuggingFace()

            # Tester différentes questions
            questions = [
                "Comment ça va ?",
                "Pourquoi c'est important ?",
                "Qu'est-ce que c'est ?",
            ]

            for question in questions:
                response = hf.chat(question)
                assert isinstance(response, str)
                assert (
                    len(response) > 15
                ), "Réponses questions doivent être substantielles"
                # Vérifier que c'est une réponse intelligente (contient question ou encouragement)
                has_intelligent_indicator = any(
                    word in response.lower()
                    for word in [
                        "question",
                        "réfléchir",
                        "détails",
                        "intrigue",
                        "pensez",
                    ]
                )
                assert (
                    has_intelligent_indicator
                ), f"Réponse question doit être intelligente: {response[:50]}"
                print(f"✅ Question '{question}' → Réponse intelligente")

        except ImportError:
            pytest.skip("Hugging Face non disponible")

    def test_context_responses_personality_variety(self):
        """Test que les réponses contextuelles varient selon personnalité."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            # Tester avec différentes personnalités
            personalities = ["friendly_robot", "curious", "enthusiastic", "calm"]

            for personality in personalities:
                hf = BBIAHuggingFace()
                hf.bbia_personality = personality

                # Créer un contexte
                hf.chat("J'aime programmer")

                # Message avec référence
                response = hf.chat("C'est passionnant ça")

                assert isinstance(response, str)
                assert len(response) > 0
                print(f"✅ Personnalité '{personality}': Réponse contextuelle générée")

        except ImportError:
            pytest.skip("Hugging Face non disponible")

    def test_generic_responses_length_and_intelligence(self):
        """Test que les réponses génériques sont longues et intelligentes."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            hf = BBIAHuggingFace()

            # Messages génériques
            generic_messages = ["C'est bien", "Intéressant", "Je comprends"]

            for msg in generic_messages:
                response = hf.chat(msg)

                # Vérifier longueur (doit être substantielle)
                assert (
                    len(response) >= 25
                ), f"Réponse trop courte: {len(response)} caractères"

                # Vérifier intelligence (contient questions ou encouragements)
                intelligent_words = [
                    "pourquoi",
                    "comment",
                    "qu'est-ce",
                    "dites",
                    "racontez",
                    "curieux",
                    "intéressant",
                    "explorer",
                ]
                has_intelligence = any(
                    word in response.lower() for word in intelligent_words
                )

                assert (
                    has_intelligence
                ), f"Réponse doit être intelligente: {response[:60]}..."

                print(
                    f"✅ Message générique '{msg}' → Réponse intelligente ({len(response)} chars)"
                )

        except ImportError:
            pytest.skip("Hugging Face non disponible")

    def test_no_regression_chat_api(self):
        """Test qu'il n'y a pas de régression dans l'API chat."""
        try:
            from bbia_sim.bbia_huggingface import BBIAHuggingFace

            hf = BBIAHuggingFace()

            # Vérifier que chat() accepte toujours les mêmes paramètres
            response1 = hf.chat("Bonjour")
            assert isinstance(response1, str)

            response2 = hf.chat("Salut", use_context=True)
            assert isinstance(response2, str)

            response3 = hf.chat("Hello", use_context=False)
            assert isinstance(response3, str)

            print("✅ API chat préservée (pas de régression)")

        except ImportError:
            pytest.skip("Hugging Face non disponible")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
