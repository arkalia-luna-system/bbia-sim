#!/usr/bin/env python3
"""
Tests pour l'intelligence conversationnelle améliorée de BBIA
Valide que ConversationBehavior utilise BBIAHuggingFace si disponible
et fallback enrichi sinon, avec réponses variées et personnalité.
"""

import sys
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import pytest


class TestBBIAConversationIntelligence:
    """Tests pour l'intelligence conversationnelle améliorée."""

    def test_conversation_behavior_init(self):
        """Test 1: Initialisation ConversationBehavior avec fallback."""
        from bbia_sim.bbia_behavior import ConversationBehavior

        # Test sans robot_api
        behavior = ConversationBehavior(robot_api=None)
        assert behavior is not None
        assert behavior.name == "conversation"
        assert behavior.priority == 7
        assert hasattr(behavior, "enriched_responses")
        assert isinstance(behavior.enriched_responses, dict)
        print("✅ ConversationBehavior initialisé correctement")

    def test_enriched_responses_variety(self):
        """Test 2: Vérifier variété des réponses enrichies."""
        from bbia_sim.bbia_behavior import ConversationBehavior

        behavior = ConversationBehavior(robot_api=None)

        # Vérifier que chaque catégorie a plusieurs réponses
        categories = [
            "greeting",
            "how_are_you",
            "goodbye",
            "thanks",
            "positive",
            "question",
            "default",
            "not_heard",
        ]

        for category in categories:
            responses = behavior.enriched_responses.get(category, [])
            assert len(responses) >= 2, f"Catégorie {category} doit avoir >= 2 réponses"
            print(f"✅ Catégorie {category}: {len(responses)} réponses disponibles")

    def test_enriched_response_selection(self):
        """Test 3: Vérifier sélection aléatoire des réponses."""
        from bbia_sim.bbia_behavior import ConversationBehavior

        behavior = ConversationBehavior(robot_api=None)

        # Tester sélection multiple fois (doit varier)
        responses_selected = []
        for _ in range(10):
            response = behavior._get_enriched_response("greeting")
            responses_selected.append(response)
            assert response in behavior.enriched_responses["greeting"]

        # Vérifier qu'on a eu de la variété (au moins 2 réponses différentes)
        unique_responses = set(responses_selected)
        assert (
            len(unique_responses) >= 2
        ), "La sélection doit produire de la variété après 10 tentatives"
        print(
            f"✅ Variété des réponses: {len(unique_responses)} réponses uniques sur 10"
        )

    def test_emotion_detection_from_text(self):
        """Test 4: Vérifier détection d'émotion depuis texte."""
        from bbia_sim.bbia_behavior import ConversationBehavior

        behavior = ConversationBehavior(robot_api=None)

        test_cases = [
            ("Je suis super content !", "happy"),
            ("Je suis triste aujourd'hui", "sad"),
            ("Je suis curieux de savoir", "curious"),
            ("Je me sens calme", "calm"),
            ("Je suis excité !", "excited"),
            ("Texte neutre sans émotion", "neutral"),
        ]

        for texte, expected_emotion in test_cases:
            detected = behavior._detect_emotion_from_text(texte.lower())
            assert (
                detected == expected_emotion
            ), f"Texte '{texte}' → {detected} (attendu {expected_emotion})"
            print(f"✅ Émotion détectée: '{texte}' → {detected}")

    def test_enriched_response_generation(self):
        """Test 5: Vérifier génération de réponses enrichies."""
        from bbia_sim.bbia_behavior import ConversationBehavior

        behavior = ConversationBehavior(robot_api=None)

        test_cases = [
            ("bonjour", "greeting"),
            ("salut", "greeting"),
            ("comment allez-vous", "how_are_you"),
            ("merci beaucoup", "thanks"),
            ("au revoir", "goodbye"),
            ("super génial", "positive"),
            ("pourquoi ?", "question"),
            ("texte quelconque", "default"),
        ]

        for texte, expected_category in test_cases:
            response = behavior._generate_enriched_response(texte.lower())
            assert response is not None
            assert len(response) > 0
            assert response in behavior.enriched_responses[expected_category]
            print(f"✅ Réponse '{texte}' → catégorie {expected_category}")

    def test_huggingface_fallback_graceful(self):
        """Test 6: Vérifier fallback gracieux si HuggingFace non disponible."""
        from bbia_sim.bbia_behavior import ConversationBehavior

        # Le comportement doit fonctionner même sans HuggingFace
        behavior = ConversationBehavior(robot_api=None)

        # Si HF n'est pas disponible, hf_chat doit être None
        # et le système enrichi doit être prêt
        if behavior.hf_chat is None:
            print("ℹ️  HuggingFace non disponible - système enrichi actif")
            assert hasattr(behavior, "enriched_responses")
            assert len(behavior.enriched_responses) > 0
        else:
            print("✅ HuggingFace disponible - mode intelligent actif")

    def test_conversation_behavior_no_regression(self):
        """Test 7: Vérifier qu'il n'y a pas de régression (API existante fonctionne)."""
        from bbia_sim.bbia_behavior import ConversationBehavior

        behavior = ConversationBehavior(robot_api=None)

        # Vérifier que l'API de base existe toujours
        assert hasattr(behavior, "execute")
        assert hasattr(behavior, "name")
        assert hasattr(behavior, "priority")
        assert behavior.name == "conversation"

        # Vérifier que execute accepte un contexte (même si on ne l'exécute pas complètement)
        context = {}
        # Note: On ne peut pas tester execute complètement car il nécessite
        # dire_texte et reconnaitre_parole qui nécessitent hardware/audio
        assert callable(behavior.execute)
        print("✅ Aucune régression détectée - API existante fonctionne")

    def test_emotional_response_behavior_comments(self):
        """Test 8: Vérifier que EmotionalResponseBehavior génère des commentaires variés."""
        from bbia_sim.bbia_behavior import EmotionalResponseBehavior
        from bbia_sim.bbia_emotions import BBIAEmotions

        emotions = BBIAEmotions()
        behavior = EmotionalResponseBehavior(emotions, robot_api=None)

        # Tester avec différents stimuli pour vérifier variété des commentaires
        test_stimuli = ["compliment", "surprise", "danger", "greeting"]

        for stimulus in test_stimuli:
            context = {"stimulus": stimulus}
            # Note: execute() appelle dire_texte() qui nécessite pyttsx3
            # On teste juste que le comportement peut s'exécuter
            result = behavior.execute(context)
            assert result is True
            print(f"✅ Stimulus '{stimulus}' traité avec succès")

    def test_vision_tracking_comments(self):
        """Test 9: Vérifier que VisionTrackingBehavior génère des commentaires lors de détection."""
        from bbia_sim.bbia_behavior import VisionTrackingBehavior
        from bbia_sim.bbia_vision import BBIAVision

        vision = BBIAVision()
        behavior = VisionTrackingBehavior(vision, robot_api=None)

        # Tester exécution (avec objets détectés)
        context = {}
        result = behavior.execute(context)
        # Le résultat peut être True ou False selon si des objets sont détectés
        assert isinstance(result, bool)
        print(f"✅ VisionTrackingBehavior exécuté: {result}")

    def test_no_regression_behavior_apis(self):
        """Test 10: Vérifier qu'il n'y a pas de régression dans les APIs des comportements."""
        from bbia_sim.bbia_behavior import (
            EmotionalResponseBehavior,
            VisionTrackingBehavior,
        )
        from bbia_sim.bbia_emotions import BBIAEmotions
        from bbia_sim.bbia_vision import BBIAVision

        # Vérifier que toutes les méthodes existent
        emotions = BBIAEmotions()
        vision = BBIAVision()

        emotional_behavior = EmotionalResponseBehavior(emotions)
        vision_behavior = VisionTrackingBehavior(vision)

        # Vérifier APIs
        assert hasattr(emotional_behavior, "execute")
        assert hasattr(emotional_behavior, "name")
        assert hasattr(vision_behavior, "execute")
        assert hasattr(vision_behavior, "name")

        assert emotional_behavior.name == "emotional_response"
        assert vision_behavior.name == "vision_tracking"

        print("✅ Aucune régression détectée - APIs préservées")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
