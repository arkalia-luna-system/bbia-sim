#!/usr/bin/env python3
"""Tests améliorés pour BBIA Emotions - Coverage 70%+."""

import unittest

from src.bbia_sim.bbia_emotions import BBIAEmotions


class TestBBIAEmotionsImproved(unittest.TestCase):
    """Tests améliorés pour BBIAEmotions."""

    def setUp(self) -> None:
        """Setup test."""
        self.emotions = BBIAEmotions()

    def test_set_emotion_invalid(self) -> None:
        """Test changement émotion invalide."""
        result = self.emotions.set_emotion("invalid_emotion")
        self.assertFalse(result)
        # Émotion ne devrait pas changer
        self.assertEqual(self.emotions.current_emotion, "neutral")

    def test_set_emotion_intensity_limits(self) -> None:
        """Test limites intensité."""
        # Intensité négative → 0.0
        self.emotions.set_emotion("happy", -1.0)
        self.assertEqual(self.emotions.emotion_intensity, 0.0)

        # Intensité > 1.0 → 1.0
        self.emotions.set_emotion("happy", 2.0)
        self.assertEqual(self.emotions.emotion_intensity, 1.0)

        # Intensité normale
        self.emotions.set_emotion("happy", 0.7)
        self.assertEqual(self.emotions.emotion_intensity, 0.7)

    def test_get_current_emotion_structure(self) -> None:
        """Test structure émotion actuelle."""
        self.emotions.set_emotion("happy", 0.8)
        emotion_data = self.emotions.get_current_emotion()

        self.assertIn("name", emotion_data)
        self.assertIn("intensity", emotion_data)
        self.assertIn("timestamp", emotion_data)
        self.assertIn("description", emotion_data)
        self.assertEqual(emotion_data["name"], "happy")

    def test_get_emotion_history_limit(self) -> None:
        """Test historique avec limite."""
        # Créer plusieurs émotions
        for emotion in ["happy", "sad", "angry", "curious"]:
            self.emotions.set_emotion(emotion, 0.6)

        # Limite à 2
        history = self.emotions.get_emotion_history(limit=2)
        self.assertEqual(len(history), 2)
        self.assertEqual(history[-1]["emotion"], "curious")

        # Limite 0 = tout
        history_all = self.emotions.get_emotion_history(limit=0)
        self.assertGreaterEqual(len(history_all), 4)

    def test_random_emotion(self) -> None:
        """Test émotion aléatoire."""
        original = self.emotions.current_emotion
        result = self.emotions.random_emotion()

        # Devrait retourner une émotion différente ou la même si seule option
        self.assertIn(result, self.emotions.emotions.keys())

        # Si seule émotion disponible, devrait rester la même
        available = list(self.emotions.emotions.keys())
        if len(available) == 1:
            self.assertEqual(result, original)

    def test_emotional_response_compliment(self) -> None:
        """Test réponse émotionnelle à compliment."""
        result = self.emotions.emotional_response("compliment")
        self.assertIn(result, ["happy", "excited"])
        self.assertIn(self.emotions.current_emotion, ["happy", "excited"])

    def test_emotional_response_insult(self) -> None:
        """Test réponse émotionnelle à insulte."""
        result = self.emotions.emotional_response("insult")
        self.assertIn(result, ["angry", "sad"])

    def test_emotional_response_surprise(self) -> None:
        """Test réponse émotionnelle à surprise."""
        result = self.emotions.emotional_response("surprise")
        self.assertIn(result, ["surprised", "curious"])

    def test_emotional_response_unknown(self) -> None:
        """Test réponse émotionnelle à stimulus inconnu."""
        result = self.emotions.emotional_response("xyz_unknown_stimulus")
        self.assertEqual(result, "curious")
        self.assertEqual(self.emotions.current_emotion, "curious")

    def test_emotional_response_case_insensitive(self) -> None:
        """Test réponse émotionnelle insensible à la casse."""
        result_upper = self.emotions.emotional_response("COMPLIMENT")
        result_lower = self.emotions.emotional_response("compliment")
        # Devrait être cohérent
        self.assertIn(result_upper, ["happy", "excited"])

    def test_transition_preserves_previous(self) -> None:
        """Test que transition préserve émotion précédente."""
        self.emotions.set_emotion("happy", 0.8)
        self.emotions.set_emotion("sad", 0.6)

        history = self.emotions.get_emotion_history()
        last = history[-1]
        self.assertEqual(last["previous"], "happy")
        self.assertEqual(last["emotion"], "sad")

    def test_all_emotions_defined(self) -> None:
        """Test que toutes les émotions ont structure complète."""
        for _emotion_name, emotion_data in self.emotions.emotions.items():
            self.assertIn("yeux", emotion_data)
            self.assertIn("antennes", emotion_data)
            self.assertIn("tete", emotion_data)
            self.assertIn("description", emotion_data)
            self.assertIn("color", emotion_data)


if __name__ == "__main__":
    unittest.main()
