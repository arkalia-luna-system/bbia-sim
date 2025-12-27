#!/usr/bin/env python3

"""Tests étendus pour BBIA Emotions
Tests ciblés pour améliorer la couverture de code.
"""

from datetime import datetime
from unittest.mock import patch

from bbia_sim.bbia_emotions import BBIAEmotions


class TestBBIAEmotionsExtended:
    """Tests étendus pour BBIAEmotions."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.emotions = BBIAEmotions()

    def test_init_defaults(self):
        """Test initialisation avec valeurs par défaut."""
        assert self.emotions.current_emotion == "neutral"
        assert self.emotions.emotion_intensity == 0.5
        assert self.emotions.transition_duration == 1.0
        # emotion_history est un deque, pas une liste
        assert len(self.emotions.emotion_history) == 0

    def test_init_emotions_structure(self):
        """Test structure des émotions."""
        assert len(self.emotions.emotions) == 12
        assert "neutral" in self.emotions.emotions
        assert "happy" in self.emotions.emotions
        assert "sad" in self.emotions.emotions
        assert "angry" in self.emotions.emotions
        assert "curious" in self.emotions.emotions
        assert "excited" in self.emotions.emotions
        assert "surprised" in self.emotions.emotions
        assert "fearful" in self.emotions.emotions

    def test_emotion_data_structure(self):
        """Test structure des données d'émotion."""
        emotion_data = self.emotions.emotions["happy"]
        assert "yeux" in emotion_data
        assert "antennes" in emotion_data
        assert "tete" in emotion_data
        assert "description" in emotion_data
        assert "color" in emotion_data

    @patch("builtins.print")
    def test_set_emotion_valid(self, mock_print):
        """Test changement d'émotion valide."""
        result = self.emotions.set_emotion("happy", 0.8)

        assert result is True
        assert self.emotions.current_emotion == "happy"
        assert self.emotions.emotion_intensity == 0.8
        assert len(self.emotions.emotion_history) == 1

    @patch("builtins.print")
    def test_set_emotion_invalid(self, mock_print):
        """Test changement d'émotion invalide."""
        result = self.emotions.set_emotion("invalid_emotion", 0.5)

        assert result is False
        assert self.emotions.current_emotion == "neutral"  # Reste inchangé
        assert len(self.emotions.emotion_history) == 0

    @patch("builtins.print")
    def test_set_emotion_intensity_clamping(self, mock_print):
        """Test limitation de l'intensité."""
        # Test intensité trop élevée
        self.emotions.set_emotion("happy", 1.5)
        assert self.emotions.emotion_intensity == 1.0

        # Test intensité trop faible
        self.emotions.set_emotion("sad", -0.5)
        assert self.emotions.emotion_intensity == 0.0

    @patch("builtins.print")
    def test_set_emotion_history(self, mock_print):
        """Test historique des émotions."""
        self.emotions.set_emotion("happy", 0.7)
        self.emotions.set_emotion("sad", 0.6)

        assert len(self.emotions.emotion_history) == 2
        assert self.emotions.emotion_history[0]["emotion"] == "happy"
        assert self.emotions.emotion_history[0]["intensity"] == 0.7
        assert self.emotions.emotion_history[0]["previous"] == "neutral"
        assert self.emotions.emotion_history[1]["emotion"] == "sad"
        assert self.emotions.emotion_history[1]["intensity"] == 0.6
        assert self.emotions.emotion_history[1]["previous"] == "happy"

    def test_display_emotion_transition(self):
        """Test affichage de transition d'émotion."""
        # Capturer les logs au lieu de mock print (la fonction utilise logger.info)
        import io
        import logging

        log_capture = io.StringIO()
        handler = logging.StreamHandler(log_capture)
        handler.setLevel(logging.INFO)

        # Obtenir le logger utilisé par bbia_emotions
        logger = logging.getLogger("bbia_sim.bbia_emotions")
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)

        try:
            self.emotions._display_emotion_transition("neutral", "happy")

            # Vérifier que les logs contiennent les messages attendus
            log_output = log_capture.getvalue()
            assert "Transition d'émotion" in log_output
            assert "neutral" in log_output or "happy" in log_output
        finally:
            logger.removeHandler(handler)

    def test_get_current_emotion(self):
        """Test récupération de l'émotion actuelle."""
        emotion_data = self.emotions.get_current_emotion()

        assert emotion_data["name"] == "neutral"
        assert emotion_data["intensity"] == "0.5"
        assert "timestamp" in emotion_data
        assert "yeux" in emotion_data
        assert "antennes" in emotion_data
        assert "tete" in emotion_data
        assert "description" in emotion_data
        assert "color" in emotion_data

    def test_get_emotion_history_limit(self):
        """Test historique avec limite."""
        # Ajouter plusieurs émotions
        self.emotions.set_emotion("happy", 0.7)
        self.emotions.set_emotion("sad", 0.6)
        self.emotions.set_emotion("angry", 0.8)

        # Test avec limite
        history = self.emotions.get_emotion_history(2)
        assert len(history) == 2
        assert history[0]["emotion"] == "sad"
        assert history[1]["emotion"] == "angry"

    def test_get_emotion_history_no_limit(self):
        """Test historique sans limite."""
        self.emotions.set_emotion("happy", 0.7)
        self.emotions.set_emotion("sad", 0.6)

        history = self.emotions.get_emotion_history(0)
        assert len(history) == 2

    @patch("secrets.choice")
    @patch("secrets.randbelow")
    @patch("builtins.print")
    def test_random_emotion_success(self, mock_print, mock_randbelow, mock_choice):
        """Test émotion aléatoire réussie."""
        mock_choice.return_value = "happy"
        mock_randbelow.return_value = 40  # 40/100 + 0.3 = 0.7

        result = self.emotions.random_emotion()

        assert result == "happy"
        assert self.emotions.current_emotion == "happy"
        assert self.emotions.emotion_intensity == 0.7

    @patch("builtins.print")
    def test_random_emotion_no_available(self, mock_print):
        """Test émotion aléatoire sans émotions disponibles."""
        # Simuler une liste vide d'émotions disponibles
        with patch.object(self.emotions, "emotions", {"neutral": {}}):
            result = self.emotions.random_emotion()
            assert result == "neutral"  # Retourne l'émotion actuelle

    @patch("secrets.choice")
    @patch("secrets.randbelow")
    @patch("builtins.print")
    def test_emotional_response_compliment(
        self, mock_print, mock_randbelow, mock_choice
    ):
        """Test réponse émotionnelle à un compliment."""
        mock_choice.return_value = "happy"
        mock_randbelow.return_value = 50  # 50/100 + 0.4 = 0.9

        result = self.emotions.emotional_response("compliment")

        assert result in [
            "happy",
            "excited",
        ]  # Peut être happy ou excited selon le hasard
        assert self.emotions.current_emotion in ["happy", "excited"]

    @patch("secrets.choice")
    @patch("secrets.randbelow")
    @patch("builtins.print")
    def test_emotional_response_insult(self, mock_print, mock_randbelow, mock_choice):
        """Test réponse émotionnelle à une insulte."""
        mock_choice.return_value = "angry"
        mock_randbelow.return_value = 50  # 50/100 + 0.4 = 0.9

        result = self.emotions.emotional_response("insult")

        assert result == "angry"
        assert self.emotions.current_emotion == "angry"

    @patch("secrets.choice")
    @patch("secrets.randbelow")
    @patch("builtins.print")
    def test_emotional_response_unknown(self, mock_print, mock_randbelow, mock_choice):
        """Test réponse émotionnelle à un stimulus inconnu."""
        mock_choice.return_value = "curious"
        mock_randbelow.return_value = 10  # 10/100 + 0.4 = 0.5

        result = self.emotions.emotional_response("unknown_stimulus")

        assert result == "curious"
        assert self.emotions.current_emotion == "curious"

    @patch("builtins.print")
    def test_emotional_response_case_insensitive(self, mock_print):
        """Test réponse émotionnelle insensible à la casse."""
        result = self.emotions.emotional_response("COMPLIMENT")

        assert result in ["happy", "excited"]

    @patch("builtins.print")
    def test_blend_emotions_valid(self, mock_print):
        """Test mélange d'émotions valides."""
        result = self.emotions.blend_emotions("happy", "excited", 0.3)

        assert result == "happy"  # ratio < 0.5
        assert self.emotions.current_emotion == "happy"

    @patch("builtins.print")
    def test_blend_emotions_ratio_high(self, mock_print):
        """Test mélange d'émotions avec ratio élevé."""
        result = self.emotions.blend_emotions("happy", "excited", 0.7)

        assert result == "excited"  # ratio >= 0.5
        assert self.emotions.current_emotion == "excited"

    @patch("builtins.print")
    def test_blend_emotions_invalid_first(self, mock_print):
        """Test mélange avec première émotion invalide."""
        result = self.emotions.blend_emotions("invalid", "happy", 0.5)

        assert result == "neutral"  # Retourne l'émotion actuelle

    @patch("builtins.print")
    def test_blend_emotions_invalid_second(self, mock_print):
        """Test mélange avec seconde émotion invalide."""
        result = self.emotions.blend_emotions("happy", "invalid", 0.5)

        assert result == "neutral"  # Retourne l'émotion actuelle

    def test_blend_emotions_intensity_calculation(self):
        """Test calcul d'intensité dans le mélange."""
        self.emotions.blend_emotions("happy", "excited", 0.8)

        # Intensité = 0.5 + (abs(0.8 - 0.5) * 0.5) = 0.5 + 0.15 = 0.65
        expected_intensity = 0.5 + (abs(0.8 - 0.5) * 0.5)
        assert self.emotions.emotion_intensity == expected_intensity

    def test_get_emotion_stats_empty(self):
        """Test statistiques avec historique vide."""
        stats = self.emotions.get_emotion_stats()

        assert stats["current_emotion"] == "neutral"
        assert stats["current_intensity"] == 0.5
        assert stats["total_transitions"] == 0
        assert stats["emotion_counts"] == {}
        assert len(stats["available_emotions"]) == 12

    def test_get_emotion_stats_with_history(self):
        """Test statistiques avec historique."""
        self.emotions.set_emotion("happy", 0.7)
        self.emotions.set_emotion("sad", 0.6)
        self.emotions.set_emotion("happy", 0.8)

        stats = self.emotions.get_emotion_stats()

        assert stats["current_emotion"] == "happy"
        assert stats["current_intensity"] == 0.8
        assert stats["total_transitions"] == 3
        assert stats["emotion_counts"]["happy"] == 2
        assert stats["emotion_counts"]["sad"] == 1

    @patch("builtins.print")
    def test_reset_emotions(self, mock_print):
        """Test remise à zéro des émotions."""
        self.emotions.set_emotion("happy", 0.8)
        self.emotions.set_emotion("sad", 0.6)

        self.emotions.reset_emotions()

        assert self.emotions.current_emotion == "neutral"
        assert self.emotions.emotion_intensity == 0.5
        assert len(self.emotions.emotion_history) == 0

    def test_emotion_history_timestamp(self):
        """Test timestamp dans l'historique."""
        self.emotions.set_emotion("happy", 0.7)

        entry = self.emotions.emotion_history[0]
        assert "timestamp" in entry
        # Vérifier que c'est un format ISO valide
        datetime.fromisoformat(entry["timestamp"])

    def test_emotional_response_all_stimuli(self):
        """Test toutes les réponses émotionnelles."""
        stimuli = [
            "compliment",
            "insult",
            "surprise",
            "danger",
            "question",
            "greeting",
            "goodbye",
            "achievement",
            "failure",
        ]

        for stimulus in stimuli:
            result = self.emotions.emotional_response(stimulus)
            assert result in self.emotions.emotions

    def test_emotion_intensity_range(self):
        """Test plage d'intensité des émotions."""
        # Test limites
        self.emotions.set_emotion("happy", 0.0)
        assert self.emotions.emotion_intensity == 0.0

        self.emotions.set_emotion("sad", 1.0)
        assert self.emotions.emotion_intensity == 1.0

        # Test valeurs intermédiaires
        self.emotions.set_emotion("angry", 0.5)
        assert self.emotions.emotion_intensity == 0.5

    def test_emotion_transition_chain(self):
        """Test chaîne de transitions d'émotions."""
        emotions_chain = ["neutral", "happy", "sad", "angry", "curious"]

        for emotion in emotions_chain[1:]:  # Skip neutral (déjà défini)
            self.emotions.set_emotion(emotion, 0.6)
            assert self.emotions.current_emotion == emotion

    def test_emotion_history_previous_tracking(self):
        """Test suivi de l'émotion précédente."""
        self.emotions.set_emotion("happy", 0.7)
        self.emotions.set_emotion("sad", 0.6)
        self.emotions.set_emotion("angry", 0.8)

        history = self.emotions.emotion_history
        assert history[0]["previous"] == "neutral"
        assert history[1]["previous"] == "happy"
        assert history[2]["previous"] == "sad"
