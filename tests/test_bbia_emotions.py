#!/usr/bin/env python3
"""Tests pour le module BBIA Emotions."""

from src.bbia_sim.bbia_emotions import BBIAEmotions


class TestBBIAEmotions:
    """Tests pour BBIAEmotions."""

    def test_emotions_creation(self):
        """Test création d'une instance BBIAEmotions."""
        emotions = BBIAEmotions()
        assert emotions.current_emotion == "neutral"
        assert emotions.emotion_intensity == 0.5
        assert emotions.transition_duration == 1.0

    def test_set_emotion(self):
        """Test changement d'émotion."""
        emotions = BBIAEmotions()

        # Test émotion valide
        emotions.set_emotion("happy", 0.8)
        assert emotions.current_emotion == "happy"
        assert emotions.emotion_intensity == 0.8

        # Test émotion avec intensité par défaut
        emotions.set_emotion("sad")
        assert emotions.current_emotion == "sad"
        assert emotions.emotion_intensity == 0.5

    def test_get_emotion_info(self):
        """Test récupération d'informations sur l'émotion."""
        emotions = BBIAEmotions()

        # Test émotion neutre
        info = emotions.emotions["neutral"]
        assert info["description"] == "État de repos, attention normale"
        assert info["color"] == "⚪"

        # Test émotion heureuse
        info = emotions.emotions["happy"]
        assert info["description"] == "Joie, satisfaction, bien-être"
        assert info["color"] == "😊"

    def test_get_available_emotions(self):
        """Test récupération des émotions disponibles."""
        emotions = BBIAEmotions()
        available = list(emotions.emotions.keys())

        expected_emotions = [
            "neutral",
            "happy",
            "sad",
            "angry",
            "curious",
            "excited",
            "surprised",
            "fearful",
        ]

        assert len(available) == len(expected_emotions)
        for emotion in expected_emotions:
            assert emotion in available

    def test_emotion_history(self):
        """Test historique des émotions."""
        emotions = BBIAEmotions()

        # Changer plusieurs émotions
        emotions.set_emotion("happy", 0.8)
        emotions.set_emotion("sad", 0.6)
        emotions.set_emotion("neutral", 0.5)

        # Vérifier l'historique
        history = emotions.get_emotion_history()
        assert len(history) >= 3

        # Vérifier la dernière émotion
        assert history[-1]["emotion"] == "neutral"
        assert history[-1]["intensity"] == 0.5

    def test_transition_smooth(self):
        """Test transition fluide entre émotions."""
        emotions = BBIAEmotions()

        # Test changement de durée de transition
        emotions.transition_duration = 0.5
        assert emotions.transition_duration == 0.5

        # Test transition normale
        emotions.set_emotion("sad", 0.6)
        assert emotions.current_emotion == "sad"
        assert emotions.emotion_intensity == 0.6
