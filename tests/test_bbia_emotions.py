#!/usr/bin/env python3
"""Tests pour le module BBIA Emotions."""

# IMPORTANT: Import direct (pas dans try/except) pour que coverage le détecte
import bbia_sim.bbia_emotions  # noqa: F401

from bbia_sim.bbia_emotions import BBIAEmotions


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
            "confused",
            "determined",
            "nostalgic",
            "proud",
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

    def test_reset_emotions(self):
        """Test remise à zéro des émotions."""
        emotions = BBIAEmotions()

        # Changer plusieurs émotions
        emotions.set_emotion("happy", 0.8)
        emotions.set_emotion("sad", 0.6)
        assert len(emotions.emotion_history) == 2

        # Reset
        emotions.reset_emotions()

        # Vérifier reset
        assert emotions.current_emotion == "neutral"
        assert emotions.emotion_intensity == 0.5
        assert len(emotions.emotion_history) == 0

    def test_set_emotion_invalid(self):
        """Test changement d'émotion invalide."""
        emotions = BBIAEmotions()
        original_emotion = emotions.current_emotion

        # Test émotion invalide
        result = emotions.set_emotion("invalid_emotion", 0.8)
        assert result is False
        assert emotions.current_emotion == original_emotion
        assert len(emotions.emotion_history) == 0

    def test_set_emotion_intensity_clamping(self):
        """Test limitation de l'intensité (clamping)."""
        emotions = BBIAEmotions()

        # Test intensité trop élevée
        emotions.set_emotion("happy", 1.5)
        assert emotions.emotion_intensity == 1.0

        # Test intensité trop faible
        emotions.set_emotion("sad", -0.5)
        assert emotions.emotion_intensity == 0.0

    def test_emotional_response_unknown_stimulus(self):
        """Test réponse émotionnelle à stimulus inconnu (ligne 197-198)."""
        emotions = BBIAEmotions()

        # Test avec un stimulus qui ne matche aucun pattern connu
        # (éviter "unknown" car c'est un pattern reconnu)
        result = emotions.emotional_response("xyz_completely_random_stimulus_12345")

        # Devrait retourner "curious" par défaut (ligne 197-198) si aucun pattern ne matche
        # Note: Le code vérifie si key in stimulus_lower, donc on évite tous les mots-clés
        assert result == "curious"
        assert emotions.current_emotion == "curious"
        assert emotions.emotion_intensity == 0.5

    def test_emotion_rapid_sequences(self):
        """Test transitions rapides (happy → sad → excited en < 1 seconde) (Issue #6)."""
        emotions = BBIAEmotions()

        # Changer émotions rapidement
        emotions.set_emotion("happy", 0.8)
        emotions.set_emotion("sad", 0.6)
        emotions.set_emotion("excited", 0.9)

        # Vérifier que l'historique contient les 3 transitions
        history = emotions.get_emotion_history()
        assert len(history) >= 3
        assert history[-3]["emotion"] == "happy"
        assert history[-2]["emotion"] == "sad"
        assert history[-1]["emotion"] == "excited"
        assert emotions.current_emotion == "excited"

    def test_emotion_transition_different_durations(self):
        """Test transitions avec durées différentes (Issue #6)."""
        emotions = BBIAEmotions()

        # Transition rapide
        emotions.transition_duration = 0.1
        emotions.set_emotion("happy", 0.8)
        assert emotions.transition_duration == 0.1
        assert emotions.current_emotion == "happy"

        # Transition lente
        emotions.transition_duration = 2.0
        emotions.set_emotion("sad", 0.6)
        assert emotions.transition_duration == 2.0
        assert emotions.current_emotion == "sad"

    def test_emotion_stress_multiple_transitions(self):
        """Test stress : 10+ transitions successives rapides (Issue #6)."""
        emotions = BBIAEmotions()

        # 15 transitions successives rapides
        emotion_list = ["happy", "sad", "excited", "neutral", "curious"]
        for i in range(15):
            emotion = emotion_list[i % len(emotion_list)]
            intensity = 0.5 + (i % 5) * 0.1
            emotions.set_emotion(emotion, intensity)

        # Vérifier que l'historique contient au moins 10 transitions (peut être limité)
        history = emotions.get_emotion_history()
        assert (
            len(history) >= 10
        ), f"Historique doit contenir au moins 10 transitions, a {len(history)}"

        # Vérifier que la dernière émotion est correcte
        assert emotions.current_emotion in emotion_list
        assert emotions.current_emotion == emotion_list[14 % len(emotion_list)]

    def test_emotion_extreme_intensities(self):
        """Test transitions avec intensités extrêmes (0.0 → 1.0 → 0.0) (Issue #6)."""
        emotions = BBIAEmotions()

        # Intensité minimale
        emotions.set_emotion("happy", 0.0)
        assert emotions.emotion_intensity == 0.0
        assert emotions.current_emotion == "happy"

        # Intensité maximale
        emotions.set_emotion("excited", 1.0)
        assert emotions.emotion_intensity == 1.0
        assert emotions.current_emotion == "excited"

        # Retour à minimale avec une émotion valide (neutral au lieu de calm qui n'existe pas)
        emotions.set_emotion("neutral", 0.0)
        assert emotions.emotion_intensity == 0.0
        assert emotions.current_emotion == "neutral"
