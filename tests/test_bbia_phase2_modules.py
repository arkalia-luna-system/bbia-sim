#!/usr/bin/env python3
"""
Tests pour les modules BBIA Phase 2 - Hugging Face, Emotion Recognition, Adaptive Behavior
"""

import sys
from pathlib import Path

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_huggingface import BBIAHuggingFace
from bbia_sim.bbia_emotion_recognition import BBIAEmotionRecognition
from bbia_sim.bbia_adaptive_behavior import BBIAAdaptiveBehavior


class TestBBIAHuggingFace:
    """Tests pour le module BBIA Hugging Face."""
    
    def test_initialization(self):
        """Test l'initialisation du module."""
        try:
            hf = BBIAHuggingFace()
            assert hf.device is not None
            assert hf.model_configs is not None
            assert len(hf.model_configs) > 0
        except ImportError:
            pytest.skip("Hugging Face transformers non disponible")
    
    def test_device_detection(self):
        """Test la détection automatique du device."""
        try:
            hf = BBIAHuggingFace(device="auto")
            assert hf.device in ["cpu", "cuda", "mps"]
        except ImportError:
            pytest.skip("Hugging Face transformers non disponible")
    
    def test_model_configs(self):
        """Test la configuration des modèles."""
        try:
            hf = BBIAHuggingFace()
            configs = hf.model_configs
            
            assert "vision" in configs
            assert "audio" in configs
            assert "nlp" in configs
            assert "multimodal" in configs
            
            # Vérification des modèles recommandés
            assert "clip" in configs["vision"]
            assert "blip" in configs["vision"]
            assert "whisper" in configs["audio"]
            assert "sentiment" in configs["nlp"]
            assert "emotion" in configs["nlp"]
        except ImportError:
            pytest.skip("Hugging Face transformers non disponible")
    
    def test_get_available_models(self):
        """Test la récupération des modèles disponibles."""
        try:
            hf = BBIAHuggingFace()
            models = hf.get_available_models()
            
            assert isinstance(models, dict)
            assert len(models) > 0
        except ImportError:
            pytest.skip("Hugging Face transformers non disponible")
    
    def test_get_loaded_models(self):
        """Test la récupération des modèles chargés."""
        try:
            hf = BBIAHuggingFace()
            loaded = hf.get_loaded_models()
            
            assert isinstance(loaded, list)
            assert len(loaded) == 0  # Aucun modèle chargé initialement
        except ImportError:
            pytest.skip("Hugging Face transformers non disponible")
    
    def test_model_info(self):
        """Test les informations sur les modèles."""
        try:
            hf = BBIAHuggingFace()
            info = hf.get_model_info()
            
            assert "device" in info
            assert "loaded_models" in info
            assert "available_models" in info
            assert "hf_available" in info
        except ImportError:
            pytest.skip("Hugging Face transformers non disponible")


class TestBBIAEmotionRecognition:
    """Tests pour le module BBIA Emotion Recognition."""
    
    def test_initialization(self):
        """Test l'initialisation du module."""
        try:
            emotion_rec = BBIAEmotionRecognition()
            assert emotion_rec.device is not None
            assert emotion_rec.supported_emotions is not None
            assert len(emotion_rec.supported_emotions) > 0
        except ImportError:
            pytest.skip("Dépendances ML non disponibles")
    
    def test_supported_emotions(self):
        """Test les émotions supportées."""
        try:
            emotion_rec = BBIAEmotionRecognition()
            emotions = emotion_rec.supported_emotions
            
            expected_emotions = [
                "happy", "sad", "angry", "surprised", "fearful", 
                "disgusted", "neutral", "excited", "calm", "confused"
            ]
            
            for emotion in expected_emotions:
                assert emotion in emotions
        except ImportError:
            pytest.skip("Dépendances ML non disponibles")
    
    def test_detection_config(self):
        """Test la configuration de détection."""
        try:
            emotion_rec = BBIAEmotionRecognition()
            config = emotion_rec.detection_config
            
            assert "face_detection_confidence" in config
            assert "emotion_confidence_threshold" in config
            assert "temporal_window_size" in config
            assert "fusion_weights" in config
            
            # Vérification des valeurs
            assert 0.0 <= config["face_detection_confidence"] <= 1.0
            assert 0.0 <= config["emotion_confidence_threshold"] <= 1.0
            assert config["temporal_window_size"] > 0
        except ImportError:
            pytest.skip("Dépendances ML non disponibles")
    
    def test_analyze_vocal_emotion(self):
        """Test l'analyse d'émotion vocale."""
        try:
            emotion_rec = BBIAEmotionRecognition()
            
            # Test avec texte positif
            result = emotion_rec.analyze_vocal_emotion("Je suis très heureux!")
            assert isinstance(result, dict)
            
            # Test avec texte négatif
            result = emotion_rec.analyze_vocal_emotion("Je suis triste et déprimé.")
            assert isinstance(result, dict)
        except ImportError:
            pytest.skip("Dépendances ML non disponibles")
    
    def test_fuse_emotions(self):
        """Test la fusion d'émotions."""
        try:
            emotion_rec = BBIAEmotionRecognition()
            
            facial_result = {
                "emotion": "happy",
                "confidence": 0.8
            }
            
            vocal_result = {
                "emotion": "excited",
                "confidence": 0.7
            }
            
            fused = emotion_rec.fuse_emotions(facial_result, vocal_result)
            assert isinstance(fused, dict)
            assert "emotion" in fused
            assert "confidence" in fused
        except ImportError:
            pytest.skip("Dépendances ML non disponibles")
    
    def test_emotion_history(self):
        """Test l'historique des émotions."""
        try:
            emotion_rec = BBIAEmotionRecognition()
            
            # Ajout d'émotions à l'historique
            emotion_rec.emotion_history = [
                {"emotion": "happy", "confidence": 0.8},
                {"emotion": "excited", "confidence": 0.7},
                {"emotion": "happy", "confidence": 0.9}
            ]
            
            stats = emotion_rec.get_emotion_statistics()
            assert isinstance(stats, dict)
            assert "total_analyses" in stats
            assert "emotion_distribution" in stats
        except ImportError:
            pytest.skip("Dépendances ML non disponibles")
    
    def test_reset_history(self):
        """Test la réinitialisation de l'historique."""
        try:
            emotion_rec = BBIAEmotionRecognition()
            emotion_rec.emotion_history = [{"emotion": "happy", "confidence": 0.8}]
            
            emotion_rec.reset_history()
            assert len(emotion_rec.emotion_history) == 0
        except ImportError:
            pytest.skip("Dépendances ML non disponibles")


class TestBBIAAdaptiveBehavior:
    """Tests pour le module BBIA Adaptive Behavior."""
    
    def test_initialization(self):
        """Test l'initialisation du module."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        
        assert adaptive_behavior.current_context == "neutral"
        assert adaptive_behavior.current_emotion == "neutral"
        assert adaptive_behavior.emotion_intensity == 0.5
        assert len(adaptive_behavior.contexts) > 0
        assert len(adaptive_behavior.behaviors) > 0
    
    def test_contexts(self):
        """Test les contextes disponibles."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        contexts = adaptive_behavior.contexts
        
        expected_contexts = [
            "greeting", "conversation", "attention", "rest", 
            "playful", "serious", "sleepy"
        ]
        
        for context in expected_contexts:
            assert context in contexts
            assert "priority" in contexts[context]
            assert "duration" in contexts[context]
            assert "emotions" in contexts[context]
    
    def test_behaviors(self):
        """Test les comportements disponibles."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        behaviors = adaptive_behavior.behaviors
        
        expected_behaviors = [
            "nod", "shake", "look_around", "focus", "stretch", 
            "dance", "hide", "celebrate"
        ]
        
        for behavior in expected_behaviors:
            assert behavior in behaviors
            assert "description" in behaviors[behavior]
            assert "emotions" in behaviors[behavior]
            assert "duration" in behaviors[behavior]
            assert "intensity_range" in behaviors[behavior]
            assert "joints" in behaviors[behavior]
    
    def test_set_context(self):
        """Test le changement de contexte."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        
        # Test contexte valide
        result = adaptive_behavior.set_context("greeting", 0.9)
        assert result is True
        assert adaptive_behavior.current_context == "greeting"
        
        # Test contexte invalide
        result = adaptive_behavior.set_context("invalid_context")
        assert result is False
    
    def test_set_emotion_state(self):
        """Test le changement d'état émotionnel."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        
        # Test émotion valide
        result = adaptive_behavior.set_emotion_state("happy", 0.8)
        assert result is True
        assert adaptive_behavior.current_emotion == "happy"
        assert adaptive_behavior.emotion_intensity == 0.8
        
        # Test intensité limitée
        adaptive_behavior.set_emotion_state("sad", 1.5)
        assert adaptive_behavior.emotion_intensity == 1.0
        
        adaptive_behavior.set_emotion_state("angry", -0.5)
        assert adaptive_behavior.emotion_intensity == 0.0
    
    def test_generate_behavior(self):
        """Test la génération de comportement."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        
        # Configuration d'un contexte et émotion
        adaptive_behavior.set_context("greeting")
        adaptive_behavior.set_emotion_state("happy", 0.8)
        
        # Génération de comportement
        behavior = adaptive_behavior.generate_behavior("user_arrival")
        
        assert isinstance(behavior, dict)
        assert "name" in behavior
        assert "description" in behavior
        assert "context" in behavior
        assert "emotion" in behavior
        assert "parameters" in behavior
        assert "timestamp" in behavior
        assert "id" in behavior
    
    def test_get_suitable_behaviors(self):
        """Test la sélection de comportements adaptés."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        
        # Test avec émotion happy
        adaptive_behavior.set_emotion_state("happy", 0.8)
        suitable = adaptive_behavior._get_suitable_behaviors()
        
        assert isinstance(suitable, list)
        assert len(suitable) > 0
        
        # Vérification que les comportements sont adaptés à l'émotion
        for behavior_name in suitable:
            behavior_config = adaptive_behavior.behaviors[behavior_name]
            assert "happy" in behavior_config["emotions"]
    
    def test_user_preferences(self):
        """Test les préférences utilisateur."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        
        # Génération de plusieurs comportements
        adaptive_behavior.set_context("playful")
        adaptive_behavior.set_emotion_state("excited", 0.9)
        
        for _ in range(5):
            adaptive_behavior.generate_behavior("test")
        
        # Vérification des préférences
        preferences = adaptive_behavior.user_preferences
        assert "preferred_behaviors" in preferences
        assert "preferred_emotions" in preferences
    
    def test_adapt_to_feedback(self):
        """Test l'adaptation basée sur les retours."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        
        # Génération d'un comportement
        behavior = adaptive_behavior.generate_behavior("test")
        behavior_id = behavior["id"]
        
        # Test adaptation positive
        adaptive_behavior.adapt_to_feedback(behavior_id, "positive", 0.8)
        
        # Test adaptation négative
        adaptive_behavior.adapt_to_feedback(behavior_id, "negative", 0.6)
        
        # Test adaptation neutre
        adaptive_behavior.adapt_to_feedback(behavior_id, "neutral", 0.5)
    
    def test_behavior_statistics(self):
        """Test les statistiques de comportement."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        
        # Génération de plusieurs comportements
        for context in ["greeting", "conversation", "playful"]:
            adaptive_behavior.set_context(context)
            adaptive_behavior.set_emotion_state("happy", 0.7)
            adaptive_behavior.generate_behavior(f"test_{context}")
        
        stats = adaptive_behavior.get_behavior_statistics()
        
        assert isinstance(stats, dict)
        assert "total_behaviors" in stats
        assert "behavior_distribution" in stats
        assert "emotion_distribution" in stats
        assert "context_distribution" in stats
        assert "user_preferences" in stats
        assert "current_state" in stats
    
    def test_reset_learning(self):
        """Test la réinitialisation de l'apprentissage."""
        adaptive_behavior = BBIAAdaptiveBehavior()
        
        # Configuration d'un contexte et émotion valides
        adaptive_behavior.set_context("greeting")
        adaptive_behavior.set_emotion_state("happy", 0.8)
        
        # Génération de comportements
        behavior = adaptive_behavior.generate_behavior("test")
        
        # Vérification que l'historique n'est pas vide
        assert len(adaptive_behavior.behavior_history) > 0
        
        # Réinitialisation
        adaptive_behavior.reset_learning()
        
        # Vérification que tout est réinitialisé
        assert len(adaptive_behavior.behavior_history) == 0
        assert len(adaptive_behavior.user_preferences["preferred_behaviors"]) == 0
        assert len(adaptive_behavior.user_preferences["preferred_emotions"]) == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
