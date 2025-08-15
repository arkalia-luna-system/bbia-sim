#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BBIA Emotions - Module d'émotions avancé pour Reachy Mini Wireless
Émotions complexes, expressions faciales, transitions fluides
"""

import random
from typing import Dict, List
from datetime import datetime


class BBIAEmotions:
    """Module d'émotions avancé pour BBIA"""

    def __init__(self):
        self.current_emotion = "neutral"
        self.emotion_intensity = 0.5  # 0.0 à 1.0
        self.transition_duration = 1.0  # secondes
        self.emotion_history = []

        # Définition des émotions basées sur la référence visuelle
        self.emotions = {
            "neutral": {
                "yeux": "Cercles noirs normaux",
                "antennes": "Droites, calmes",
                "tete": "Position neutre",
                "description": "État de repos, attention normale",
                "color": "⚪",
            },
            "happy": {
                "yeux": "Cercles légèrement agrandis",
                "antennes": "Légèrement relevées",
                "tete": "Relevée, regard joyeux",
                "description": "Joie, satisfaction, bien-être",
                "color": "😊",
            },
            "sad": {
                "yeux": "Cercles plus petits",
                "antennes": "Tombantes",
                "tete": "Baissée, regard triste",
                "description": "Tristesse, mélancolie, déception",
                "color": "😢",
            },
            "angry": {
                "yeux": "Cercles plus intenses",
                "antennes": "Rigides",
                "tete": "Penchée, regard dur",
                "description": "Colère, frustration, irritation",
                "color": "😠",
            },
            "curious": {
                "yeux": "Cercles inclinés",
                "antennes": "Frémissantes",
                "tete": "Inclinée, regard attentif",
                "description": "Curiosité, intérêt, attention",
                "color": "🤔",
            },
            "excited": {
                "yeux": "Cercles vibrants",
                "antennes": "Vibrantes",
                "tete": "Relevée, regard enthousiaste",
                "description": "Excitation, enthousiasme, énergie",
                "color": "🤩",
            },
            "surprised": {
                "yeux": "Cercles très agrandis",
                "antennes": "Dressées",
                "tete": "Relevée, regard étonné",
                "description": "Surprise, étonnement, choc",
                "color": "😲",
            },
            "fearful": {
                "yeux": "Cercles tremblants",
                "antennes": "Tremblantes",
                "tete": "Reculée, regard craintif",
                "description": "Peur, anxiété, inquiétude",
                "color": "😨",
            },
        }

        print("🎭 BBIA Emotions initialisé")
        print(f"   • Émotion actuelle : {self.current_emotion}")
        print(f"   • Intensité : {self.emotion_intensity}")
        print(f"   • Émotions disponibles : {len(self.emotions)}")

    def set_emotion(self, emotion: str, intensity: float = 0.5) -> bool:
        """Change l'émotion de BBIA"""
        if emotion not in self.emotions:
            print(f"❌ Émotion inconnue : {emotion}")
            return False

        old_emotion = self.current_emotion
        self.current_emotion = emotion
        self.emotion_intensity = max(0.0, min(1.0, intensity))

        # Enregistrer dans l'historique
        self.emotion_history.append(
            {
                "emotion": emotion,
                "intensity": self.emotion_intensity,
                "timestamp": datetime.now().isoformat(),
                "previous": old_emotion,
            }
        )

        # Afficher la transition
        self._display_emotion_transition(old_emotion, emotion)

        return True

    def _display_emotion_transition(self, old_emotion: str, new_emotion: str):
        """Affiche la transition d'émotion"""
        old_data = self.emotions[old_emotion]
        new_data = self.emotions[new_emotion]

        print(f"\n🎭 Transition d'émotion : {old_emotion} → {new_emotion}")
        print(f"   {old_data['color']} {old_data['description']}")
        print(f"   ↓ (transition de {self.transition_duration}s)")
        print(f"   {new_data['color']} {new_data['description']}")

        print(f"\n📋 Détails de l'émotion {new_emotion} :")
        print(f"   • Yeux : {new_data['yeux']}")
        print(f"   • Antennes : {new_data['antennes']}")
        print(f"   • Tête : {new_data['tete']}")
        print(f"   • Intensité : {self.emotion_intensity*100:.0f}%")

    def get_current_emotion(self) -> Dict:
        """Retourne l'émotion actuelle avec ses détails"""
        emotion_data = self.emotions[self.current_emotion].copy()
        emotion_data.update(
            {
                "name": self.current_emotion,
                "intensity": self.emotion_intensity,
                "timestamp": datetime.now().isoformat(),
            }
        )
        return emotion_data

    def get_emotion_history(self, limit: int = 10) -> List[Dict]:
        """Retourne l'historique des émotions"""
        return self.emotion_history[-limit:] if limit > 0 else self.emotion_history

    def random_emotion(self) -> str:
        """Change vers une émotion aléatoire"""
        available_emotions = list(self.emotions.keys())
        available_emotions.remove(self.current_emotion)  # Éviter la même émotion

        if available_emotions:
            new_emotion = random.choice(available_emotions)
            intensity = random.uniform(0.3, 1.0)
            self.set_emotion(new_emotion, intensity)
            return new_emotion
        return self.current_emotion

    def emotional_response(self, stimulus: str) -> str:
        """Réponse émotionnelle à un stimulus"""
        responses = {
            "compliment": ["happy", "excited"],
            "insult": ["angry", "sad"],
            "surprise": ["surprised", "curious"],
            "danger": ["fearful", "angry"],
            "question": ["curious", "neutral"],
            "greeting": ["happy", "neutral"],
            "goodbye": ["sad", "neutral"],
            "achievement": ["excited", "happy"],
            "failure": ["sad", "angry"],
            "unknown": ["curious", "neutral"],
        }

        stimulus_lower = stimulus.lower()
        for key, emotions in responses.items():
            if key in stimulus_lower:
                emotion = random.choice(emotions)
                intensity = random.uniform(0.4, 0.9)
                self.set_emotion(emotion, intensity)
                return emotion

        # Réponse par défaut
        self.set_emotion("curious", 0.5)
        return "curious"

    def blend_emotions(self, emotion1: str, emotion2: str, ratio: float = 0.5) -> str:
        """Mélange deux émotions"""
        if emotion1 not in self.emotions or emotion2 not in self.emotions:
            print("❌ Une des émotions n'existe pas")
            return self.current_emotion

        # Logique simple de mélange
        if ratio < 0.5:
            result_emotion = emotion1
        else:
            result_emotion = emotion2

        intensity = 0.5 + (abs(ratio - 0.5) * 0.5)  # Intensité basée sur la différence
        self.set_emotion(result_emotion, intensity)

        print(f"🎨 Mélange d'émotions : {emotion1} + {emotion2} = {result_emotion}")
        return result_emotion

    def get_emotion_stats(self) -> Dict:
        """Retourne les statistiques des émotions"""
        emotion_counts = {}
        for entry in self.emotion_history:
            emotion = entry["emotion"]
            emotion_counts[emotion] = emotion_counts.get(emotion, 0) + 1

        return {
            "current_emotion": self.current_emotion,
            "current_intensity": self.emotion_intensity,
            "total_transitions": len(self.emotion_history),
            "emotion_counts": emotion_counts,
            "available_emotions": list(self.emotions.keys()),
        }

    def reset_emotions(self):
        """Remet BBIA en état neutre"""
        print("🔄 Remise à zéro des émotions")
        self.set_emotion("neutral", 0.5)
        self.emotion_history.clear()


def main():
    """Test du module BBIA Emotions"""
    print("🧪 Test du module BBIA Emotions")
    print("=" * 50)

    # Créer l'instance
    emotions = BBIAEmotions()

    # Test émotions de base
    print("\n1️⃣ Test émotions de base")
    emotions.set_emotion("happy", 0.8)
    emotions.set_emotion("curious", 0.6)
    emotions.set_emotion("sad", 0.7)

    # Test émotion aléatoire
    print("\n2️⃣ Test émotion aléatoire")
    random_emotion = emotions.random_emotion()
    print(f"Émotion aléatoire : {random_emotion}")

    # Test réponses émotionnelles
    print("\n3️⃣ Test réponses émotionnelles")
    emotions.emotional_response("compliment")
    emotions.emotional_response("question")
    emotions.emotional_response("danger")

    # Test mélange d'émotions
    print("\n4️⃣ Test mélange d'émotions")
    emotions.blend_emotions("happy", "excited", 0.3)

    # Test statistiques
    print("\n5️⃣ Test statistiques")
    stats = emotions.get_emotion_stats()
    print(f"Statistiques : {stats}")

    # Test historique
    print("\n6️⃣ Test historique")
    history = emotions.get_emotion_history(5)
    print(f"Dernières émotions : {[h['emotion'] for h in history]}")

    # Remise à zéro
    print("\n7️⃣ Remise à zéro")
    emotions.reset_emotions()

    print("\n✅ Test BBIA Emotions terminé")


if __name__ == "__main__":
    main()
