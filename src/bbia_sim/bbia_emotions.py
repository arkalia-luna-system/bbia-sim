#!/usr/bin/env python3

"""BBIA Emotions - Module d'émotions avancé pour Reachy Mini Wireless
Émotions complexes, expressions faciales, transitions fluides.
"""

import secrets
from datetime import datetime
from typing import Any


class BBIAEmotions:
    """Module d'émotions avancé pour BBIA."""

    def __init__(self) -> None:
        self.current_emotion = "neutral"
        self.emotion_intensity = 0.5  # 0.0 à 1.0
        self.transition_duration = 1.0  # secondes
        self.emotion_history: list[dict[str, Any]] = []

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
            "confused": {
                "yeux": "Cercles désynchronisés",
                "antennes": "Mouvements erratiques",
                "tete": "Mouvements de confusion",
                "description": "Confusion, perplexité, incertitude",
                "color": "😕",
            },
            "determined": {
                "yeux": "Cercles fixes et intenses",
                "antennes": "Droites et fermes",
                "tete": "Position ferme et décidée",
                "description": "Détermination, résolution, volonté",
                "color": "😤",
            },
            "nostalgic": {
                "yeux": "Cercles doux et rêveurs",
                "antennes": "Légèrement tombantes",
                "tete": "Inclinée, regard mélancolique",
                "description": "Nostalgie, mélancolie, souvenirs",
                "color": "😌",
            },
            "proud": {
                "yeux": "Cercles brillants et fiers",
                "antennes": "Hautes et droites",
                "tete": "Relevée, regard satisfait",
                "description": "Fierté, satisfaction, accomplissement",
                "color": "😎",
            },
        }

    def set_emotion(self, emotion: str, intensity: float = 0.5) -> bool:
        """Change l'émotion de BBIA."""
        if emotion not in self.emotions:
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
            },
        )

        # Afficher la transition
        self._display_emotion_transition(old_emotion, emotion)

        return True

    def _display_emotion_transition(self, old_emotion: str, new_emotion: str) -> None:
        """Affiche la transition d'émotion."""
        old_data = self.emotions[old_emotion]
        new_data = self.emotions[new_emotion]

        print(
            f"🔄 Transition d'émotion : {old_data['color']} {old_emotion} → {new_data['color']} {new_emotion}",
        )
        print(f"📝 {old_data['description']} → {new_data['description']}")
        print(f"🎭 Intensité : {self.emotion_intensity:.1f}")
        print(f"⏰ {datetime.now().strftime('%H:%M:%S')}")

    def get_current_emotion(self) -> dict[str, Any]:
        """Retourne l'émotion actuelle avec ses détails."""
        emotion_data = dict(self.emotions[self.current_emotion])
        emotion_data.update(
            {
                "name": self.current_emotion,
                "intensity": str(self.emotion_intensity),
                "timestamp": datetime.now().isoformat(),
            },
        )
        return emotion_data

    def get_emotion_history(self, limit: int = 10) -> list[dict[str, Any]]:
        """Retourne l'historique des émotions."""
        return self.emotion_history[-limit:] if limit > 0 else self.emotion_history

    def random_emotion(self) -> str:
        """Change vers une émotion aléatoire."""
        available_emotions = list(self.emotions.keys())
        available_emotions.remove(self.current_emotion)  # Éviter la même émotion

        if available_emotions:
            new_emotion = secrets.choice(available_emotions)
            intensity = secrets.randbelow(70) / 100.0 + 0.3  # 0.3 à 1.0
            self.set_emotion(new_emotion, intensity)
            return str(new_emotion)
        return str(self.current_emotion)

    def emotional_response(self, stimulus: str) -> str:
        """Réponse émotionnelle à un stimulus."""
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
                emotion = secrets.choice(emotions)
                intensity = secrets.randbelow(50) / 100.0 + 0.4  # 0.4 à 0.9
                self.set_emotion(emotion, intensity)
                return emotion

        # Réponse par défaut
        self.set_emotion("curious", 0.5)
        return "curious"

    def blend_emotions(self, emotion1: str, emotion2: str, ratio: float = 0.5) -> str:
        """Mélange deux émotions."""
        if emotion1 not in self.emotions or emotion2 not in self.emotions:
            return str(self.current_emotion)

        # Logique simple de mélange
        result_emotion = emotion1 if ratio < 0.5 else emotion2

        intensity = 0.5 + (abs(ratio - 0.5) * 0.5)  # Intensité basée sur la différence
        self.set_emotion(result_emotion, intensity)

        return result_emotion

    def get_emotion_stats(self) -> dict[str, Any]:
        """Retourne les statistiques des émotions."""
        emotion_counts: dict[str, int] = {}
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

    def reset_emotions(self) -> None:
        """Remet BBIA en état neutre."""
        self.set_emotion("neutral", 0.5)
        self.emotion_history.clear()


def main() -> None:
    """Test du module BBIA Emotions."""
    # Créer l'instance
    emotions = BBIAEmotions()

    # Test émotions de base
    emotions.set_emotion("happy", 0.8)
    emotions.set_emotion("curious", 0.6)
    emotions.set_emotion("sad", 0.7)

    # Test émotion aléatoire
    emotions.random_emotion()

    # Test réponses émotionnelles
    emotions.emotional_response("compliment")
    emotions.emotional_response("question")
    emotions.emotional_response("danger")

    # Test mélange d'émotions
    emotions.blend_emotions("happy", "excited", 0.3)

    # Test statistiques
    emotions.get_emotion_stats()

    # Test historique
    emotions.get_emotion_history(5)

    # Remise à zéro
    emotions.reset_emotions()


if __name__ == "__main__":
    main()
