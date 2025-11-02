#!/usr/bin/env python3
"""
Démonstration SIMPLE du chat BBIA (sans Hugging Face)
Version qui fonctionne avec la structure de base
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class MockHuggingFace:
    """Mock pour démontrer le chat sans HF installé."""

    def __init__(self):
        self.bbia_personality = "friendly_robot"
        self.conversation_history = []
        self.context = {}

    def chat(self, user_message: str) -> str:
        """Chat simple sans HF."""
        message_lower = user_message.lower()

        # Salutations
        if any(word in message_lower for word in ["bonjour", "salut", "hello"]):
            response = (
                "🤖 Bonjour ! Comment allez-vous ? Je suis BBIA, votre robot compagnon."
            )
        # Au revoir
        elif any(word in message_lower for word in ["au revoir", "bye", "à bientôt"]):
            response = "🤖 Au revoir ! Ce fut un plaisir. À bientôt !"
        # Positif
        elif any(
            word in message_lower for word in ["content", "heureux", "cool", "super"]
        ):
            response = "🤖 C'est super ! Je suis content pour vous. Continuez !"
        # Question
        elif message_lower.count("?") > 0:
            response = "🤖 C'est une excellente question ! Je réfléchis..."
        # Par défaut
        else:
            response = f"🤖 Je comprends: {user_message}. C'est intéressant, dites-moi en plus !"

        # Sauvegarder
        from datetime import datetime

        self.conversation_history.append(
            {
                "user": user_message,
                "bbia": response,
                "sentiment": {"sentiment": "NEUTRAL", "score": 0.5},
                "timestamp": datetime.now().isoformat(),
            }
        )

        return response


def main():
    """Démonstration complète."""
    print("💬 Démonstration Chat BBIA (Version Simple)")
    print("=" * 60)

    # Initialiser mock
    bbia = MockHuggingFace()
    print(f"\n🤖 BBIA initialisé avec personnalité: {bbia.bbia_personality}")

    # Conversation
    messages = [
        "Bonjour BBIA",
        "Comment allez-vous ?",
        "Je suis très content de te rencontrer",
        "C'est un super projet !",
        "Qu'est-ce que tu fais ?",
        "Je dois partir, au revoir",
    ]

    print("\n📝 Conversation:")
    print("-" * 60)

    for msg in messages:
        response = bbia.chat(msg)
        print(f"\nVous  : {msg}")
        print(f"BBIA  : {response}")

    # Statistiques
    print("\n" + "=" * 60)
    print("📊 Statistiques:")
    print(f"  - Messages: {len(bbia.conversation_history)}")
    print(f"  - Personnalité: {bbia.bbia_personality}")

    print("\n✅ Démo terminée !")


if __name__ == "__main__":
    main()
