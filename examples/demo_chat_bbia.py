#!/usr/bin/env python3
"""
Démonstration du chat intelligent BBIA
Exemple d'utilisation de la nouvelle fonctionnalité chat avec BBIA
"""

import sys
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

try:
    from bbia_sim.bbia_huggingface import BBIAHuggingFace

    def main():
        """Démonstration complète du chat BBIA."""
        print("💬 Démonstration Chat Intelligent BBIA")
        print("=" * 60)

        # Initialiser BBIA
        bbia = BBIAHuggingFace()
        print(f"\n🤖 BBIA initialisé avec personnalité: {bbia.bbia_personality}")

        # Conversation exemple
        messages = [
            "Bonjour BBIA",
            "Comment allez-vous ?",
            "Je suis très content de te rencontrer",
            "C'est un super projet !",
            "Qu'est-ce que tu fais ?",
            "Je dois partir maintenant, au revoir",
        ]

        print("\n📝 Conversation:")
        print("-" * 60)

        for msg in messages:
            response = bbia.chat(msg)
            print(f"\nVous  : {msg}")
            print(f"BBIA  : {response}")

        # Statistiques
        print("\n" + "=" * 60)
        print("📊 Statistiques de la conversation:")
        print(f"  - Messages échangés: {len(bbia.conversation_history)}")
        print(f"  - Personnalité: {bbia.bbia_personality}")

        # Tester différentes personnalités
        print("\n" + "=" * 60)
        print("🎭 Test des Personnalités BBIA:")
        print("-" * 60)

        personalities = {
            "friendly_robot": "🤖 Robot Amical",
            "curious": "🤔 Curieux",
            "enthusiastic": "🎉 Enthousiaste",
            "calm": "😌 Calme",
        }

        for personality, label in personalities.items():
            bbia.bbia_personality = personality
            response = bbia.chat("Comment te portes-tu ?")
            print(f"\n{label} ({personality}):")
            print(f"  BBIA: {response}")

        # Réinitialiser et démontrer le contexte
        print("\n" + "=" * 60)
        print("🔄 Démontration du contexte conversationnel:")
        print("-" * 60)

        bbia.conversation_history = []  # Reset
        print("\nVous  : Salut !")
        print(f"BBIA  : {bbia.chat('Salut !')}")

        print("\nVous  : Comment ça va ?")
        print(f"BBIA  : {bbia.chat('Comment ça va ?')}")

        print("\nVous  : Je suis triste")
        print(f"BBIA  : {bbia.chat('Je suis triste')}")

        print("\n" + "=" * 60)
        print("✅ Démonstration terminée !")
        print("=" * 60)

    if __name__ == "__main__":
        main()

except ImportError as e:
    print(f"❌ Erreur: {e}")
    print("\n📦 Pour installer les dépendances Hugging Face:")
    print("   pip install transformers torch")
    print("\n💡 Note: Cette démo nécessite Hugging Face transformers")
    sys.exit(1)
except Exception as e:
    print(f"❌ Erreur inattendue: {e}")
    import traceback

    traceback.print_exc()
    sys.exit(1)
