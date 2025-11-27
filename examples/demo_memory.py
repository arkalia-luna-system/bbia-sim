#!/usr/bin/env python3
"""Démo BBIAMemory - Mémoire persistante BBIA.

Démonstration du module mémoire pour sauvegarder et charger
l'historique conversation, les préférences et les apprentissages.

Ce script démontre :
- Sauvegarde de conversation, préférences et apprentissages
- Chargement de données mémoire
- Scénario complet de démonstration

Exemples d'utilisation :
    # Scénario complet de démonstration
    python examples/demo_memory.py --action demo

    # Sauvegarder des données exemple
    python examples/demo_memory.py --action save

    # Charger des données sauvegardées
    python examples/demo_memory.py --action load

    # Utiliser un répertoire personnalisé
    python examples/demo_memory.py --action demo --memory-dir /tmp/bbia_memory
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_memory import BBIAMemory


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo BBIAMemory")
    parser.add_argument(
        "--action",
        choices=["save", "load", "demo"],
        default="demo",
        help="Action à effectuer",
    )
    parser.add_argument(
        "--memory-dir",
        default="bbia_memory",
        help="Répertoire pour stocker les fichiers mémoire",
    )

    args = parser.parse_args()

    print("💾 Démo BBIAMemory - Mémoire persistante BBIA")
    print(f"   • Action : {args.action}")
    print(f"   • Répertoire mémoire : {args.memory_dir}")

    # Créer instance mémoire
    memory = BBIAMemory(memory_dir=args.memory_dir)

    try:
        if args.action == "save":
            # Sauvegarder exemple
            print("\n💾 Sauvegarde exemple...")

            # Conversation exemple
            conversation = [
                {
                    "user": "Bonjour",
                    "assistant": "Bonjour ! Comment allez-vous ?",
                    "timestamp": "2025-11-22T10:00:00",
                },
                {
                    "user": "Je vais bien",
                    "assistant": "C'est super !",
                    "timestamp": "2025-11-22T10:01:00",
                },
            ]
            memory.save_conversation(conversation)
            print("   ✅ Conversation sauvegardée")

            # Préférences exemple
            memory.remember_preference("favorite_color", "blue")
            memory.remember_preference("preferred_language", "fr")
            memory.remember_preference("personality", "friendly")
            print("   ✅ Préférences sauvegardées")

            # Apprentissages exemple
            memory.remember_learning("user_likes_greetings", "True")
            memory.remember_learning("user_prefers_short_responses", "False")
            print("   ✅ Apprentissages sauvegardés")

        elif args.action == "load":
            # Charger depuis fichiers
            print("\n📂 Chargement depuis fichiers...")

            conversation = memory.load_conversation()
            print(f"   • Conversation : {len(conversation)} messages")
            for msg in conversation[:3]:  # Afficher 3 premiers
                print(
                    f"     - {msg.get('user', 'N/A')} → {msg.get('assistant', 'N/A')}"
                )

            preferences = memory.load_preferences()
            print(f"   • Préférences : {len(preferences)} entrées")
            for key, value in list(preferences.items())[:3]:
                print(f"     - {key} : {value}")

            learnings = memory.load_learnings()
            print(f"   • Apprentissages : {len(learnings)} entrées")
            for key, value in list(learnings.items())[:3]:
                print(f"     - {key} : {value}")

        elif args.action == "demo":
            # Démo complète
            print("\n🎬 Démo complète mémoire...")

            # 1. Sauvegarder
            print("\n1️⃣ Sauvegarde...")
            conversation = [
                {
                    "user": "Salut BBIA",
                    "assistant": "Salut ! Ravi de te voir !",
                    "timestamp": "2025-11-22T10:00:00",
                },
            ]
            memory.save_conversation(conversation)
            print("   ✅ Conversation sauvegardée")

            memory.remember_preference("favorite_emotion", "happy")
            memory.remember_preference("preferred_voice", "friendly")
            print("   ✅ Préférences sauvegardées")

            # 2. Charger
            print("\n2️⃣ Chargement...")
            loaded_conv = memory.load_conversation()
            print(f"   • Messages chargés : {len(loaded_conv)}")
            loaded_prefs = memory.load_preferences()
            print(f"   • Préférences chargées : {len(loaded_prefs)}")
            loaded_learnings = memory.load_learnings()
            print(f"   • Apprentissages chargés : {len(loaded_learnings)}")

            # 3. Statistiques
            print("\n3️⃣ Statistiques...")
            print(f"   • Taille conversation : {len(loaded_conv)} messages")
            print(f"   • Nombre préférences : {len(loaded_prefs)}")
            print(f"   • Nombre apprentissages : {len(loaded_learnings)}")

            # 4. Test récupération
            print("\n4️⃣ Test récupération...")
            emotion = memory.get_preference("favorite_emotion", "non défini")
            print(f"   • Émotion préférée : {emotion}")
            learning = memory.get_learning("user_likes_greetings")
            if learning:
                print(f"   • Apprentissage trouvé : {learning}")

        print("\n✅ Démo terminée avec succès")
        return 0

    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
