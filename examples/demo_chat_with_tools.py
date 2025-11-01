#!/usr/bin/env python3
"""
Demo chat avec outils LLM - BBIA-SIM
Démontre l'intégration outils LLM avec BBIAHuggingFace.chat()

Usage:
    python examples/demo_chat_with_tools.py
"""

import sys
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_huggingface import BBIAHuggingFace
from bbia_sim.bbia_tools import BBIATools
from bbia_sim.bbia_vision import BBIAVision
from bbia_sim.robot_factory import RobotFactory


def main() -> None:
    """Demo chat avec outils."""
    print("🤖 Demo Chat avec Outils LLM - BBIA-SIM")
    print("=" * 50)

    # Créer backend robot (simulation)
    print("\n📡 Connexion robot (simulation)...")
    robot_api = RobotFactory.create_backend("mujoco")
    if not robot_api:
        print("❌ Impossible de créer backend")
        return

    try:
        robot_api.connect()
        print("✅ Robot connecté")

        # Initialiser modules BBIA
        print("\n🔧 Initialisation modules BBIA...")
        vision = BBIAVision(robot_api=robot_api)
        tools = BBIATools(robot_api=robot_api, vision=vision)

        # Initialiser BBIAHuggingFace avec outils
        print("\n🤗 Initialisation BBIAHuggingFace avec outils...")
        hf = BBIAHuggingFace(tools=tools)
        print("✅ BBIAHuggingFace initialisé avec outils LLM")

        # Exemples de conversation avec détection d'outils
        print("\n💬 Exemples de conversation avec outils:")
        print("-" * 50)

        test_messages = [
            "tourne la tête à gauche",
            "capture une image et analyse l'environnement",
            "fais danser le robot",
            "sois heureux",
            "bonjour, comment vas-tu ?",
        ]

        for i, message in enumerate(test_messages, 1):
            print(f"\n{i}. Utilisateur: {message}")
            response = hf.chat(message, enable_tools=True)
            print(f"   BBIA: {response}")

        print("\n✅ Demo terminée")
        print("\n💡 Note: Les outils sont détectés automatiquement depuis le message")
        print("   et exécutés avant la génération de réponse LLM (si applicable)")

    except Exception as e:
        print(f"\n❌ Erreur: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if robot_api:
            robot_api.disconnect()
            print("\n🔌 Robot déconnecté")


if __name__ == "__main__":
    main()

