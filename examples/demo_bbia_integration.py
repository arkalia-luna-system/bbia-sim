#!/usr/bin/env python3
"""
Exemple d'utilisation simple de l'intégration BBIA ↔ Robot
Démonstration rapide des fonctionnalités principales
"""

import asyncio
import sys
from pathlib import Path

# Ajouter le chemin du module parent
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.bbia_sim.bbia_integration import create_bbia_integration


async def demo_simple():
    """Démonstration simple de l'intégration BBIA."""

    print("🎭 DÉMONSTRATION BBIA ↔ ROBOT REACHY MINI")
    print("=" * 50)

    # Créer l'intégration BBIA
    print("🚀 Initialisation de l'intégration BBIA...")
    integration = await create_bbia_integration(
        "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    )

    print("✅ Intégration BBIA prête !")

    # Démonstration des émotions
    print("\n🎭 Démonstration des émotions :")

    emotions_demo = [
        ("neutral", "État de repos"),
        ("happy", "Joie et contentement"),
        ("sad", "Tristesse et mélancolie"),
        ("angry", "Colère et frustration"),
        ("surprised", "Surprise et étonnement"),
        ("curious", "Curiosité et intérêt"),
        ("excited", "Excitation et enthousiasme"),
        ("fearful", "Peur et appréhension"),
    ]

    for emotion, description in emotions_demo:
        print(f"\n🎭 {emotion.upper()} - {description}")
        await integration.apply_emotion_to_robot(emotion, 0.7)
        await asyncio.sleep(2.0)

    # Démonstration des réactions visuelles
    print("\n👁️ Démonstration des réactions visuelles :")

    # Simulation détection de visage
    print("👤 Simulation détection de visage...")
    face_data = {"faces": [{"position": (0.2, 0.1), "confidence": 0.9}]}
    await integration.react_to_vision_detection(face_data)
    await asyncio.sleep(2.0)

    # Simulation détection d'objet
    print("📦 Simulation détection d'objet...")
    object_data = {
        "objects": [{"name": "bottle", "position": (0.1, 0.0), "confidence": 0.85}]
    }
    await integration.react_to_vision_detection(object_data)
    await asyncio.sleep(2.0)

    # Démonstration synchronisation voix
    print("\n🗣️ Démonstration synchronisation voix :")

    test_phrases = [
        ("Bonjour ! Je suis BBIA.", "happy"),
        ("Je suis très surpris !", "surprised"),
        ("Je suis curieux de vous.", "curious"),
    ]

    for text, emotion in test_phrases:
        print(f"🗣️ '{text}' (émotion: {emotion})")
        await integration.sync_voice_with_movements(text, emotion)
        await asyncio.sleep(1.5)

    # Démonstration comportements
    print("\n🎬 Démonstration des comportements :")

    behaviors = ["greeting", "antenna_animation", "hide"]

    for behavior in behaviors:
        print(f"🎬 Comportement : {behavior}")
        await integration.execute_behavior_sequence(behavior)
        await asyncio.sleep(2.0)

    # Retour à l'état neutre
    print("\n🏁 Retour à l'état neutre...")
    await integration.apply_emotion_to_robot("neutral", 0.5)

    # Afficher le statut final
    status = integration.get_integration_status()
    print("\n📊 Statut final :")
    print(f"   • Émotion : {status['current_emotion']}")
    print(f"   • Intensité : {status['emotion_intensity']}")
    print(f"   • Active : {status['is_active']}")

    print("\n🎉 Démonstration terminée avec succès !")

    # Arrêter l'intégration
    await integration.stop_integration()


async def demo_interactive():
    """Démonstration interactive de l'intégration BBIA."""

    print("🎮 DÉMONSTRATION INTERACTIVE BBIA ↔ ROBOT")
    print("=" * 50)

    # Créer l'intégration BBIA
    integration = await create_bbia_integration(
        "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    )

    print("✅ Intégration BBIA prête !")
    print("\nCommandes disponibles :")
    print("  emotion <nom> <intensité> - Appliquer une émotion")
    print("  vision <type> - Simuler une détection visuelle")
    print("  voice <texte> <émotion> - Synchroniser voix + mouvements")
    print("  behavior <nom> - Exécuter un comportement")
    print("  status - Afficher le statut")
    print("  quit - Quitter")

    while True:
        try:
            command = input("\n🎮 Commande : ").strip().lower()

            if command == "quit":
                break

            elif command.startswith("emotion "):
                parts = command.split()
                if len(parts) >= 2:
                    emotion = parts[1]
                    intensity = float(parts[2]) if len(parts) > 2 else 0.5
                    await integration.apply_emotion_to_robot(emotion, intensity)
                    print(f"✅ Émotion '{emotion}' appliquée (intensité: {intensity})")
                else:
                    print("❌ Usage: emotion <nom> [intensité]")

            elif command.startswith("vision "):
                vision_type = command.split()[1] if len(command.split()) > 1 else "face"
                if vision_type == "face":
                    data = {"faces": [{"position": (0.2, 0.1), "confidence": 0.9}]}
                elif vision_type == "object":
                    data = {
                        "objects": [
                            {
                                "name": "bottle",
                                "position": (0.1, 0.0),
                                "confidence": 0.85,
                            }
                        ]
                    }
                else:
                    print("❌ Types disponibles: face, object")
                    continue

                await integration.react_to_vision_detection(data)
                print(f"✅ Réaction à la détection '{vision_type}' appliquée")

            elif command.startswith("voice "):
                parts = command.split(" ", 2)
                if len(parts) >= 3:
                    text = parts[1]
                    emotion = parts[2]
                    await integration.sync_voice_with_movements(text, emotion)
                    print("✅ Synchronisation voix + mouvements appliquée")
                else:
                    print("❌ Usage: voice <texte> <émotion>")

            elif command.startswith("behavior "):
                behavior = (
                    command.split()[1] if len(command.split()) > 1 else "greeting"
                )
                await integration.execute_behavior_sequence(behavior)
                print(f"✅ Comportement '{behavior}' exécuté")

            elif command == "status":
                status = integration.get_integration_status()
                print(
                    f"📊 Statut : {status['current_emotion']} (intensité: {status['emotion_intensity']})"
                )

            else:
                print("❌ Commande inconnue")

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"❌ Erreur : {e}")

    # Arrêter l'intégration
    await integration.stop_integration()
    print("\n👋 Démonstration interactive terminée !")


async def main():
    """Fonction principale."""

    if len(sys.argv) > 1 and sys.argv[1] == "interactive":
        await demo_interactive()
    else:
        await demo_simple()


if __name__ == "__main__":
    asyncio.run(main())
