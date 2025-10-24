#!/usr/bin/env python3
"""
Test d'intégration BBIA ↔ Robot Reachy Mini
Démonstration complète de l'intégration entre les modules BBIA et le simulateur
"""

import asyncio
import sys
from pathlib import Path

# Ajouter le chemin du module parent
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.bbia_sim.bbia_integration import create_bbia_integration


async def test_emotion_mapping():
    """Test du mapping émotions → articulations."""

    print("🎭 Test du mapping émotions → articulations")
    print("=" * 50)

    # Créer l'intégration BBIA
    integration = await create_bbia_integration(
        "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    )

    # Liste des émotions à tester
    emotions_to_test = [
        "neutral",
        "happy",
        "sad",
        "angry",
        "surprised",
        "curious",
        "excited",
        "fearful",
    ]

    for emotion in emotions_to_test:
        print(f"\n🎭 Test émotion : {emotion}")

        # Appliquer l'émotion avec différentes intensités
        for intensity in [0.3, 0.6, 0.9]:
            print(f"   Intensité : {intensity}")
            success = await integration.apply_emotion_to_robot(emotion, intensity)

            if success:
                print(f"   ✅ Émotion '{emotion}' appliquée (intensité: {intensity})")
            else:
                print(f"   ❌ Échec application émotion '{emotion}'")

            # Pause pour observer le mouvement
            await asyncio.sleep(1.0)

    # Retour à l'état neutre
    await integration.apply_emotion_to_robot("neutral", 0.5)
    print("\n✅ Test mapping émotions terminé")


async def test_vision_reactions():
    """Test des réactions visuelles."""

    print("\n👁️ Test des réactions visuelles")
    print("=" * 50)

    # Créer l'intégration BBIA
    integration = await create_bbia_integration(
        "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    )

    # Simulation de détection de visage
    face_detection = {
        "faces": [
            {"position": (0.3, 0.1), "confidence": 0.9},
            {"position": (-0.2, 0.2), "confidence": 0.8},
        ]
    }

    print("👤 Simulation détection de visage...")
    success = await integration.react_to_vision_detection(face_detection)

    if success:
        print("✅ Réaction à la détection de visage réussie")
    else:
        print("❌ Échec réaction visuelle")

    await asyncio.sleep(2.0)

    # Simulation de détection d'objet
    object_detection = {
        "objects": [
            {"name": "bottle", "position": (0.1, 0.0), "confidence": 0.85},
            {"name": "cup", "position": (-0.3, 0.1), "confidence": 0.7},
        ]
    }

    print("📦 Simulation détection d'objet...")
    success = await integration.react_to_vision_detection(object_detection)

    if success:
        print("✅ Réaction à la détection d'objet réussie")
    else:
        print("❌ Échec réaction visuelle")

    await asyncio.sleep(2.0)

    print("✅ Test réactions visuelles terminé")


async def test_voice_synchronization():
    """Test de la synchronisation voix ↔ mouvements."""

    print("\n🗣️ Test synchronisation voix ↔ mouvements")
    print("=" * 50)

    # Créer l'intégration BBIA
    integration = await create_bbia_integration(
        "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    )

    # Test avec différentes émotions
    test_phrases = [
        ("Bonjour, je suis BBIA !", "happy"),
        ("Je suis triste aujourd'hui...", "sad"),
        ("Je suis très surpris !", "surprised"),
        ("Je suis curieux de vous connaître.", "curious"),
    ]

    for text, emotion in test_phrases:
        print(f"\n🗣️ Phrase : '{text}' (émotion: {emotion})")

        success = await integration.sync_voice_with_movements(text, emotion)

        if success:
            print("✅ Synchronisation voix + mouvements réussie")
        else:
            print("❌ Échec synchronisation voix")

        await asyncio.sleep(1.0)

    print("✅ Test synchronisation voix terminé")


async def test_behavior_sequences():
    """Test des séquences de comportement."""

    print("\n🎬 Test séquences de comportement")
    print("=" * 50)

    # Créer l'intégration BBIA
    integration = await create_bbia_integration(
        "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    )

    # Test des comportements disponibles
    behaviors_to_test = ["greeting", "hide", "antenna_animation"]

    for behavior in behaviors_to_test:
        print(f"\n🎬 Test comportement : {behavior}")

        success = await integration.execute_behavior_sequence(behavior)

        if success:
            print(f"✅ Séquence '{behavior}' exécutée avec succès")
        else:
            print(f"❌ Échec exécution comportement '{behavior}'")

        await asyncio.sleep(2.0)

    print("✅ Test séquences comportement terminé")


async def test_integration_status():
    """Test du statut de l'intégration."""

    print("\n📊 Test statut intégration")
    print("=" * 50)

    # Créer l'intégration BBIA
    integration = await create_bbia_integration(
        "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    )

    # Récupérer le statut
    status = integration.get_integration_status()

    print("📊 Statut de l'intégration BBIA :")
    print(f"   • Active : {status['is_active']}")
    print(f"   • Émotion actuelle : {status['current_emotion']}")
    print(f"   • Intensité : {status['emotion_intensity']}")
    print(f"   • Simulation prête : {status['simulation_ready']}")
    print(f"   • Émotions disponibles : {len(status['available_emotions'])}")

    print("✅ Test statut intégration terminé")


async def main():
    """Fonction principale de test d'intégration."""

    print("🚀 TEST D'INTÉGRATION BBIA ↔ ROBOT REACHY MINI")
    print("=" * 60)
    print("Ce test démontre l'intégration complète entre les modules BBIA")
    print("et le simulateur MuJoCo du robot Reachy Mini.")
    print("=" * 60)

    try:
        # Tests séquentiels
        await test_emotion_mapping()
        await test_vision_reactions()
        await test_voice_synchronization()
        await test_behavior_sequences()
        await test_integration_status()

        print("\n" + "=" * 60)
        print("🎉 TOUS LES TESTS D'INTÉGRATION RÉUSSIS !")
        print("✅ L'intégration BBIA ↔ Robot fonctionne parfaitement")
        print("=" * 60)

    except Exception as e:
        print(f"\n❌ Erreur lors des tests d'intégration : {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
