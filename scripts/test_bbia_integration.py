#!/usr/bin/env python3
"""Test d'intégration BBIA ↔ Robot Reachy Mini
Démonstration complète de l'intégration entre les modules BBIA et le simulateur.
"""

import asyncio
import sys
from pathlib import Path

# Ajouter le chemin du module parent
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.bbia_sim.bbia_integration import create_bbia_integration


async def test_emotion_mapping():
    """Test du mapping émotions → articulations."""
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

        # Appliquer l'émotion avec différentes intensités
        for intensity in [0.3, 0.6, 0.9]:
            success = await integration.apply_emotion_to_robot(emotion, intensity)

            if success:
                pass
            else:
                pass

            # Pause pour observer le mouvement
            await asyncio.sleep(1.0)

    # Retour à l'état neutre
    await integration.apply_emotion_to_robot("neutral", 0.5)


async def test_vision_reactions():
    """Test des réactions visuelles."""
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

    success = await integration.react_to_vision_detection(face_detection)

    if success:
        pass
    else:
        pass

    await asyncio.sleep(2.0)

    # Simulation de détection d'objet
    object_detection = {
        "objects": [
            {"name": "bottle", "position": (0.1, 0.0), "confidence": 0.85},
            {"name": "cup", "position": (-0.3, 0.1), "confidence": 0.7},
        ]
    }

    success = await integration.react_to_vision_detection(object_detection)

    if success:
        pass
    else:
        pass

    await asyncio.sleep(2.0)


async def test_voice_synchronization():
    """Test de la synchronisation voix ↔ mouvements."""
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

        success = await integration.sync_voice_with_movements(text, emotion)

        if success:
            pass
        else:
            pass

        await asyncio.sleep(1.0)


async def test_behavior_sequences():
    """Test des séquences de comportement."""
    # Créer l'intégration BBIA
    integration = await create_bbia_integration(
        "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    )

    # Test des comportements disponibles
    behaviors_to_test = ["greeting", "hide", "antenna_animation"]

    for behavior in behaviors_to_test:

        success = await integration.execute_behavior_sequence(behavior)

        if success:
            pass
        else:
            pass

        await asyncio.sleep(2.0)


async def test_integration_status():
    """Test du statut de l'intégration."""
    # Créer l'intégration BBIA
    integration = await create_bbia_integration(
        "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    )

    # Récupérer le statut
    integration.get_integration_status()


async def main():
    """Fonction principale de test d'intégration."""
    try:
        # Tests séquentiels
        await test_emotion_mapping()
        await test_vision_reactions()
        await test_voice_synchronization()
        await test_behavior_sequences()
        await test_integration_status()

    except Exception:
        return 1

    return 0


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
