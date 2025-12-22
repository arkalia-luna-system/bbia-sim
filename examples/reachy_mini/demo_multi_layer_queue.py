#!/usr/bin/env python3
"""Démonstration de la file d'attente multicouche.

Exemple d'utilisation pour danses, émotions, poses simultanées.
"""

import asyncio
import sys
from pathlib import Path

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

from bbia_sim.multi_layer_queue import (
    MovementPriority,
    MovementType,
    get_multi_layer_queue,
)


async def demo_dance():
    """Simule une danse."""
    print("🕺 Début danse...")
    await asyncio.sleep(2.0)
    print("✅ Danse terminée")


async def demo_emotion(emotion: str, intensity: float):
    """Simule une émotion."""
    print(f"😊 Application émotion: {emotion} (intensité: {intensity})")
    await asyncio.sleep(1.0)
    print(f"✅ Émotion {emotion} appliquée")


async def demo_pose(pose_name: str):
    """Simule une pose."""
    print(f"🎭 Application pose: {pose_name}")
    await asyncio.sleep(0.5)
    print(f"✅ Pose {pose_name} appliquée")


async def main():
    """Démonstration principale."""
    print("=" * 70)
    print("🎯 DÉMONSTRATION FILE D'ATTENTE MULTICOUCHE")
    print("=" * 70)
    print()

    queue = get_multi_layer_queue()

    # Ajouter plusieurs mouvements avec priorités différentes
    print("📝 Ajout de mouvements à la queue...")
    print()

    # 1. Danse (priorité haute)
    result1 = await queue.add_dance(
        lambda: demo_dance(),
        dance_id="dance_1",
        metadata={"name": "Danse joyeuse"},
    )
    print(f"✅ Danse ajoutée: {result1['movement_id']}")

    # 2. Émotion (priorité moyenne)
    result2 = await queue.add_emotion(
        lambda: demo_emotion("happy", 0.8),
        emotion_id="emotion_1",
        metadata={"emotion": "happy", "intensity": 0.8},
    )
    print(f"✅ Émotion ajoutée: {result2['movement_id']}")

    # 3. Pose (priorité basse)
    result3 = await queue.add_pose(
        lambda: demo_pose("pose_neutre"),
        pose_id="pose_1",
        metadata={"name": "Pose neutre"},
    )
    print(f"✅ Pose ajoutée: {result3['movement_id']}")

    # 4. Autre émotion
    result4 = await queue.add_emotion(
        lambda: demo_emotion("excited", 0.9),
        emotion_id="emotion_2",
        metadata={"emotion": "excited", "intensity": 0.9},
    )
    print(f"✅ Émotion ajoutée: {result4['movement_id']}")

    print()
    print("📊 Statistiques de la queue:")
    stats = queue.get_stats()
    print(f"  - Taille queue: {sum(stats['queue_sizes'].values())}")
    print(f"  - Mouvements en cours: {stats['running_count']}")
    print(f"  - Max parallèle: {stats['max_parallel']}")
    print()

    # Attendre que tous les mouvements se terminent
    print("⏳ Attente de l'exécution de tous les mouvements...")
    print()

    # Flush pour forcer l'exécution immédiate
    await queue.flush()

    print()
    print("📊 Statistiques finales:")
    final_stats = queue.get_stats()
    print(f"  - Total exécuté: {final_stats['stats']['total_executed']}")
    print(f"  - Total échoué: {final_stats['stats']['total_failed']}")
    print(f"  - Par priorité: {final_stats['stats']['by_priority']}")

    print()
    print("=" * 70)
    print("✅ DÉMONSTRATION TERMINÉE")
    print("=" * 70)


if __name__ == "__main__":
    asyncio.run(main())

