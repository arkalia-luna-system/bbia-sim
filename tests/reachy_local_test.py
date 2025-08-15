#!/usr/bin/env python3
"""
Test du simulateur local Reachy pour BBIA
Utilise le simulateur local au lieu du simulateur web
"""

import sys
import os
import time

# Ajouter le répertoire src au path pour importer nos modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim.reachy_local_sim import get_simulator


def test_local_simulation():
    """Test de la simulation locale Reachy"""
    print("🤖 Test de la simulation locale Reachy")
    print("=" * 50)

    # Récupérer l'instance du simulateur
    sim = get_simulator()

    print("✨ Connectée au simulateur local !")
    time.sleep(1)

    # Test du bras droit
    print("\n🦾 Test du bras droit...")
    sim.right_arm.elbow_pitch.goal_position = -80
    time.sleep(3)

    # Test de la tête
    print("\n👁️ Test de la tête...")
    sim.head.yaw.goal_position = 30
    time.sleep(3)

    # Test du bras gauche
    print("\n🦾 Test du bras gauche...")
    sim.left_arm.shoulder_pitch.goal_position = 45
    time.sleep(3)

    # Test d'un mouvement complexe
    print("\n🎭 Test d'un mouvement complexe...")
    sim.right_arm.shoulder_pitch.goal_position = 60
    sim.left_arm.elbow_pitch.goal_position = -45
    sim.head.neck_pitch.goal_position = 15
    time.sleep(5)

    # Afficher le statut final
    print("\n📊 Statut final du robot:")
    status = sim.get_status()
    for part, joints in status.items():
        print(f"  {part}:")
        for joint_name, position in joints.items():
            print(f"    {joint_name}: {position:.1f}°")

    print("\n🎉 Test de simulation locale terminé avec succès !")

    # Ne pas arrêter le simulateur pour permettre d'autres tests
    # sim.stop()


if __name__ == "__main__":
    try:
        test_local_simulation()
    except KeyboardInterrupt:
        print("\n⚠️ Test interrompu par l'utilisateur")
    except Exception as e:
        print(f"\n❌ Erreur lors du test: {e}")
        import traceback

        traceback.print_exc()
