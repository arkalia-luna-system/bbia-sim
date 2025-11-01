#!/usr/bin/env python3
"""
Demo animations idle - BBIA-SIM
Démontre les animations d'inactivité (respiration, poses de passage).

Usage:
    python examples/demo_idle_animations.py
"""

import sys
import time
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_idle_animations import BBIIdleAnimationManager
from bbia_sim.robot_factory import RobotFactory


def main() -> None:
    """Demo animations idle."""
    print("🤖 Demo Animations Idle - BBIA-SIM")
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

        # Initialiser gestionnaire animations idle
        print("\n🎬 Initialisation animations idle...")
        idle_manager = BBIIdleAnimationManager(robot_api=robot_api)

        # Démarrer animations
        print("\n▶️ Démarrage animations idle...")
        success = idle_manager.start()

        if success:
            print("✅ Animations idle démarrées:")
            print("  - Respiration automatique")
            print("  - Poses de passage (toutes les 15s)")
            print("  - Tremblement vocal (activé)")

            # Laisser tourner pendant 30 secondes
            print("\n⏳ Animations actives pendant 30 secondes...")
            print("   (Observez la respiration et les poses de passage)")

            for i in range(6):
                time.sleep(5)
                print(f"  ✓ {5 * (i + 1)}s écoulées...")

            # Tester tremblement vocal
            print("\n🎤 Test tremblement vocal...")
            for level in [0.4, 0.6, 0.8, 0.5, 0.3]:
                idle_manager.update_vocal_tremor(level)
                print(f"  Niveau audio: {level:.1f} → tremblement appliqué")
                time.sleep(1)

            # Arrêter animations
            print("\n⏹️ Arrêt animations idle...")
            idle_manager.stop()
            print("✅ Animations arrêtées")

        else:
            print("⚠️ Impossible de démarrer animations idle")

        print("\n✅ Demo terminée")

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

