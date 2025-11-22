#!/usr/bin/env python3
"""Démo BBIAIntegration - Intégration complète BBIA ↔ Robot.

Démonstration du module d'intégration complet connectant tous les modules BBIA.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.bbia_integration import BBIAIntegration
from bbia_sim.daemon.simulation_service import SimulationService


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo BBIAIntegration")
    parser.add_argument(
        "--action",
        choices=["emotion", "vision", "voice", "behavior"],
        default="emotion",
        help="Action à effectuer",
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("🔗 Démo BBIAIntegration - Intégration complète BBIA")
    print(f"   • Action : {args.action}")
    print(f"   • Backend : {args.backend}")

    try:
        # Créer backend
        if args.backend == "mujoco":
            backend = MuJoCoBackend()
        else:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            backend = ReachyMiniBackend()

        backend.connect()
        print("✅ Backend connecté")

        # Créer service simulation
        simulation_service = SimulationService()
        simulation_service.robot_api = backend
        print("✅ SimulationService créé")

        # Créer intégration
        integration = BBIAIntegration(simulation_service=simulation_service)
        print("✅ BBIAIntegration créé")

        # Démarrer intégration
        import asyncio

        async def run_integration():
            await integration.start_integration()

            # Exécuter action
            if args.action == "emotion":
                print("\n🎭 Test émotion → mouvement...")
                await integration.apply_emotion_to_robot("happy", 0.8)
                print("   Émotion 'happy' appliquée")

            elif args.action == "vision":
                print("\n👁️  Test vision → tracking...")
                # Simuler détection objet
                detection_data = {"objects": [{"name": "person", "bbox": {}}]}
                await integration.react_to_vision_detection(detection_data)
                print("   Réaction vision déclenchée")

            elif args.action == "voice":
                print("\n🎤 Test voix → action...")
                await integration.sync_voice_with_movements("Bonjour", "happy")
                print("   Réaction voix déclenchée")

            elif args.action == "behavior":
                print("\n🎭 Test comportement...")
                await integration.execute_behavior_sequence("wake_up")
                print("   Comportement 'wake_up' exécuté")

            await integration.stop_integration()

        asyncio.run(run_integration())

        print("\n✅ Démo terminée avec succès")
        return 0

    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback

        traceback.print_exc()
        return 1
    finally:
        backend.disconnect()


if __name__ == "__main__":
    sys.exit(main())
