#!/usr/bin/env python3
"""Démo PhotoBoothBehavior - Mode photo avec poses expressives.

Démonstration du comportement photo booth avec détection visage.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.photo_booth import PhotoBoothBehavior
from bbia_sim.bbia_vision import BBIAVision


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo PhotoBoothBehavior")
    parser.add_argument(
        "--pose",
        default="happy",
        choices=["happy", "cool", "surprised", "proud"],
        help="Pose à utiliser",
    )
    parser.add_argument("--num-photos", type=int, default=1, help="Nombre de photos")
    parser.add_argument(
        "--no-countdown", action="store_true", help="Désactiver compte à rebours"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("📸 Démo PhotoBoothBehavior - Mode photo avec poses expressives")
    print(f"   • Pose : {args.pose}")
    print(f"   • Nombre de photos : {args.num_photos}")
    print(f"   • Compte à rebours : {not args.no_countdown}")
    print(f"   • Backend : {args.backend}")

    # Créer backend
    if args.backend == "mujoco":
        backend = MuJoCoBackend()
    else:
        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend()

    try:
        backend.connect()
        print("✅ Backend connecté")

        # Créer vision
        vision = BBIAVision(robot_api=backend)
        print("✅ BBIAVision créé")

        # Créer comportement
        photo_booth = PhotoBoothBehavior(vision=vision, robot_api=backend)
        print("✅ PhotoBoothBehavior créé")

        # Exécuter photo booth
        context = {
            "pose": args.pose,
            "num_photos": args.num_photos,
            "countdown": not args.no_countdown,
            "auto_capture": True,
        }
        print(f"\n🚀 Démarrage photo booth avec pose '{args.pose}'...")
        success = photo_booth.execute(context)

        if success:
            print("✅ Photo booth terminé avec succès")
            return 0
        print("❌ Erreur durant le photo booth")
        return 1

    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback

        traceback.print_exc()
        return 1
    finally:
        backend.disconnect()


if __name__ == "__main__":
    exit(main())
