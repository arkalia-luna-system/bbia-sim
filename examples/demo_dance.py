#!/usr/bin/env python3
"""Démo DanceBehavior - Danse synchronisée avec musique.

Démonstration du comportement de danse avec différents types de musique.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.dance import DanceBehavior


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo DanceBehavior")
    parser.add_argument(
        "--music-type",
        default="happy",
        choices=["happy", "calm", "energetic"],
        help="Type de musique",
    )
    parser.add_argument(
        "--duration", type=float, default=30.0, help="Durée en secondes"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("🎵 Démo DanceBehavior - Danse synchronisée avec musique")
    print(f"   • Type musique : {args.music_type}")
    print(f"   • Durée : {args.duration}s")
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

        # Créer comportement
        dance = DanceBehavior(robot_api=backend)
        print("✅ DanceBehavior créé")

        # Exécuter danse
        context = {
            "music_type": args.music_type,
            "duration": args.duration,
        }
        print(f"\n🚀 Démarrage danse type '{args.music_type}'...")
        success = dance.execute(context)

        if success:
            print("✅ Danse terminée avec succès")
            return 0
        print("❌ Erreur durant la danse")
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
