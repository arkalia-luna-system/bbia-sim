#!/usr/bin/env python3
"""Démo ExerciseBehavior - Guide exercices physiques.

Démonstration du comportement exercise avec mouvements démonstratifs.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.exercise import ExerciseBehavior


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo ExerciseBehavior")
    parser.add_argument(
        "--exercise",
        default="head_rotation",
        choices=["head_rotation", "neck_stretch", "shoulder_roll"],
        help="Exercice à effectuer",
    )
    parser.add_argument(
        "--repetitions", type=int, default=5, help="Nombre de répétitions"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("💪 Démo ExerciseBehavior - Guide exercices physiques")
    print(f"   • Exercice : {args.exercise}")
    print(f"   • Répétitions : {args.repetitions}")
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
        exercise = ExerciseBehavior(robot_api=backend)
        print("✅ ExerciseBehavior créé")

        # Exécuter exercise
        context = {
            "exercise": args.exercise,
            "repetitions": args.repetitions,
        }
        print(f"\n🚀 Démarrage exercice '{args.exercise}' ({args.repetitions} rép.)...")
        success = exercise.execute(context)

        if success:
            print("✅ Exercice terminé avec succès")
            return 0
        print("❌ Erreur durant l'exercice")
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
