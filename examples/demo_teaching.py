#!/usr/bin/env python3
"""Démo TeachingBehavior - Mode éducatif interactif.

Démonstration du comportement teaching avec leçons et questions.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.teaching import TeachingBehavior


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo TeachingBehavior")
    parser.add_argument(
        "--subject",
        default="maths",
        choices=["maths", "sciences", "geographie"],
        help="Matière à enseigner",
    )
    parser.add_argument(
        "--level",
        default="beginner",
        choices=["beginner", "intermediate", "advanced"],
        help="Niveau",
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("📖 Démo TeachingBehavior - Mode éducatif interactif")
    print(f"   • Matière : {args.subject}")
    print(f"   • Niveau : {args.level}")
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
        teaching = TeachingBehavior(robot_api=backend)
        print("✅ TeachingBehavior créé")

        # Exécuter teaching
        context = {
            "subject": args.subject,
            "level": args.level,
        }
        print(f"\n🚀 Démarrage leçon '{args.subject}' niveau '{args.level}'...")
        success = teaching.execute(context)

        if success:
            print("✅ Leçon terminée avec succès")
            return 0
        print("❌ Erreur durant la leçon")
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
