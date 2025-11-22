#!/usr/bin/env python3
"""Démo NewsReaderBehavior - Lecture actualités avec réactions.

Démonstration du comportement news_reader avec réactions émotionnelles.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.news_reader import NewsReaderBehavior


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo NewsReaderBehavior")
    parser.add_argument(
        "--max-items", type=int, default=5, help="Nombre max d'actualités"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("📰 Démo NewsReaderBehavior - Lecture actualités")
    print(f"   • Nombre d'actualités : {args.max_items}")
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
        news_reader = NewsReaderBehavior(robot_api=backend)
        print("✅ NewsReaderBehavior créé")

        # Exécuter news_reader
        context = {"max_items": args.max_items}
        print(f"\n🚀 Démarrage lecture actualités ({args.max_items} items)...")
        success = news_reader.execute(context)

        if success:
            print("✅ Lecture actualités terminée avec succès")
            return 0
        print("❌ Erreur durant la lecture")
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
