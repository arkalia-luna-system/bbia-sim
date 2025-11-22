#!/usr/bin/env python3
"""Démo StorytellingBehavior - Raconter histoires avec mouvements expressifs.

Démonstration du comportement storytelling avec histoires pré-enregistrées.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.storytelling import StorytellingBehavior


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo StorytellingBehavior")
    parser.add_argument(
        "--story",
        default="petit_chaperon_rouge",
        choices=["petit_chaperon_rouge", "trois_petits_cochons", "blanche_neige"],
        help="Histoire à raconter",
    )
    parser.add_argument(
        "--interactive", action="store_true", help="Mode interactif avec questions"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("📚 Démo StorytellingBehavior - Raconter histoires")
    print(f"   • Histoire : {args.story}")
    print(f"   • Mode interactif : {args.interactive}")
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
        storytelling = StorytellingBehavior(robot_api=backend)
        print("✅ StorytellingBehavior créé")

        # Exécuter storytelling
        context = {
            "story": args.story,
            "interactive": args.interactive,
        }
        print(f"\n🚀 Démarrage narration de '{args.story}'...")
        success = storytelling.execute(context)

        if success:
            print("✅ Narration terminée avec succès")
            return 0
        print("❌ Erreur durant la narration")
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
