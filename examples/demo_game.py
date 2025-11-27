#!/usr/bin/env python3
"""Démo GameBehavior - Jeux interactifs avec réactions émotionnelles.

Démonstration du comportement game avec différents jeux.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.game import GameBehavior


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo GameBehavior")
    parser.add_argument(
        "--game",
        default="rock_paper_scissors",
        choices=["rock_paper_scissors", "guess_number", "memory"],
        help="Jeu à jouer",
    )
    parser.add_argument("--rounds", type=int, default=3, help="Nombre de rounds")
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("🎮 Démo GameBehavior - Jeux interactifs")
    print(f"   • Jeu : {args.game}")
    print(f"   • Rounds : {args.rounds}")
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
        game = GameBehavior(robot_api=backend)
        print("✅ GameBehavior créé")

        # Exécuter game
        context = {
            "game": args.game,
            "rounds": args.rounds,
        }
        print(f"\n🚀 Démarrage jeu '{args.game}' ({args.rounds} rounds)...")
        success = game.execute(context)

        if success:
            print("✅ Jeu terminé avec succès")
            return 0
        print("❌ Erreur durant le jeu")
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
