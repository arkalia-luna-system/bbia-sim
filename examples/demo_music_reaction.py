#!/usr/bin/env python3
"""Démo MusicReactionBehavior - Réagir à la musique avec mouvements.

Démonstration du comportement music_reaction avec synchronisation rythme.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.music_reaction import MusicReactionBehavior


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo MusicReactionBehavior")
    parser.add_argument(
        "--genre",
        default="pop",
        choices=["pop", "classical", "rock", "jazz"],
        help="Genre musical",
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

    print("🎵 Démo MusicReactionBehavior - Réaction à la musique")
    print(f"   • Genre : {args.genre}")
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
        music_reaction = MusicReactionBehavior(robot_api=backend)
        print("✅ MusicReactionBehavior créé")

        # Exécuter music_reaction
        context = {
            "genre": args.genre,
            "duration": args.duration,
        }
        print(f"\n🚀 Démarrage réaction musique genre '{args.genre}'...")
        success = music_reaction.execute(context)

        if success:
            print("✅ Réaction musique terminée avec succès")
            return 0
        print("❌ Erreur durant la réaction")
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
