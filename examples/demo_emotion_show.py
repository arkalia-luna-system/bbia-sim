#!/usr/bin/env python3
"""Démo EmotionShowBehavior - Démonstration des 12 émotions BBIA.

Parcourt toutes les émotions avec transitions fluides et explications vocales.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.emotion_show import EmotionShowBehavior


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo EmotionShowBehavior")
    parser.add_argument(
        "--emotions",
        nargs="+",
        help="Liste d'émotions à démontrer (par défaut toutes)",
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("🎭 Démo EmotionShowBehavior - Démonstration des émotions BBIA")
    if args.emotions:
        print(f"   • Émotions : {', '.join(args.emotions)}")
    else:
        print("   • Émotions : Toutes (12 émotions)")
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
        emotion_show = EmotionShowBehavior(robot_api=backend)
        print("✅ EmotionShowBehavior créé")

        # Exécuter démonstration
        context = {}
        if args.emotions:
            context["emotions_list"] = args.emotions

        print("\n🚀 Démarrage démonstration des émotions...")
        success = emotion_show.execute(context)

        if success:
            print("✅ Démonstration terminée avec succès")
            return 0
        print("❌ Erreur durant la démonstration")
        return 1

    except KeyboardInterrupt:
        print("\n⚠️  Interrompu par l'utilisateur")
        emotion_show.cancel()
        return 0
    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback

        traceback.print_exc()
        return 1
    finally:
        backend.disconnect()


if __name__ == "__main__":
    exit(main())
