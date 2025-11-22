#!/usr/bin/env python3
"""Démo FollowObjectBehavior - Suivi d'objet avec priorisation intelligente.

Démonstration du comportement de suivi d'objet avec détection YOLO.

Ce script démontre :
- Suivi automatique d'objets détectés par YOLO
- Priorisation intelligente (personne > objet)
- Réactions quand un objet est perdu
- Utilisation de BBIAVision pour la détection

Exemples d'utilisation :
    # Suivi automatique (priorisation)
    python examples/demo_follow_object.py

    # Suivi d'un objet spécifique
    python examples/demo_follow_object.py --target-object person

    # Mode headless (sans viewer)
    python examples/demo_follow_object.py --headless --duration 5.0
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.bbia_vision import BBIAVision
from bbia_sim.behaviors.follow_object import FollowObjectBehavior
from bbia_sim.robot_factory import RobotFactory


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo FollowObjectBehavior")
    parser.add_argument(
        "--target-object",
        default=None,
        help="Objet spécifique à suivre (ex: 'person', 'phone', 'cup')",
    )
    parser.add_argument(
        "--duration", type=float, default=10.0, help="Durée du suivi en secondes"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("👁️ Démo FollowObjectBehavior - Suivi d'objet avec priorisation")
    print(f"   • Objet cible : {args.target_object or 'auto (priorisation)'}")
    print(f"   • Durée : {args.duration}s")
    print(f"   • Backend : {args.backend}")

    # Créer backend
    if args.backend == "mujoco":
        backend = MuJoCoBackend(headless=args.headless)
    else:
        from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

        backend = ReachyMiniBackend(use_sim=True)

    try:
        # Connexion
        if not backend.connect():
            print("❌ Erreur connexion backend")
            return 1

        # Créer RobotAPI
        robot_api = RobotFactory.create_robot_api(backend=backend)

        # Créer vision
        vision = BBIAVision(robot_api=robot_api)

        # Créer comportement
        behavior = FollowObjectBehavior(vision=vision, robot_api=robot_api)

        # Exécuter
        context = {
            "duration": args.duration,
            "target_object": args.target_object,
        }

        print("\n🎯 Démarrage suivi d'objet...")
        success = behavior.execute(context)

        if success:
            print("✅ Suivi terminé avec succès")
        else:
            print("⚠️ Suivi terminé avec avertissements")

        return 0

    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par l'utilisateur")
        behavior.stop()
        return 0
    except Exception as e:
        print(f"❌ Erreur : {e}")
        return 1
    finally:
        backend.disconnect()


if __name__ == "__main__":
    sys.exit(main())
