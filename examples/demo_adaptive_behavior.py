#!/usr/bin/env python3
"""Démo BBIAAdaptiveBehavior - Comportements adaptatifs contextuels.

Démonstration du module de comportements adaptatifs qui génère
des comportements dynamiques basés sur le contexte et l'émotion.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.bbia_adaptive_behavior import BBIAAdaptiveBehavior
from bbia_sim.robot_factory import RobotFactory


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo BBIAAdaptiveBehavior")
    parser.add_argument(
        "--context",
        default="greeting",
        choices=[
            "greeting",
            "conversation",
            "attention",
            "rest",
            "playful",
            "serious",
            "sleepy",
        ],
        help="Contexte pour générer le comportement",
    )
    parser.add_argument(
        "--emotion",
        default="happy",
        choices=["happy", "sad", "excited", "curious", "calm", "neutral"],
        help="Émotion à appliquer",
    )
    parser.add_argument(
        "--duration", type=float, default=5.0, help="Durée du comportement en secondes"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("🎭 Démo BBIAAdaptiveBehavior - Comportements adaptatifs")
    print(f"   • Contexte : {args.context}")
    print(f"   • Émotion : {args.emotion}")
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

        # Créer module adaptatif
        adaptive = BBIAAdaptiveBehavior(robot_api=robot_api)

        # Configurer contexte et émotion
        print("\n🎯 Configuration contexte et émotion...")
        adaptive.set_context(args.context, confidence=0.9)
        adaptive.set_emotion_state(args.emotion, intensity=0.7)

        # Générer comportement adaptatif
        print(f"   • Génération comportement pour contexte '{args.context}'...")
        behavior = adaptive.generate_behavior(trigger="demo")

        if behavior:
            print(f"   • Comportement généré : {behavior.get('name', 'N/A')}")
            print(f"   • Description : {behavior.get('description', 'N/A')}")

            # Exécuter comportement
            success = adaptive.execute_behavior(behavior, robot_api=robot_api)
        else:
            print("   ⚠️ Aucun comportement généré")
            success = False

        if success:
            print("✅ Comportement adaptatif exécuté avec succès")
        else:
            print("⚠️ Comportement adaptatif terminé avec avertissements")

        # Afficher historique
        history = adaptive.behavior_history
        if history:
            print(f"\n📊 Historique comportements : {len(history)} entrées")
            for i, entry in enumerate(history[-3:], 1):  # 3 derniers
                print(
                    f"   {i}. {entry.get('name', 'N/A')} - {entry.get('context', 'N/A')}"
                )

        return 0

    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par l'utilisateur")
        return 0
    except Exception as e:
        print(f"❌ Erreur : {e}")
        import traceback

        traceback.print_exc()
        return 1
    finally:
        backend.disconnect()


if __name__ == "__main__":
    sys.exit(main())
