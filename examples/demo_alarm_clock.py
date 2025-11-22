#!/usr/bin/env python3
"""Démo AlarmClockBehavior - Réveil intelligent avec interactions.

Démonstration du comportement alarm_clock avec séquence progressive.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.alarm_clock import AlarmClockBehavior


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo AlarmClockBehavior")
    parser.add_argument("--hour", type=int, default=8, help="Heure de réveil (0-23)")
    parser.add_argument("--minute", type=int, default=0, help="Minute de réveil (0-59)")
    parser.add_argument(
        "--snooze-minutes",
        type=int,
        default=5,
        help="Minutes de snooze",
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    # Validation
    if not (0 <= args.hour <= 23):
        print("❌ Heure doit être entre 0 et 23")
        return 1
    if not (0 <= args.minute <= 59):
        print("❌ Minute doit être entre 0 et 59")
        return 1

    print("⏰ Démo AlarmClockBehavior - Réveil intelligent")
    print(f"   • Heure : {args.hour:02d}:{args.minute:02d}")
    print(f"   • Snooze : {args.snooze_minutes} minutes")
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
        alarm_clock = AlarmClockBehavior(robot_api=backend)
        print("✅ AlarmClockBehavior créé")

        # Exécuter alarm_clock
        context = {
            "hour": args.hour,
            "minute": args.minute,
            "snooze_minutes": args.snooze_minutes,
        }
        print(f"\n🚀 Configuration réveil {args.hour:02d}:{args.minute:02d}...")
        success = alarm_clock.execute(context)

        if success:
            print("✅ Réveil configuré avec succès")
            return 0
        print("❌ Erreur durant la configuration")
        return 1

    except KeyboardInterrupt:
        print("\n⚠️  Interrompu par l'utilisateur")
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
