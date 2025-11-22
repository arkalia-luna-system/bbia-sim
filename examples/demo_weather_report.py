#!/usr/bin/env python3
"""Démo WeatherReportBehavior - Rapport météo avec gestes expressifs.

Démonstration du comportement weather_report avec mouvements selon conditions.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.behaviors.weather_report import WeatherReportBehavior


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo WeatherReportBehavior")
    parser.add_argument("--city", default="Paris", help="Ville pour météo")
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument(
        "--backend",
        default="mujoco",
        choices=["mujoco", "reachy_mini"],
        help="Backend à utiliser",
    )

    args = parser.parse_args()

    print("🌤️  Démo WeatherReportBehavior - Rapport météo")
    print(f"   • Ville : {args.city}")
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
        weather_report = WeatherReportBehavior(robot_api=backend)
        print("✅ WeatherReportBehavior créé")

        # Exécuter weather_report
        context = {"city": args.city}
        print(f"\n🚀 Démarrage rapport météo pour '{args.city}'...")
        success = weather_report.execute(context)

        if success:
            print("✅ Rapport météo terminé avec succès")
            return 0
        print("❌ Erreur durant le rapport")
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
