#!/usr/bin/env python3
"""Exemple simple : lancement simulation MuJoCo headless.

Usage:
    python examples/hello_sim.py [--duration SECONDS] [--verbose]

Exemple:
    python examples/hello_sim.py --duration 2 --verbose
"""

import argparse
import logging
import sys
import time
from pathlib import Path

# Ajouter le répertoire parent au path pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.bbia_sim.sim.simulator import MuJoCoSimulator


def main():
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Lancement simulation MuJoCo headless")
    parser.add_argument(
        "--duration", type=int, default=2, help="Durée en secondes (défaut: 2)"
    )
    parser.add_argument("--verbose", action="store_true", help="Mode verbeux")

    args = parser.parse_args()

    # Configuration logging
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level, format="%(levelname)s: %(message)s")

    try:
        print(f"🚀 Lancement simulation MuJoCo headless ({args.duration}s)")

        # Initialisation simulateur
        model_path = "src/bbia_sim/sim/models/reachy_mini.xml"
        simulator = MuJoCoSimulator(model_path)

        # Mesure des performances
        start_time = time.time()

        print("📊 Démarrage simulation...")

        # Simulation headless avec comptage
        simulator.launch_simulation(headless=True, duration=args.duration)

        # Calcul des métriques
        elapsed_time = time.time() - start_time
        # Estimation basée sur la durée et la fréquence MuJoCo (~1000 Hz)
        estimated_steps = int(elapsed_time * 1000)
        steps_per_second = estimated_steps / elapsed_time if elapsed_time > 0 else 0

        print("✅ Simulation terminée")
        print(f"📈 Performance: {steps_per_second:.1f} steps/s")
        print(f"⏱️  Temps: {elapsed_time:.2f}s")

        return 0

    except FileNotFoundError as e:
        print(f"❌ Erreur: Modèle MuJoCo introuvable - {e}")
        return 1
    except Exception as e:
        print(f"❌ Erreur: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
