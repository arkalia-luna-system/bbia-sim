#!/usr/bin/env python3
"""Démo BBIA Awake - Séquence de réveil optimisée.

Démonstration de la séquence de réveil BBIA avec intelligence et variété.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_awake import start_bbia_sim


def main() -> int:
    """Fonction principale."""
    parser = argparse.ArgumentParser(description="Démo BBIA Awake")
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Mode verbose pour plus de détails",
    )

    args = parser.parse_args()

    print("🌅 Démo BBIA Awake - Séquence de réveil optimisée")
    print("=" * 60)

    try:
        # Exécuter séquence de réveil
        start_bbia_sim()

        print("\n✅ Séquence de réveil terminée avec succès")
        return 0

    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par l'utilisateur")
        return 0
    except Exception as e:
        print(f"❌ Erreur : {e}")
        if args.verbose:
            import traceback

            traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
