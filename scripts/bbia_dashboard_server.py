#!/usr/bin/env python3
"""
bbia_dashboard_server.py - Serveur dashboard BBIA
Script pour lancer le dashboard web minimal
"""

import argparse
import logging
import sys
from pathlib import Path

# Ajouter le chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.dashboard import FASTAPI_AVAILABLE, run_dashboard


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description="Dashboard BBIA")
    parser.add_argument(
        "--host", default="127.0.0.1", help="Adresse d'écoute"
    )  # nosec B104
    parser.add_argument("--port", type=int, default=8000, help="Port d'écoute")
    parser.add_argument(
        "--backend",
        choices=["mujoco", "reachy"],
        default="mujoco",
        help="Backend robot à utiliser",
    )
    parser.add_argument(
        "--headless", action="store_true", help="Mode headless (pour CI)"
    )

    args = parser.parse_args()

    # Configuration logging
    if args.headless:
        logging.basicConfig(level=logging.WARNING)
    else:
        logging.basicConfig(
            level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
        )

    print("🤖 Dashboard BBIA")
    print("=" * 40)

    if not FASTAPI_AVAILABLE:
        print("❌ FastAPI non disponible")
        print("Installez avec: pip install fastapi uvicorn websockets")
        return 1

    print(f"🌐 Serveur: {args.host}:{args.port}")
    print(f"🤖 Backend: {args.backend}")
    print(f"🔗 URL: http://{args.host}:{args.port}")

    if args.headless:
        print("🔇 Mode headless activé")

    try:
        run_dashboard(host=args.host, port=args.port, backend=args.backend)
    except KeyboardInterrupt:
        print("\n👋 Arrêt du dashboard")
        return 0
    except Exception as e:
        print(f"❌ Erreur: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
