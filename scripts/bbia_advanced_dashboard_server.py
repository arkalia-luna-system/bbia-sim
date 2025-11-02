#!/usr/bin/env python3
"""bbia_advanced_dashboard_server.py - Serveur dashboard BBIA avancé
Script pour lancer le dashboard web sophistiqué avec métriques temps réel
"""

import argparse
import logging
import sys
from pathlib import Path

# Ajouter le chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.dashboard_advanced import FASTAPI_AVAILABLE, run_advanced_dashboard


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description="BBIA Advanced Dashboard")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Adresse d'écoute",
    )  # nosec B104
    parser.add_argument("--port", type=int, default=8000, help="Port d'écoute")
    parser.add_argument(
        "--backend",
        choices=["mujoco", "reachy", "reachy_mini"],
        default="mujoco",
        help="Backend robot à utiliser",
    )
    parser.add_argument(
        "--log-level",
        choices=["debug", "info", "warning", "error"],
        default="info",
        help="Niveau de log",
    )

    args = parser.parse_args()

    # Configuration du logging
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    if not FASTAPI_AVAILABLE:
        print("❌ FastAPI non disponible. Installez avec: pip install fastapi uvicorn")
        sys.exit(1)

    print("🚀 Lancement BBIA Advanced Dashboard...")
    print("📊 Métriques temps réel: Activées")
    print("🎮 Contrôles avancés: Disponibles")
    print("📈 Graphiques: Chart.js intégré")
    print("🔧 API REST: Endpoints complets")
    print("🌐 WebSocket: Communication temps réel")
    print()

    try:
        run_advanced_dashboard(host=args.host, port=args.port, backend=args.backend)
    except KeyboardInterrupt:
        print("\n👋 Arrêt du dashboard avancé")
    except Exception as e:
        print(f"❌ Erreur: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
