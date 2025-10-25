#!/usr/bin/env python3
"""Script de démarrage pour l'API publique BBIA-SIM - Phase 3."""

import argparse
import logging
import sys
from pathlib import Path

import uvicorn

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.daemon.app.main import app

# Configuration du logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


def main():
    """Point d'entrée principal pour l'API publique BBIA-SIM."""
    parser = argparse.ArgumentParser(
        description="🚀 API Publique BBIA-SIM v1.2.0 - Écosystème Reachy Mini",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation :

  # Démarrage en mode développement
  python scripts/start_public_api.py --dev

  # Démarrage en mode production
  python scripts/start_public_api.py --prod --port 8000

  # Démarrage avec logs détaillés
  python scripts/start_public_api.py --dev --log-level debug

  # Démarrage sur toutes les interfaces
  python scripts/start_public_api.py --host 0.0.0.0 --port 8000

Endpoints disponibles :
  - http://localhost:8000/          # Page d'accueil
  - http://localhost:8000/docs      # Swagger UI
  - http://localhost:8000/redoc     # ReDoc
  - http://localhost:8000/openapi.json # Spécification OpenAPI
  - http://localhost:8000/api/ecosystem/ # Endpoints écosystème
  - ws://localhost:8000/ws/telemetry   # WebSocket télémétrie
        """,
    )

    # Arguments de configuration
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Adresse IP d'écoute (défaut: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port d'écoute (défaut: 8000)",
    )
    parser.add_argument(
        "--dev",
        action="store_true",
        help="Mode développement avec rechargement automatique",
    )
    parser.add_argument(
        "--prod",
        action="store_true",
        help="Mode production optimisé",
    )
    parser.add_argument(
        "--log-level",
        choices=["debug", "info", "warning", "error"],
        default="info",
        help="Niveau de log (défaut: info)",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=1,
        help="Nombre de workers (défaut: 1)",
    )

    args = parser.parse_args()

    # Configuration selon le mode
    if args.dev and args.prod:
        logger.error("❌ Impossible de spécifier --dev et --prod simultanément")
        sys.exit(1)

    if args.dev:
        logger.info("🔧 Mode développement activé")
        reload = True
        workers = 1
    elif args.prod:
        logger.info("🚀 Mode production activé")
        reload = False
        workers = args.workers
    else:
        logger.info("⚙️ Mode par défaut (développement)")
        reload = True
        workers = 1

    # Configuration uvicorn
    config = {
        "app": app,
        "host": args.host,
        "port": args.port,
        "log_level": args.log_level,
        "reload": reload,
        "workers": workers,
        "access_log": True,
    }

    # Messages de démarrage
    logger.info("🚀 Démarrage de l'API Publique BBIA-SIM v1.2.0")
    logger.info(f"📍 Adresse: http://{args.host}:{args.port}")
    logger.info(f"📚 Documentation: http://{args.host}:{args.port}/docs")
    logger.info(f"🌐 Écosystème: http://{args.host}:{args.port}/api/ecosystem/")
    logger.info(f"📡 WebSocket: ws://{args.host}:{args.port}/ws/telemetry")

    if args.dev:
        logger.info("🔄 Rechargement automatique activé")
    if args.prod:
        logger.info(f"👥 Workers: {workers}")

    try:
        # Démarrage du serveur
        uvicorn.run(**config)
    except KeyboardInterrupt:
        logger.info("🛑 Arrêt de l'API demandé par l'utilisateur")
    except Exception as e:
        logger.error(f"❌ Erreur lors du démarrage: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
