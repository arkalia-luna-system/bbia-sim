#!/usr/bin/env python3
"""Script de démarrage pour l'API BBIA-SIM."""

import os
import sys
from pathlib import Path

import uvicorn

# Ajout du chemin src au PYTHONPATH
src_path = Path(__file__).parent / "src"
sys.path.insert(0, str(src_path))


def main():
    """Point d'entrée principal."""
    # Configuration
    host = os.getenv("BBIA_HOST", "127.0.0.1")  # nosec B104
    port = int(os.getenv("BBIA_PORT", "8000"))
    reload = os.getenv("BBIA_RELOAD", "true").lower() == "true"

    # Démarrage du serveur
    uvicorn.run(
        "bbia_sim.daemon.app.main:app",
        host=host,
        port=port,
        reload=reload,
        log_level="info",
    )


if __name__ == "__main__":
    main()
