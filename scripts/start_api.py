#!/usr/bin/env python3
"""Script de démarrage pour l'API BBIA-SIM."""

import os
import sys
import uvicorn
from pathlib import Path

# Ajout du chemin src au PYTHONPATH
src_path = Path(__file__).parent / "src"
sys.path.insert(0, str(src_path))

def main():
    """Point d'entrée principal."""
    print("🚀 Démarrage de l'API BBIA-SIM...")
    
    # Configuration
    host = os.getenv("BBIA_HOST", "0.0.0.0")
    port = int(os.getenv("BBIA_PORT", "8000"))
    reload = os.getenv("BBIA_RELOAD", "true").lower() == "true"
    
    print(f"📍 Serveur : http://{host}:{port}")
    print(f"📚 Documentation : http://{host}:{port}/docs")
    print(f"🔄 Reload : {'Activé' if reload else 'Désactivé'}")
    
    # Démarrage du serveur
    uvicorn.run(
        "bbia_sim.daemon.app.main:app",
        host=host,
        port=port,
        reload=reload,
        log_level="info"
    )

if __name__ == "__main__":
    main()
