"""Point d'entrée pour exécuter le daemon BBIA-SIM comme module.

Permet l'exécution via: python -m bbia_sim.daemon.app.main

Note: Fonctionne en mode simulation (MuJoCo) même sans robot physique.
Le daemon démarre automatiquement la simulation MuJoCo au démarrage.
"""

import sys
from pathlib import Path

# Ajouter src au path pour imports (depuis racine projet)
project_root = Path(__file__).parent.parent.parent.parent.parent
src_path = project_root / "src"
if str(src_path) not in sys.path:
    sys.path.insert(0, str(src_path))

"""Point d'entrée principal pour le daemon BBIA."""
import uvicorn

from bbia_sim.daemon.app.main import (
    app,
)  # noqa: E402 - Import après configuration uvicorn

if __name__ == "__main__":
    print("🚀 Démarrage du daemon BBIA-SIM (mode simulation)")
    print("📍 Dashboard: http://127.0.0.1:8000/")
    print("📚 API Docs: http://127.0.0.1:8000/docs")
    print("💡 Note: Mode simulation activé (robot physique non requis)")

    # Importer directement l'app (pas de string) pour éviter problèmes reload
    uvicorn.run(
        app,  # Objet app directement (pas de string)
        host="127.0.0.1",
        port=8000,
        reload=False,  # Pas de reload pour éviter problèmes
        log_level="info",
    )
