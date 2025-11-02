#!/usr/bin/env python3
"""
Configuration pour déploiement public de la documentation BBIA-SIM
Déploie Swagger UI et ReDoc sur Render.com ou Fly.io
"""

import os
import sys
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import RedirectResponse

# Import des modules BBIA-SIM
from bbia_sim.daemon.app.main import app as main_app
from bbia_sim.daemon.app.routers.sanity import router as sanity_router
from bbia_sim.daemon.bridge import app as bridge_app

# Configuration pour déploiement public
app = FastAPI(
    title="BBIA-SIM Public API",
    description="""
    # 🤖 BBIA-SIM - Reachy Mini Simulation API

    **API publique pour la simulation robotique BBIA-SIM**

    ## 🎯 Fonctionnalités

    - **RobotAPI Unifié** : Interface simulation ↔ robot réel
    - **Modules BBIA** : IA cognitive avancée
    - **Bridge Zenoh** : Communication distribuée
    - **Conformité SDK** : 100% conforme au SDK officiel Reachy Mini

    ## 🚀 Quick Start

    ```python
    from bbia_sim.robot_factory import RobotFactory

    # Créer robot simulation
    robot = RobotFactory.create_robot(backend="mujoco")
    robot.wake_up()
    robot.set_emotion("happy", 0.8)
    ```

    ## 📚 Documentation

    - **Architecture** : [ARCHITECTURE_OVERVIEW.md](https://github.com/arkalia-luna-system/bbia-sim/blob/develop/docs/ARCHITECTURE_OVERVIEW.md)
    - **Migration** : [MIGRATION_GUIDE.md](https://github.com/arkalia-luna-system/bbia-sim/blob/develop/docs/MIGRATION_GUIDE.md)
    - **Tests** : [Test Results](https://github.com/arkalia-luna-system/bbia-sim/actions)

    ## 🏆 Innovation Technique

    - **RobotAPI Unifié** : Innovation architecturale majeure
    - **Modules BBIA** : IA cognitive unique
    - **Conformité SDK** : 100% conforme SDK officiel
    - **Qualité Professionnelle** : Tests, CI/CD, documentation
    """,
    version="1.3.0-pre-release",
    contact={
        "name": "Arkalia Luna System",
        "email": "arkalia.luna.system@gmail.com",
        "url": "https://github.com/arkalia-luna-system/bbia-sim",
    },
    license_info={
        "name": "MIT License",
        "url": "https://github.com/arkalia-luna-system/bbia-sim/blob/develop/LICENSE",
    },
    servers=[
        {
            "url": "https://bbia-sim-docs.onrender.com",
            "description": "Production server",
        },
        {"url": "http://localhost:8000", "description": "Development server"},
    ],
)

# Configuration CORS pour accès public
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # En production, spécifier les domaines autorisés
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Redirection racine vers documentation
@app.get("/", response_class=RedirectResponse)
async def root():
    """Redirige vers la documentation Swagger."""
    return "/docs"


# Endpoints de santé pour monitoring
@app.get("/health")
async def health_check():
    """Vérification de santé de l'API."""
    return {
        "status": "healthy",
        "version": "1.3.0-pre-release",
        "service": "BBIA-SIM Public API",
        "timestamp": "2024-10-25T18:00:00Z",
    }


@app.get("/api/status")
async def api_status():
    """Statut détaillé de l'API."""
    return {
        "api": "BBIA-SIM Public API",
        "version": "1.3.0-pre-release",
        "status": "operational",
        "features": {
            "robot_api": "unified",
            "bbia_modules": "advanced",
            "sdk_conformity": "100%",
            "bridge_zenoh": "available",
            "dashboard": "web_realtime",
        },
        "endpoints": {
            "docs": "/docs",
            "redoc": "/redoc",
            "openapi": "/openapi.json",
            "health": "/health",
        },
    }


# Intégration des apps BBIA-SIM
app.mount("/api", main_app)
app.mount("/bridge", bridge_app)

# Ajouter routeur sanity dans l'API publique principale
app.include_router(sanity_router)

# Configuration pour déploiement
if __name__ == "__main__":
    # Configuration pour Render.com
    port = int(os.environ.get("PORT", 8000))
    host = os.environ.get("HOST", "0.0.0.0")

    uvicorn.run(app, host=host, port=port, log_level="info", access_log=True)
