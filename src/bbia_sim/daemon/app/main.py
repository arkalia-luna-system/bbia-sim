"""Application FastAPI principale pour BBIA-SIM."""

import logging
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from typing import Any

from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

from ..config import settings
from ..middleware import RateLimitMiddleware, SecurityMiddleware
from ..simulation_service import simulation_service
from ..ws import telemetry
from .routers import ecosystem, motion, state

# Configuration du logging
logging.basicConfig(
    level=getattr(logging, settings.log_level.upper()),
    format=settings.log_format,
)
logger = logging.getLogger(__name__)

# Configuration de sécurité
security = HTTPBearer()

# Variables globales pour l'état de l'application
app_state: dict[str, Any] = {
    "simulator": None,
    "is_running": False,
    "robot_state": {
        "position": {"x": 0, "y": 0, "z": 0},
        "status": "ready",
        "battery": 85.5,
        "temperature": 25.5,
    },
}


def verify_token(credentials: HTTPAuthorizationCredentials = Depends(security)) -> str:
    """Vérifie le token d'authentification.

    Args:
        credentials: Credentials HTTP Bearer

    Returns:
        Token vérifié

    Raises:
        HTTPException: Si le token est invalide

    """
    if credentials.credentials != settings.api_token:
        # Log sécurisé sans exposer le token
        logger.warning(
            f"Tentative d'authentification avec token invalide: {settings.mask_token(credentials.credentials)}"
        )
        raise HTTPException(
            status_code=401,
            detail="Token d'authentification invalide",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Log de succès en mode debug uniquement
    if settings.log_level.upper() == "DEBUG":
        logger.debug(
            f"Authentification réussie avec token: {settings.mask_token(credentials.credentials)}"
        )

    return credentials.credentials


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Gestionnaire de cycle de vie de l'application."""
    # Démarrage
    logger.info("🚀 Démarrage de l'API BBIA-SIM")

    # Démarrage de la simulation MuJoCo
    sim_config = settings.get_simulation_config()
    success = await simulation_service.start_simulation(headless=sim_config["headless"])

    if success:
        logger.info("✅ Simulation MuJoCo démarrée avec succès")
        app_state["simulator"] = simulation_service
        app_state["is_running"] = True
    else:
        logger.warning("⚠️ Échec du démarrage de la simulation MuJoCo")
        app_state["simulator"] = None
        app_state["is_running"] = False

    yield

    # Arrêt
    logger.info("🛑 Arrêt de l'API BBIA-SIM")

    # Arrêt de la simulation
    if app_state["simulator"]:
        await simulation_service.stop_simulation()
        app_state["simulator"] = None
        app_state["is_running"] = False


# Création de l'application FastAPI
app = FastAPI(
    title="BBIA-SIM API - Écosystème Reachy Mini",
    description="""
    ## 🚀 API BBIA-SIM v1.2.0 - Écosystème Reachy Mini

    **API publique pour le contrôle du robot Reachy Mini avec modules BBIA (Bio-Inspired Artificial Intelligence)**

    ### 🎯 Fonctionnalités Principales

    - **🤖 Contrôle Robot** : Mouvements, poses, états
    - **😊 Émotions BBIA** : 12 émotions contrôlant les articulations
    - **🎭 Comportements** : Actions complexes prédéfinies
    - **📊 Télémétrie** : WebSocket temps réel
    - **🎮 Modes Démo** : Simulation, robot réel, mixte

    ### 🔧 Backends Supportés

    - **MuJoCo** : Simulation physique réaliste
    - **Reachy Mini SDK** : Robot physique officiel
    - **Reachy Mock** : Mode développement

    ### 📚 Documentation

    - **Swagger UI** : Interface interactive `/docs`
    - **ReDoc** : Documentation alternative `/redoc`
    - **OpenAPI** : Spécification complète `/openapi.json`

    ### 🔐 Authentification

    Utilisez le token Bearer dans l'en-tête Authorization :
    ```
    Authorization: Bearer YOUR_API_TOKEN
    ```

    ### 🌐 WebSocket

    Connexion temps réel pour la télémétrie :
    ```
    ws://localhost:8000/ws/telemetry
    ```
    """,
    version="1.2.0",
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
    lifespan=lifespan,
    contact={
        "name": "Arkalia Luna System",
        "email": "arkalia.luna.system@gmail.com",
        "url": "https://github.com/arkalia-luna-system/bbia-sim",
    },
    license_info={
        "name": "MIT License",
        "url": "https://opensource.org/licenses/MIT",
    },
    servers=[
        {
            "url": "http://localhost:8000",
            "description": "Serveur de développement local",
        },
        {
            "url": "https://api.bbia-sim.com",
            "description": "Serveur de production (à venir)",
        },
    ],
    tags_metadata=[
        {
            "name": "ecosystem",
            "description": (
                "**🌐 Écosystème BBIA-SIM** - Endpoints publics pour l'intégration et la démonstration"
            ),
            "externalDocs": {
                "description": "Documentation complète",
                "url": "https://github.com/arkalia-luna-system/bbia-sim#readme",
            },
        },
        {
            "name": "motion",
            "description": (
                "**🤖 Contrôle des Mouvements** - Gestion des poses et mouvements du robot"
            ),
        },
        {
            "name": "state",
            "description": (
                "**📊 État du Robot** - Informations sur l'état, la batterie, les capteurs"
            ),
        },
        {
            "name": "telemetry",
            "description": "**📡 Télémétrie** - WebSocket pour les données temps réel",
        },
    ],
)

# Configuration CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.get_cors_origins(),
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE"],
    allow_headers=["*"],
)

# Middleware de sécurité
app.add_middleware(SecurityMiddleware)

# Rate limiting (en production uniquement)
if settings.is_production():
    app.add_middleware(
        RateLimitMiddleware, requests_per_minute=settings.rate_limit_requests
    )

# Inclusion des routers
app.include_router(
    ecosystem.router,
    prefix="/api/ecosystem",
    tags=["ecosystem"],
)

app.include_router(
    state.router,
    prefix="/api/state",
    tags=["state"],
    dependencies=[Depends(verify_token)],
)

app.include_router(
    motion.router,
    prefix="/api/motion",
    tags=["motion"],
    dependencies=[Depends(verify_token)],
)

app.include_router(telemetry.router, prefix="/ws", tags=["telemetry"])


@app.get("/", response_class=JSONResponse)
async def root() -> dict[str, Any]:
    """Point d'entrée principal de l'API."""
    return {
        "message": "BBIA-SIM API - Écosystème Reachy Mini",
        "version": "1.2.0",
        "status": "running",
        "description": (
            "API publique pour le contrôle du robot Reachy Mini avec modules BBIA"
        ),
        "endpoints": {
            "ecosystem": "/api/ecosystem",
            "state": "/api/state",
            "motion": "/api/motion",
            "telemetry": "/ws/telemetry",
            "docs": "/docs",
            "redoc": "/redoc",
            "openapi": "/openapi.json",
        },
        "features": [
            "REST API",
            "WebSocket telemetry",
            "MuJoCo simulation",
            "Robot control",
            "BBIA emotions",
            "BBIA behaviors",
            "Demo modes",
            "OpenAPI documentation",
        ],
        "contact": {
            "name": "Arkalia Luna System",
            "email": "arkalia.luna.system@gmail.com",
            "github": "https://github.com/arkalia-luna-system/bbia-sim",
        },
    }


@app.get("/health", response_class=JSONResponse)
async def health_check() -> dict[str, Any]:
    """Vérification de l'état de santé de l'API."""
    return {
        "status": "healthy",
        "timestamp": "2025-10-22T14:42:00Z",
        "services": {
            "api": "running",
            "simulator": "available" if app_state["simulator"] else "unavailable",
            "robot": "ready",
        },
    }


@app.get("/api/info", response_class=JSONResponse)
async def api_info() -> dict[str, Any]:
    """Informations détaillées sur l'API."""
    return {
        "name": "BBIA-SIM API - Écosystème Reachy Mini",
        "version": "1.2.0",
        "description": (
            "API publique pour le contrôle du robot Reachy Mini avec modules BBIA"
        ),
        "phase": "Phase 3 - Ouverture Écosystème",
        "features": [
            "REST API",
            "WebSocket telemetry",
            "MuJoCo simulation",
            "Robot control",
            "BBIA emotions (12 émotions)",
            "BBIA behaviors (8 comportements)",
            "Demo modes (simulation, robot réel, mixte)",
            "OpenAPI documentation",
            "Authentication",
            "Rate limiting",
            "CORS support",
        ],
        "robot": {
            "model": "Reachy Mini Wireless",
            "joints": 16,
            "status": "ready",
            "backends": ["mujoco", "reachy_mini", "reachy"],
        },
        "bbia_modules": {
            "emotions": 12,
            "behaviors": 8,
            "vision": "YOLOv8n + MediaPipe",
            "audio": "Whisper STT + pyttsx3 TTS",
            "integration": "Complete",
        },
        "documentation": {
            "swagger": "/docs",
            "redoc": "/redoc",
            "openapi": "/openapi.json",
            "github": "https://github.com/arkalia-luna-system/bbia-sim",
        },
    }


# Gestionnaire d'erreurs global
@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException) -> JSONResponse:
    """Gestionnaire d'erreurs HTTP personnalisé."""
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.detail,
            "status_code": exc.status_code,
            "path": str(request.url),
        },
    )


@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception) -> JSONResponse:
    """Gestionnaire d'erreurs générales."""
    logger.error(f"Erreur non gérée : {exc}")
    return JSONResponse(
        status_code=500,
        content={
            "error": "Erreur interne du serveur",
            "status_code": 500,
            "path": str(request.url),
        },
    )


if __name__ == "__main__":
    import uvicorn

    # Configuration pour le développement
    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=True, log_level="info")
