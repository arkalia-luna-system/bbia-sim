"""Application FastAPI principale pour BBIA-SIM."""

import logging
import os
from contextlib import asynccontextmanager
from typing import Any

from fastapi import Depends, FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

from ..ws import telemetry
from .routers import motion, state

# Configuration du logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration de sécurité
security = HTTPBearer()
API_TOKEN = os.getenv("BBIA_API_TOKEN", "bbia-secret-key-dev")

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
    if credentials.credentials != API_TOKEN:
        raise HTTPException(
            status_code=401,
            detail="Token d'authentification invalide",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return credentials.credentials


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Gestionnaire de cycle de vie de l'application."""
    # Démarrage
    logger.info("🚀 Démarrage de l'API BBIA-SIM")

    # Initialisation du simulateur (optionnel)
    try:
        from ...sim.simulator import MuJoCoSimulator

        model_path = os.path.join(
            os.path.dirname(__file__), "..", "..", "sim", "models", "reachy_mini.xml"
        )
        if os.path.exists(model_path):
            app_state["simulator"] = MuJoCoSimulator(model_path)
            logger.info("✅ Simulateur MuJoCo initialisé")
    except Exception as e:
        logger.warning(f"⚠️ Simulateur MuJoCo non disponible : {e}")

    yield

    # Arrêt
    logger.info("🛑 Arrêt de l'API BBIA-SIM")
    if app_state["simulator"]:
        app_state["simulator"].stop()


# Création de l'application FastAPI
app = FastAPI(
    title="BBIA-SIM API",
    description="API REST et WebSocket pour le contrôle du robot Reachy Mini",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
    lifespan=lifespan,
)

# Configuration CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # En production, spécifier les domaines autorisés
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Inclusion des routers
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
        "message": "BBIA-SIM API",
        "version": "1.0.0",
        "status": "running",
        "endpoints": {
            "state": "/api/state",
            "motion": "/api/motion",
            "telemetry": "/ws/telemetry",
            "docs": "/docs",
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
        "name": "BBIA-SIM API",
        "version": "1.0.0",
        "description": "API pour le contrôle du robot Reachy Mini",
        "features": [
            "REST API",
            "WebSocket telemetry",
            "MuJoCo simulation",
            "Robot control",
            "Authentication",
        ],
        "robot": {"model": "Reachy Mini", "joints": 8, "status": "ready"},
    }


# Gestionnaire d'erreurs global
@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc: HTTPException):
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
async def general_exception_handler(request, exc: Exception):
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
