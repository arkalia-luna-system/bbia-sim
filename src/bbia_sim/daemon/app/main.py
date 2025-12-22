"""Application FastAPI principale pour BBIA-SIM."""

import asyncio
import logging
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import APIRouter, Depends, FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from bbia_sim.daemon.config import settings
from bbia_sim.daemon.granular_rate_limit import GranularRateLimitMiddleware
from bbia_sim.daemon.middleware import SecurityMiddleware
from bbia_sim.daemon.simulation_service import simulation_service
from bbia_sim.daemon.ws import telemetry
from bbia_sim.robot_factory import RobotFactory

from .routers import (
    apps,
    daemon,
    ecosystem,
    kinematics,
    media,
    metrics,
    motion,
    motors,
    move,
    presets,
    state,
)

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
    "multi_backends": {},  # NOUVEAU: Support multi-backends simultanés
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
            "Tentative d'authentification avec token invalide: %s",
            settings.mask_token(credentials.credentials),
        )
        raise HTTPException(
            status_code=401,
            detail="Token d'authentification invalide",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Log de succès en mode debug uniquement
    if settings.log_level.upper() == "DEBUG":
        logger.debug(
            "Authentification réussie avec token: %s",
            settings.mask_token(credentials.credentials),
        )

    return str(credentials.credentials)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Gestionnaire de cycle de vie robuste avec retry et fallback."""
    # Démarrage
    logger.info("🚀 Démarrage de l'API BBIA-SIM")

    # NOUVEAU: Retry automatique startup simulation
    max_retries = 3
    retry_delay = 1.0

    for attempt in range(max_retries):
        try:
            sim_config = settings.get_simulation_config()
            success = await simulation_service.start_simulation(
                headless=sim_config["headless"]
            )

            if success:
                logger.info("✅ Simulation MuJoCo démarrée avec succès")
                app_state["simulator"] = simulation_service
                app_state["is_running"] = True
                break
            else:
                if attempt < max_retries - 1:
                    logger.warning(
                        "⚠️ Tentative %d/%d échouée, retry dans %.1fs...",
                        attempt + 1,
                        max_retries,
                        retry_delay,
                    )
                    await asyncio.sleep(retry_delay)
                else:
                    logger.warning(
                        "⚠️ Échec démarrage simulation après %d tentatives (fallback gracieux)",
                        max_retries,
                    )
                    app_state["simulator"] = None
                    app_state["is_running"] = False
        except Exception as e:
            if attempt < max_retries - 1:
                logger.warning(
                    "⚠️ Erreur démarrage (tentative %d/%d): %s",
                    attempt + 1,
                    max_retries,
                    e,
                )
                await asyncio.sleep(retry_delay)
            else:
                logger.error(
                    "❌ Échec démarrage après %d tentatives: %s (fallback gracieux)",
                    max_retries,
                    e,
                )
                app_state["simulator"] = None
                app_state["is_running"] = False

    # NOUVEAU: Initialiser multi-backends pour support simultané sim/robot
    try:
        multi_backends = RobotFactory.create_multi_backend(backends=["mujoco"])
        app_state["multi_backends"] = multi_backends
        logger.info(
            "✅ Multi-backends initialisés: %s",
            list(multi_backends.keys()),
        )
    except Exception as e:
        logger.warning("⚠️ Échec initialisation multi-backends: %s", e)
        app_state["multi_backends"] = {}

    # NOUVEAU: Health check avant de marquer "ready"
    # L'app démarre même si simulation échoue (fallback gracieux)
    logger.info(
        "✅ API BBIA-SIM prête (simulation: %s)",
        "OK" if app_state["is_running"] else "NON DISPONIBLE",
    )

    yield

    # Arrêt
    logger.info("🛑 Arrêt de l'API BBIA-SIM")

    # Issue #402: Arrêt propre même si dashboard ouvert
    # Fermer toutes les connexions WebSocket actives
    # telemetry déjà importé au niveau module
    try:
        if hasattr(telemetry, "manager") and telemetry.manager:
            await telemetry.manager.stop_broadcast()
            logger.info("✅ WebSocket telemetry arrêté")
    except Exception as e:
        logger.warning("⚠️ Erreur arrêt WebSocket telemetry: %s", e)

    try:
        from bbia_sim.dashboard_advanced import advanced_websocket_manager

        if advanced_websocket_manager and hasattr(
            advanced_websocket_manager,
            "active_connections",
        ):
            # Fermer toutes les connexions actives
            for ws in list(advanced_websocket_manager.active_connections):
                try:
                    await ws.close()
                except Exception as e:
                    logger.debug("Erreur fermeture WebSocket: %s", e)
            advanced_websocket_manager.active_connections.clear()
            logger.info("✅ WebSocket dashboard arrêté")
    except Exception as e:
        logger.warning("⚠️ Erreur arrêt WebSocket dashboard: %s", e)

    # Arrêt de la simulation
    if app_state["simulator"]:
        await simulation_service.stop_simulation()
        app_state["simulator"] = None
        app_state["is_running"] = False
        logger.info("✅ Simulation arrêtée")


# Création de l'application FastAPI
app = FastAPI(
    title="BBIA-SIM API - Écosystème Reachy Mini",
    description="""
    ## 🚀 API BBIA-SIM v1.3.2 - Écosystème Reachy Mini

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
    version="1.3.2",
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

# Rate limiting granulaire (en production uniquement)
if settings.is_production():
    app.add_middleware(
        GranularRateLimitMiddleware,
        default_requests_per_minute=settings.rate_limit_requests,
        default_window_seconds=settings.rate_limit_window,
    )

# Inclusion des routers (conforme SDK officiel: router parent /api avec sous-routers)
# Note: Les WebSockets ne peuvent pas utiliser HTTPBearer, donc on sépare les routers

# Routers SANS WebSockets (authentification globale)
api_router_http = APIRouter(prefix="/api")
api_router_http.include_router(motors.router)
api_router_http.include_router(kinematics.router)
app.include_router(api_router_http, dependencies=[Depends(verify_token)])

# Router daemon SANS auth pour permettre l'accès depuis le dashboard
# Note: Les endpoints daemon sont sécurisés mais le status doit être accessible
app.include_router(
    daemon.router,
    prefix="/api",
)  # Sans dépendance globale pour dashboard

# Router media SANS auth pour permettre l'accès depuis le dashboard
# Note: Les endpoints media sont accessibles depuis le dashboard
app.include_router(
    media.router,
)  # Préfixe /development/api/media déjà défini dans le router

# Routers AVEC WebSockets (auth via query params en prod)
# Note: Les WebSockets ne supportent pas HTTPBearer
# Les endpoints HTTP dans ces routers auront l'auth désactivée pour éviter conflits
# Auth WebSocket implémentée via query param `token` (optionnel en dev, requis en prod)
app.include_router(state.router, prefix="/api")  # Contient /ws/full + endpoints HTTP
app.include_router(
    move.router,
    prefix="/api",
)  # Contient /ws/updates, /ws/set_target + endpoints HTTP
app.include_router(
    apps.router,
    prefix="/api",
)  # Contient /ws/apps-manager/{job_id} + endpoints HTTP

# Routers BBIA supplémentaires (extensions légitimes)
app.include_router(
    ecosystem.router,
    prefix="/api/ecosystem",
    tags=["ecosystem"],
)
app.include_router(
    motion.router,
    prefix="/api/motion",
    tags=["motion"],
    dependencies=[Depends(verify_token)],
)
app.include_router(telemetry.router, prefix="/ws", tags=["telemetry"])
app.include_router(metrics.router, tags=["metrics"])  # /metrics/*, /healthz, /readyz
app.include_router(
    presets.router,
    prefix="/api/presets",
    tags=["presets"],
)  # /api/presets/*


# Endpoint JSON racine pour les tests et API clients
@app.get("/api", response_class=JSONResponse)
@app.get("/api/", response_class=JSONResponse)
async def root_api() -> dict[str, Any]:
    """Point d'entrée principal de l'API en JSON."""
    return {
        "message": "BBIA-SIM API - Écosystème Reachy Mini",
        "version": "1.3.2",
        "status": "running",
        "description": (
            "API publique pour le contrôle du robot Reachy Mini avec modules BBIA"
        ),
        "endpoints": {
            "ecosystem": "/api/ecosystem",
            "state": "/api/state",
            "motion": "/api/motion",
            "move": "/api/move",
            "motors": "/api/motors",
            "daemon": "/api/daemon",
            "kinematics": "/api/kinematics",
            "apps": "/api/apps",
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


# Dashboard (conforme SDK officiel)
STATIC_DIR = Path(__file__).parent / "dashboard" / "static"
TEMPLATES_DIR = Path(__file__).parent / "dashboard" / "templates"

if STATIC_DIR.exists() and TEMPLATES_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
    templates = Jinja2Templates(directory=str(TEMPLATES_DIR))

    @app.get("/", response_class=HTMLResponse)
    async def dashboard(request: Request) -> HTMLResponse:
        """Render the dashboard (conforme SDK officiel)."""
        return templates.TemplateResponse("index.html", {"request": request})

    @app.get("/test-logos", response_class=HTMLResponse)
    async def test_logos(request: Request) -> HTMLResponse:
        """Page de test pour voir tous les logos BBIA."""
        return templates.TemplateResponse("test_logos.html", {"request": request})

    @app.get("/bbia-interface", response_class=HTMLResponse)
    async def bbia_interface(request: Request) -> HTMLResponse:
        """Interface live BBIA avec animations et contrôles."""
        return templates.TemplateResponse("bbia_interface.html", {"request": request})

else:
    logger.warning(
        "Dashboard templates non trouvés. Dashboard non disponible. "
        "STATIC_DIR: %s, TEMPLATES_DIR: %s",
        STATIC_DIR,
        TEMPLATES_DIR,
    )

    @app.get("/", response_class=JSONResponse)
    async def root() -> dict[str, Any]:
        """Point d'entrée principal de l'API (fallback si dashboard non disponible)."""
        return {
            "message": "BBIA-SIM API - Écosystème Reachy Mini",
            "version": "1.3.2",
            "status": "running",
            "description": (
                "API publique pour le contrôle du robot Reachy Mini avec modules BBIA"
            ),
            "endpoints": {
                "ecosystem": "/api/ecosystem",
                "state": "/api/state",
                "motion": "/api/motion",
                "move": "/api/move",
                "motors": "/api/motors",
                "daemon": "/api/daemon",
                "kinematics": "/api/kinematics",
                "apps": "/api/apps",
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
        "version": "1.3.2",
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
    logger.error("Erreur non gérée : %s", exc)
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
    # Utiliser string d'import complète pour éviter problèmes
    uvicorn.run(
        "bbia_sim.daemon.app.main:app",
        host="127.0.0.1",
        port=8000,
        reload=False,  # Désactiver reload pour stabilité
        log_level="info",
    )
