"""Rate limiting granulaire par endpoint pour BBIA-SIM."""

import logging
import time
from collections.abc import Callable
from typing import Any

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

from .config import settings

logger = logging.getLogger(__name__)

# Configuration des limites par endpoint/router
ENDPOINT_RATE_LIMITS: dict[str, dict[str, int]] = {
    # Motion endpoints (plus restrictifs - mouvements coûteux)
    "/api/move": {"requests_per_minute": 30, "window_seconds": 60},
    "/api/motion": {"requests_per_minute": 30, "window_seconds": 60},
    # State endpoints (modérés - lecture fréquente)
    "/api/state": {"requests_per_minute": 60, "window_seconds": 60},
    # Media endpoints (modérés - contrôle média)
    "/api/media": {"requests_per_minute": 40, "window_seconds": 60},
    # Apps endpoints (modérés - installation/désinstallation)
    "/api/apps": {"requests_per_minute": 20, "window_seconds": 60},
    # Motors endpoints (modérés - contrôle moteurs)
    "/api/motors": {"requests_per_minute": 50, "window_seconds": 60},
    # Presets endpoints (modérés - presets)
    "/api/presets": {"requests_per_minute": 40, "window_seconds": 60},
    # Metrics endpoints (plus permissifs - monitoring)
    "/api/metrics": {"requests_per_minute": 100, "window_seconds": 60},
    # Ecosystem endpoints (permissifs - endpoints publics)
    "/api/ecosystem": {"requests_per_minute": 200, "window_seconds": 60},
    # Daemon endpoints (restrictifs - contrôle système)
    "/api/daemon": {"requests_per_minute": 10, "window_seconds": 60},
    # Kinematics endpoints (modérés - calculs IK)
    "/api/kinematics": {"requests_per_minute": 50, "window_seconds": 60},
}


class GranularRateLimitMiddleware(BaseHTTPMiddleware):
    """Middleware de rate limiting granulaire par endpoint."""

    def __init__(
        self,
        app: Any,
        default_requests_per_minute: int = 100,
        default_window_seconds: int = 60,
        force_enable: bool = False,
    ) -> None:
        """Initialise le middleware de rate limiting granulaire.

        Args:
            app: Application FastAPI
            default_requests_per_minute: Limite par défaut si endpoint non configuré
            default_window_seconds: Fenêtre temporelle par défaut
            force_enable: Forcer l'activation même en dev
        """
        super().__init__(app)
        self.default_requests_per_minute = default_requests_per_minute
        self.default_window_seconds = default_window_seconds
        self.force_enable = force_enable
        # Structure: {client_ip: {endpoint: [timestamps]}}
        self.requests: dict[str, dict[str, list[float]]] = {}

    def _get_endpoint_key(self, path: str) -> str:
        """Extrait la clé d'endpoint depuis le chemin.

        Args:
            path: Chemin de la requête (e.g., "/api/move/goto")

        Returns:
            Clé d'endpoint (e.g., "/api/move")
        """
        # Extraire le préfixe d'endpoint (e.g., "/api/move" depuis "/api/move/goto")
        parts = path.split("/")
        if len(parts) >= 3:
            return f"/{parts[1]}/{parts[2]}"
        return path

    def _get_rate_limit_config(self, endpoint_key: str) -> dict[str, int]:
        """Récupère la configuration de rate limit pour un endpoint.

        Args:
            endpoint_key: Clé d'endpoint (e.g., "/api/move")

        Returns:
            Configuration avec requests_per_minute et window_seconds
        """
        return ENDPOINT_RATE_LIMITS.get(
            endpoint_key,
            {
                "requests_per_minute": self.default_requests_per_minute,
                "window_seconds": self.default_window_seconds,
            },
        )

    async def dispatch(
        self,
        request: Request,
        call_next: Callable[[Request], Any],
    ) -> Response:
        """Applique le rate limiting granulaire."""
        if not (settings.is_production() or self.force_enable):
            # En dev, pas de rate limiting sauf si force_enable
            response: Response = await call_next(request)
            return response

        client_ip = request.client.host if request.client else "unknown"
        endpoint_key = self._get_endpoint_key(request.url.path)
        config = self._get_rate_limit_config(endpoint_key)
        requests_per_minute = config["requests_per_minute"]
        window_seconds = config["window_seconds"]
        now = time.time()

        # Initialiser les structures si nécessaire
        if client_ip not in self.requests:
            self.requests[client_ip] = {}
        if endpoint_key not in self.requests[client_ip]:
            self.requests[client_ip][endpoint_key] = []

        # Nettoyage des anciennes requêtes pour cet endpoint
        self.requests[client_ip][endpoint_key] = [
            req_time
            for req_time in self.requests[client_ip][endpoint_key]
            if now - req_time < window_seconds
        ]

        # Vérification de la limite
        if len(self.requests[client_ip][endpoint_key]) >= requests_per_minute:
            logger.warning(
                "Rate limit dépassé pour %s sur endpoint %s (%d req/min)",
                client_ip,
                endpoint_key,
                requests_per_minute,
            )
            return Response(
                content=f"Rate limit exceeded for {endpoint_key}: {requests_per_minute} requests per minute",
                status_code=429,
                headers={
                    "Retry-After": str(window_seconds),
                    "X-RateLimit-Limit": str(requests_per_minute),
                    "X-RateLimit-Remaining": "0",
                    "X-RateLimit-Reset": str(int(now + window_seconds)),
                },
            )

        # Ajout de la requête actuelle
        self.requests[client_ip][endpoint_key].append(now)

        # Traitement de la requête
        response = await call_next(request)

        # Ajout des headers de rate limit
        remaining = requests_per_minute - len(self.requests[client_ip][endpoint_key])
        response.headers["X-RateLimit-Limit"] = str(requests_per_minute)
        response.headers["X-RateLimit-Remaining"] = str(remaining)
        response.headers["X-RateLimit-Reset"] = str(int(now + window_seconds))

        return response
