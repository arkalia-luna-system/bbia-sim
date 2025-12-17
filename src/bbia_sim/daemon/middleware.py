"""Middleware de sécurité pour BBIA-SIM."""

import logging
import time
from collections import deque
from collections.abc import Callable
from typing import Any

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

from .config import settings

logger = logging.getLogger(__name__)


class SecurityMiddleware(BaseHTTPMiddleware):
    """Middleware pour appliquer les headers de sécurité."""

    def __init__(self, app: Any, max_json_size: int | None = None) -> None:
        super().__init__(app)
        self.max_json_size = max_json_size

    async def dispatch(
        self,
        request: Request,
        call_next: Callable[[Request], Any],
    ) -> Response:
        """Applique les headers de sécurité et limite la taille des requêtes."""
        # Vérification de la taille de la requête
        content_length = request.headers.get("content-length")
        max_size = self.max_json_size or settings.max_request_size
        if content_length and int(content_length) > max_size:
            logger.warning(
                f"Requête trop volumineuse rejetée: {content_length} bytes "
                f"(limite: {max_size})",
            )
            return Response(
                content="Request too large",
                status_code=413,
                headers=settings.get_security_headers(),
            )

        # Traitement de la requête
        start_time = time.time()
        response: Response = await call_next(request)
        process_time = time.time() - start_time

        # Application des headers de sécurité
        security_headers = settings.get_security_headers()
        for header, value in security_headers.items():
            response.headers[header] = value

        # Log de sécurité en production
        if settings.is_production():
            logger.info(
                f"Request: {request.method} {request.url.path} "
                f"Status: {response.status_code} "
                f"Time: {process_time:.3f}s",
            )

        return response


class RateLimitMiddleware(BaseHTTPMiddleware):
    """Middleware simple de rate limiting en mémoire."""

    def __init__(
        self,
        app: Any,
        requests_per_minute: int = 100,
        window_seconds: int = 60,
        message: str = "Rate limit exceeded",
        force_enable: bool = False,
    ) -> None:
        super().__init__(app)
        self.requests_per_minute = requests_per_minute
        self.window_seconds = window_seconds
        self.message = message
        self.force_enable = force_enable
        # OPTIMISATION RAM: Utiliser deque avec maxlen pour limiter taille
        # Max 2x la limite pour garder un peu d'historique
        self._max_timestamps = requests_per_minute * 2
        self.requests: dict[str, deque[float]] = {}

    async def dispatch(
        self,
        request: Request,
        call_next: Callable[[Request], Any],
    ) -> Response:
        """Applique le rate limiting basique."""
        if settings.is_production() or self.force_enable:
            client_ip = request.client.host if request.client else "unknown"
            now = time.time()

            # OPTIMISATION RAM: Nettoyage des anciennes requêtes
            if client_ip in self.requests:
                # Créer nouveau deque avec seulement les timestamps récents
                request_times = self.requests[client_ip]
                recent_times = deque(
                    (
                        req_time
                        for req_time in request_times
                        if now - req_time < self.window_seconds
                    ),
                    maxlen=request_times.maxlen or self._max_timestamps,
                )
                self.requests[client_ip] = recent_times
            else:
                # OPTIMISATION RAM: Initialiser avec deque limité
                self.requests[client_ip] = deque(maxlen=self._max_timestamps)

            # Vérification de la limite
            if len(self.requests[client_ip]) >= self.requests_per_minute:
                logger.warning("Rate limit dépassé pour %s", client_ip)
                return Response(
                    content=self.message,
                    status_code=429,
                    headers={"Retry-After": str(self.window_seconds)},
                )

            # Ajout de la requête actuelle
            self.requests[client_ip].append(now)

        response: Response = await call_next(request)
        return response
