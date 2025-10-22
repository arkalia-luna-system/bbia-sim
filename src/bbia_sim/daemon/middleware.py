"""Middleware de sécurité pour BBIA-SIM."""

import logging
import time
from typing import Callable

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware

from .config import settings

logger = logging.getLogger(__name__)


class SecurityMiddleware(BaseHTTPMiddleware):
    """Middleware pour appliquer les headers de sécurité."""

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Applique les headers de sécurité et limite la taille des requêtes."""
        # Vérification de la taille de la requête
        content_length = request.headers.get("content-length")
        if content_length and int(content_length) > settings.max_request_size:
            logger.warning(
                f"Requête trop volumineuse rejetée: {content_length} bytes "
                f"(limite: {settings.max_request_size})"
            )
            return Response(
                content="Request too large",
                status_code=413,
                headers=settings.get_security_headers(),
            )

        # Traitement de la requête
        start_time = time.time()
        response = await call_next(request)
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
                f"Time: {process_time:.3f}s"
            )

        return response


class RateLimitMiddleware(BaseHTTPMiddleware):
    """Middleware simple de rate limiting en mémoire."""

    def __init__(self, app, requests_per_minute: int = 100):
        super().__init__(app)
        self.requests_per_minute = requests_per_minute
        self.requests: dict[str, list[float]] = {}

    async def dispatch(self, request: Request, call_next: Callable) -> Response:
        """Applique le rate limiting basique."""
        if settings.is_production():
            client_ip = request.client.host if request.client else "unknown"
            now = time.time()

            # Nettoyage des anciennes requêtes
            if client_ip in self.requests:
                self.requests[client_ip] = [
                    req_time
                    for req_time in self.requests[client_ip]
                    if now - req_time < 60  # Garder seulement les dernières 60 secondes
                ]
            else:
                self.requests[client_ip] = []

            # Vérification de la limite
            if len(self.requests[client_ip]) >= self.requests_per_minute:
                logger.warning(f"Rate limit dépassé pour {client_ip}")
                return Response(
                    content="Rate limit exceeded",
                    status_code=429,
                    headers={"Retry-After": "60"},
                )

            # Ajout de la requête actuelle
            self.requests[client_ip].append(now)

        return await call_next(request)
