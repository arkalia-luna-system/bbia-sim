"""Auth HTTP Bearer — isolée de ``main`` pour tests sans import Jinja/FastAPI app."""

from __future__ import annotations

import hmac
import logging

from fastapi import Depends, HTTPException
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

from bbia_sim.daemon.config import settings

logger = logging.getLogger(__name__)
security = HTTPBearer()


def verify_token(credentials: HTTPAuthorizationCredentials = Depends(security)) -> str:
    """Vérifie le token d'authentification (temps constant)."""
    provided = credentials.credentials if credentials is not None else ""
    expected = settings.api_token if isinstance(settings.api_token, str) else ""

    # Token attendu vide / défaut en prod → refuse (évite accept "" == "")
    if settings.is_production() and settings.uses_insecure_api_token():
        logger.error("Auth refusée: token API insecure en production (défaut ou vide)")
        raise HTTPException(
            status_code=503,
            detail="Configuration API insecure en production",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if not provided or not expected:
        logger.warning(
            "Tentative d'authentification avec token vide: %s",
            settings.mask_token(provided or ""),
        )
        raise HTTPException(
            status_code=401,
            detail="Token d'authentification invalide",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if not hmac.compare_digest(provided, expected):
        logger.warning(
            "Tentative d'authentification avec token invalide: %s",
            settings.mask_token(provided),
        )
        raise HTTPException(
            status_code=401,
            detail="Token d'authentification invalide",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if settings.log_level.upper() == "DEBUG":
        logger.debug(
            "Authentification réussie avec token: %s",
            settings.mask_token(provided),
        )

    return str(provided)
