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
    if not hmac.compare_digest(credentials.credentials, settings.api_token):
        logger.warning(
            "Tentative d'authentification avec token invalide: %s",
            settings.mask_token(credentials.credentials),
        )
        raise HTTPException(
            status_code=401,
            detail="Token d'authentification invalide",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if settings.log_level.upper() == "DEBUG":
        logger.debug(
            "Authentification réussie avec token: %s",
            settings.mask_token(credentials.credentials),
        )

    return str(credentials.credentials)
