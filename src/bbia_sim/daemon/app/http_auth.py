"""Auth HTTP Bearer — isolée de ``main`` pour tests sans import Jinja/FastAPI app."""

from __future__ import annotations

import logging

from fastapi import Depends, HTTPException
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

from bbia_sim.daemon.auth_token import (
    api_token_matches,
    production_rejects_insecure_token,
)
from bbia_sim.daemon.config import settings

logger = logging.getLogger(__name__)
# auto_error=False : sans header Authorization, on passe par verify_token → 401
# (Starlette<1.10 renvoyait 403 via HTTPBearer ; 1.10+ renvoie 401 — unifier).
security = HTTPBearer(auto_error=False)


def verify_token(
    credentials: HTTPAuthorizationCredentials | None = Depends(security),
) -> str:
    """Vérifie le token d'authentification (temps constant)."""
    provided = (
        credentials.credentials
        if credentials is not None and isinstance(credentials.credentials, str)
        else ""
    )

    # Token attendu vide / défaut en prod → refuse (évite accept "" == "")
    if production_rejects_insecure_token():
        logger.error("Auth refusée: token API insecure en production (défaut ou vide)")
        raise HTTPException(
            status_code=503,
            detail="Configuration API insecure en production",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if not provided:
        logger.warning("Tentative d'authentification avec token vide")
        raise HTTPException(
            status_code=401,
            detail="Token d'authentification invalide",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if not api_token_matches(provided):
        # Pas de fragment de token dans les logs (même masqué) hors DEBUG
        logger.warning("Tentative d'authentification avec token invalide")
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
