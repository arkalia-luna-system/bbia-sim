"""Auth WebSocket (query param ``token``) — alignée sur HTTP ``hmac.compare_digest``.

En production, le token est requis. En dev, l'auth reste optionnelle pour le DX.
"""

from __future__ import annotations

import hmac

from bbia_sim.daemon.config import settings


def ws_token_is_valid(token: str | None) -> bool:
    """Vérifie le token WS en temps constant (anti timing attack)."""
    if not token or not isinstance(token, str):
        return False
    expected = settings.api_token
    if not isinstance(expected, str) or not expected:
        return False
    # En prod, le défaut connu n'est jamais « valide » même s'il match
    if settings.is_production() and settings.uses_insecure_api_token():
        return False
    return hmac.compare_digest(token, expected)


def ws_auth_rejected(token: str | None) -> bool:
    """True si la connexion WS doit être refusée (prod sans token valide)."""
    if not settings.is_production():
        return False
    return not ws_token_is_valid(token)
