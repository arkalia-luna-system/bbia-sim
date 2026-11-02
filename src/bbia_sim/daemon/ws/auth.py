"""Auth WebSocket (query param ``token``) — alignée sur HTTP via ``auth_token``.

En production, le token est requis. En dev, l'auth reste optionnelle pour le DX.
"""

from __future__ import annotations

from bbia_sim.daemon.auth_token import api_token_matches
from bbia_sim.daemon.config import settings


def ws_token_is_valid(token: str | None) -> bool:
    """Vérifie le token WS en temps constant (anti timing attack)."""
    return api_token_matches(token)


def ws_auth_rejected(token: str | None) -> bool:
    """True si la connexion WS doit être refusée (prod sans token valide)."""
    if not settings.is_production():
        return False
    return not ws_token_is_valid(token)
