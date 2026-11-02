"""Comparaison de tokens API — partagée HTTP Bearer / WebSocket.

Évite le doublon ``hmac.compare_digest`` + garde prod insecure entre
``daemon.app.http_auth`` et ``daemon.ws.auth``.
"""

from __future__ import annotations

import hmac
from typing import Final

from bbia_sim.daemon.config import settings

_EMPTY: Final[str] = ""


def expected_api_token() -> str:
    """Token attendu depuis Settings (chaîne vide si type inattendu)."""
    token = settings.api_token
    return token if isinstance(token, str) else _EMPTY


def production_rejects_insecure_token() -> bool:
    """True si prod + token défaut/vide (auth doit refuser toute requête)."""
    return bool(settings.is_production() and settings.uses_insecure_api_token())


def api_token_matches(provided: str | None) -> bool:
    """True si ``provided`` match le token attendu (temps constant).

    Refuse toujours si prod insecure, ou si l'un des côtés est vide.
    """
    if production_rejects_insecure_token():
        return False
    if not provided or not isinstance(provided, str):
        return False
    expected = expected_api_token()
    if not expected:
        return False
    return hmac.compare_digest(provided, expected)
