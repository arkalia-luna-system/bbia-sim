"""Gardes Hugging Face — space-id allowlist + opt-in pip install.

Extrait de ``daemon.app.hf_app_installer`` pour une surface testable / réutilisable
sans dépendre du clonage git. Défaut sûr : ``BBIA_HF_PIP_INSTALL`` OFF.
"""

from __future__ import annotations

import os
import re
from typing import Final

# username/space — refuse URL injectée, traversal, espaces, protocoles
HF_SPACE_ID_RE: Final[re.Pattern[str]] = re.compile(
    r"^[A-Za-z0-9._-]+/[A-Za-z0-9._-]+$"
)
HF_APP_NAME_RE: Final[re.Pattern[str]] = re.compile(r"^[A-Za-z0-9._-]+$")

HF_PIP_INSTALL_ENV: Final[str] = "BBIA_HF_PIP_INSTALL"
_TRUTHY: Final[frozenset[str]] = frozenset({"1", "true", "yes", "on"})


def is_valid_hf_space_id(hf_space_id: object) -> bool:
    """True si l'ID Space respecte ``user/space`` (caractères allowlistés)."""
    if not isinstance(hf_space_id, str):
        return False
    # Pas de strip silencieux : espaces / \\n / \\r = rejet (anti injection URL)
    return bool(HF_SPACE_ID_RE.fullmatch(hf_space_id))


def validate_hf_space_id(hf_space_id: object) -> str:
    """Valide et retourne l'ID Space, sinon ``ValueError``.

    Raises:
        ValueError: ID manquant, mal formé ou injecté.
    """
    if not is_valid_hf_space_id(hf_space_id):
        raise ValueError(f"HF Space ID invalide: {hf_space_id}")
    assert isinstance(hf_space_id, str)
    return hf_space_id


def normalize_hf_app_name(name: str) -> str:
    """Normalise et valide un nom d'app local (allowlist, pas de séparateurs).

    Raises:
        ValueError: nom vide ou hors allowlist.
    """
    normalized = (name or "").strip()
    if not normalized or not HF_APP_NAME_RE.fullmatch(normalized):
        raise ValueError(f"Nom d'application invalide: {name!r}")
    return normalized


def hf_pip_install_enabled(env: dict[str, str] | None = None) -> bool:
    """True uniquement si ``BBIA_HF_PIP_INSTALL`` est opt-in explicite."""
    source = env if env is not None else os.environ
    raw = source.get(HF_PIP_INSTALL_ENV, "0")
    return str(raw).strip().lower() in _TRUTHY
