#!/usr/bin/env python3
"""Garde matériel Reachy Mini — pas de robot physique sans consentement explicite.

Le SDK officiel ``ReachyMini()`` se connecte à un daemon existant (réel ou sim)
dès l'instanciation ; ``use_sim`` ne s'applique vraiment qu'avec ``spawn_daemon``.
BBIA impose donc un flag d'environnement pour toute tentative hardware.

Variable requise pour le robot physique ::

    BBIA_ALLOW_HARDWARE=1

Sans ce flag, BBIA force le mode simulation / refuse la connexion matérielle.
"""

from __future__ import annotations

import logging
import os
from typing import Final

logger = logging.getLogger(__name__)

ALLOW_HARDWARE_ENV: Final[str] = "BBIA_ALLOW_HARDWARE"
_TRUTHY: Final[frozenset[str]] = frozenset({"1", "true", "yes", "on"})


def is_hardware_allowed() -> bool:
    """True uniquement si BBIA_ALLOW_HARDWARE est explicitement activé."""
    return os.environ.get(ALLOW_HARDWARE_ENV, "").strip().lower() in _TRUTHY


def resolve_use_sim(requested_use_sim: bool, *, context: str) -> bool:
    """Retourne le ``use_sim`` effectif (True forcé si hardware non autorisé).

    Args:
        requested_use_sim: Valeur demandée par l'appelant.
        context: Libellé court pour les logs (ex. ``ReachyMiniBackend.connect``).

    Returns:
        True si simulation, False si robot physique autorisé.
    """
    if requested_use_sim:
        return True
    if is_hardware_allowed():
        logger.info(
            "Hardware autorisé (%s=1) — mode robot physique (%s)",
            ALLOW_HARDWARE_ENV,
            context,
        )
        return False
    logger.warning(
        "%s non défini/activé → use_sim forcé à True (%s). "
        "Pour le robot physique: export %s=1 et relire "
        "docs/hardware/PREMIER_JOUR_ROBOT.md",
        ALLOW_HARDWARE_ENV,
        context,
        ALLOW_HARDWARE_ENV,
    )
    return True


def require_hardware_allowed(*, action: str = "connexion robot physique") -> None:
    """Lève PermissionError si le hardware n'est pas explicitement autorisé."""
    if is_hardware_allowed():
        return
    raise PermissionError(
        f"{action} refusée: définir {ALLOW_HARDWARE_ENV}=1 "
        "(voir docs/hardware/PREMIER_JOUR_ROBOT.md). "
        "Aucun mouvement matériel sans ce flag."
    )
