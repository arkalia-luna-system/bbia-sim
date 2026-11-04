#!/usr/bin/env python3
"""Garde ``initial_goto_duration`` — anticipation Pollen #1417.

Enchaîner un move tête basse (ex. ``waiting``) vers un autre (ex. ``yes1``)
avec ``initial_goto_duration=0`` peut laisser la tête inerte jusqu'à
``disable_motors`` / ``enable_motors``. Workaround Pollen documenté :
``initial_goto_duration >= 1.0``.

BBIA applique un défaut soft configurable (pas un correctif firmware).

Variables::

    BBIA_PLAY_MOVE_INITIAL_GOTO_S=1.0  # défaut sûr ; 0 = jump SDK instantané
"""

from __future__ import annotations

import logging
import os
from typing import Final

logger = logging.getLogger(__name__)

PLAY_MOVE_INITIAL_GOTO_ENV: Final[str] = "BBIA_PLAY_MOVE_INITIAL_GOTO_S"
_DEFAULT_INITIAL_GOTO_S: Final[float] = 1.0
_MAX_INITIAL_GOTO_S: Final[float] = 30.0


def get_play_move_initial_goto_seconds(override: float | None = None) -> float:
    """Résout la durée goto initiale avant lecture d'un move enregistré.

    Args:
        override: Si fourni (y compris ``0.0``), valeur explicite du caller.
            Si ``None``, lit ``BBIA_PLAY_MOVE_INITIAL_GOTO_S`` (défaut 1.0).

    Returns:
        Secondes clampées dans ``[0, 30]``.
    """
    if override is not None:
        try:
            value = float(override)
        except (TypeError, ValueError):
            logger.warning(
                "initial_goto_duration invalide (%r) → défaut %.1f (#1417)",
                override,
                _DEFAULT_INITIAL_GOTO_S,
            )
            return _DEFAULT_INITIAL_GOTO_S
        return max(0.0, min(value, _MAX_INITIAL_GOTO_S))

    raw = os.environ.get(PLAY_MOVE_INITIAL_GOTO_ENV, "").strip()
    if raw == "":
        return _DEFAULT_INITIAL_GOTO_S
    try:
        value = float(raw)
    except ValueError:
        logger.warning(
            "%s invalide (%r) → défaut %.1f (#1417)",
            PLAY_MOVE_INITIAL_GOTO_ENV,
            raw,
            _DEFAULT_INITIAL_GOTO_S,
        )
        return _DEFAULT_INITIAL_GOTO_S
    return max(0.0, min(value, _MAX_INITIAL_GOTO_S))
