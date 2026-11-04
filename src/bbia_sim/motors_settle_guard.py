#!/usr/bin/env python3
"""Settle soft après enable_motors — anticipation Pollen #1430.

Streamer ``set_target`` immédiatement après ``enable_motors`` depuis sleep
peut faire tomber le daemon Pollen. BBIA enregistre le timestamp enable et
attend un délai soft configurable avant ``set_target`` / ``set_target_head_pose``.

Variables::

    BBIA_MOTORS_SETTLE_S=0.35   # défaut ; 0 = désactive l'attente
"""

from __future__ import annotations

import logging
import os
import time
from typing import Final

logger = logging.getLogger(__name__)

MOTORS_SETTLE_ENV: Final[str] = "BBIA_MOTORS_SETTLE_S"
_DEFAULT_SETTLE_S: Final[float] = 0.35

# Timestamp monotonique du dernier enable_motors (process-local)
_last_enable_mono: float | None = None


def get_motors_settle_seconds() -> float:
    """Délai soft (s) après enable avant streaming set_target. 0 = off."""
    raw = os.environ.get(MOTORS_SETTLE_ENV, "").strip()
    if raw == "":
        return _DEFAULT_SETTLE_S
    try:
        value = float(raw)
    except ValueError:
        logger.warning(
            "%s invalide (%r) → %.2f",
            MOTORS_SETTLE_ENV,
            raw,
            _DEFAULT_SETTLE_S,
        )
        return _DEFAULT_SETTLE_S
    return max(0.0, min(value, 5.0))


def mark_motors_enabled() -> None:
    """À appeler juste après un ``enable_motors`` réussi (ou sim)."""
    global _last_enable_mono
    _last_enable_mono = time.monotonic()


def reset_motors_settle_state() -> None:
    """Reset (tests)."""
    global _last_enable_mono
    _last_enable_mono = None


def await_motors_settle_if_needed(*, context: str = "set_target") -> float:
    """Bloque le reste du délai settle si enable récent.

    Returns:
        Secondes effectivement attendues (0.0 si pas d'attente).
    """
    settle = get_motors_settle_seconds()
    if settle <= 0.0 or _last_enable_mono is None:
        return 0.0
    elapsed = time.monotonic() - _last_enable_mono
    remaining = settle - elapsed
    if remaining <= 0.0:
        return 0.0
    logger.info(
        "%s: settle soft #1430 — attente %.3fs après enable_motors "
        "(export %s=0 pour désactiver)",
        context,
        remaining,
        MOTORS_SETTLE_ENV,
    )
    time.sleep(remaining)
    return remaining
