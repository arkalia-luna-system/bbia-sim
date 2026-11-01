#!/usr/bin/env python3
"""Gardes head tracking / vision — anticipation Pollen #1453 / #1451 / #1450.

En simulation / mock, les loops tracking daemon Pollen peuvent être absentes
(freeze / intrinsics). BBIA skip en no-op sauf opt-in explicite.

Variables::

    BBIA_HEAD_TRACKING_SIM=0   # défaut sûr : pas de tracking en sim
    BBIA_HEAD_TRACKING_SIM=1   # autorise appels tracking même en use_sim
"""

from __future__ import annotations

import logging
import math
import os
from typing import Final

logger = logging.getLogger(__name__)

HEAD_TRACKING_SIM_ENV: Final[str] = "BBIA_HEAD_TRACKING_SIM"
_TRUTHY: Final[frozenset[str]] = frozenset({"1", "true", "yes", "on"})

# Bornes direction look-at (rad / normalised image) — clamp défensif
MAX_LOOK_DIRECTION_RAD: Final[float] = 0.35
MAX_LOOK_IMAGE_U: Final[int] = 640
MAX_LOOK_IMAGE_V: Final[int] = 480


def is_head_tracking_sim_enabled() -> bool:
    """True seulement si BBIA_HEAD_TRACKING_SIM est explicitement activé."""
    return os.environ.get(HEAD_TRACKING_SIM_ENV, "0").strip().lower() in _TRUTHY


def should_skip_head_tracking(*, use_sim: bool, context: str = "head_tracking") -> bool:
    """True → no-op sûr (pas d'appel loop absente).

    Skip si ``use_sim`` et que le flag sim n'est pas activé.
    """
    if not use_sim:
        return False
    if is_head_tracking_sim_enabled():
        logger.debug(
            "%s: sim autorisée (%s=1)",
            context,
            HEAD_TRACKING_SIM_ENV,
        )
        return False
    logger.info(
        "%s: skip en sim/mock (défaut sûr). "
        "Pour forcer: export %s=1 — issues Pollen #1453/#1451/#1450",
        context,
        HEAD_TRACKING_SIM_ENV,
    )
    return True


def clamp_look_direction_rad(yaw: float, pitch: float) -> tuple[float, float]:
    """Clamp yaw/pitch (rad) pour éviter directions aberrantes (calib caméra)."""
    lim = MAX_LOOK_DIRECTION_RAD
    cy = max(-lim, min(lim, float(yaw)))
    cp = max(-lim, min(lim, float(pitch)))
    if cy != yaw or cp != pitch:
        logger.debug(
            "look direction clampée: yaw %.3f→%.3f pitch %.3f→%.3f",
            yaw,
            cy,
            pitch,
            cp,
        )
    return cy, cp


def clamp_look_image_uv(u: float, v: float) -> tuple[int, int]:
    """Clamp coordonnées image pour look_at_image."""
    cu = int(max(0, min(MAX_LOOK_IMAGE_U, int(u))))
    cv = int(max(0, min(MAX_LOOK_IMAGE_V, int(v))))
    return cu, cv


def assert_finite_direction(*values: float, context: str = "direction") -> None:
    """Refuse NaN/Inf sur une direction de tracking."""
    for value in values:
        if not math.isfinite(float(value)):
            raise ValueError(f"{context}: valeur non finie {value!r}")
