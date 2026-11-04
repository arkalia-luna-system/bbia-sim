#!/usr/bin/env python3
"""Garde pitch/roll tête — anticipation Pollen #1449.

La doc Pollen promet un clamp ±40° pitch/roll ; le SDK 1.13 ne l'applique pas
(IK accepte ~50° ou ignore silencieusement). BBIA clamp côté app avant
``create_head_pose`` / cibles euler.

Variables::

    BBIA_HEAD_PITCH_ROLL_LIMIT_DEG=40   # défaut = enveloppe documentée Pollen
    BBIA_HEAD_PITCH_ROLL_LIMIT_DEG=0    # désactive le clamp soft
"""

from __future__ import annotations

import logging
import math
import os
from typing import Final

logger = logging.getLogger(__name__)

HEAD_PITCH_ROLL_LIMIT_DEG_ENV: Final[str] = "BBIA_HEAD_PITCH_ROLL_LIMIT_DEG"
_DEFAULT_LIMIT_DEG: Final[float] = 40.0


def get_head_pitch_roll_limit_rad() -> float | None:
    """Limite soft pitch/roll (rad). ``None`` = clamp désactivé."""
    raw = os.environ.get(HEAD_PITCH_ROLL_LIMIT_DEG_ENV, "").strip()
    if raw == "":
        deg = _DEFAULT_LIMIT_DEG
    else:
        try:
            deg = float(raw)
        except ValueError:
            logger.warning(
                "%s invalide (%r) → %.1f°",
                HEAD_PITCH_ROLL_LIMIT_DEG_ENV,
                raw,
                _DEFAULT_LIMIT_DEG,
            )
            deg = _DEFAULT_LIMIT_DEG
    if deg <= 0.0:
        return None
    # Plafond défensif 60° : au-delà, inutile (IK NaN / stall Pollen)
    deg = min(deg, 60.0)
    return math.radians(deg)


def clamp_head_pitch_roll(
    pitch: float,
    roll: float,
    *,
    degrees: bool = False,
    limit_rad: float | None = None,
) -> tuple[float, float]:
    """Clamp pitch/roll à l'enveloppe documentée (±40° par défaut).

    ``yaw`` n'est pas touché ici (body/relative yaw = autres gardes / SDK).
    """
    lim = get_head_pitch_roll_limit_rad() if limit_rad is None else limit_rad
    if lim is None:
        return float(pitch), float(roll)

    p = float(pitch)
    r = float(roll)
    if degrees:
        lim_deg = math.degrees(lim)
        cp = max(-lim_deg, min(lim_deg, p))
        cr = max(-lim_deg, min(lim_deg, r))
    else:
        cp = max(-lim, min(lim, p))
        cr = max(-lim, min(lim, r))
    if cp != p or cr != r:
        logger.debug(
            "head pitch/roll clampés (#1449): pitch %s→%s roll %s→%s "
            "(degrees=%s, lim_rad=%.4f)",
            p,
            cp,
            r,
            cr,
            degrees,
            lim,
        )
    return cp, cr
