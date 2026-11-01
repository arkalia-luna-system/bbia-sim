#!/usr/bin/env python3
"""Mitigations soft sleep / yaw — anticipation Pollen #1478 / #1479.

Pas de patch firmware. BBIA peut :
- re-vérifier la pose avant ``goto_sleep``
- logger un warning droop connu
- optionnellement rappeler ``enable_motors`` si ``BBIA_SLEEP_HOLD_TORQUE=1``
  **uniquement** si le SDK expose encore les moteurs (honnêteté : le droop
  après coupe couple reste un comportement firmware Pollen).

Variables::

    BBIA_SLEEP_HOLD_TORQUE=0   # défaut : pas de re-torque agressif
    BBIA_SLEEP_HOLD_TORQUE=1   # tente enable_motors avant sleep si dispo
    BBIA_YAW_SAFE_AMPLITUDE=0.3  # clamp soft yaw_body (rad)
"""

from __future__ import annotations

import logging
import os
from typing import Any, Final

logger = logging.getLogger(__name__)

SLEEP_HOLD_TORQUE_ENV: Final[str] = "BBIA_SLEEP_HOLD_TORQUE"
YAW_SAFE_AMPLITUDE_ENV: Final[str] = "BBIA_YAW_SAFE_AMPLITUDE"
_TRUTHY: Final[frozenset[str]] = frozenset({"1", "true", "yes", "on"})
_DEFAULT_YAW_SAFE: Final[float] = 0.3


def sleep_hold_torque_requested() -> bool:
    """True si l'opérateur demande un hold couple soft avant sleep."""
    return os.environ.get(SLEEP_HOLD_TORQUE_ENV, "0").strip().lower() in _TRUTHY


def get_yaw_safe_amplitude(fallback: float = _DEFAULT_YAW_SAFE) -> float:
    """Amplitude soft yaw_body (rad) — mitigation oscillation #1479."""
    raw = os.environ.get(YAW_SAFE_AMPLITUDE_ENV, "").strip()
    if not raw:
        return max(0.05, min(float(fallback), 0.5))
    try:
        value = float(raw)
    except ValueError:
        logger.warning("%s invalide (%r) → %s", YAW_SAFE_AMPLITUDE_ENV, raw, fallback)
        return max(0.05, min(float(fallback), 0.5))
    return max(0.05, min(value, 0.5))


def clamp_yaw_amplitude(position: float, *, limit: float | None = None) -> float:
    """Clamp yaw pour limiter les oscillations agressives côté BBIA."""
    lim = get_yaw_safe_amplitude() if limit is None else float(limit)
    clamped = max(-lim, min(lim, float(position)))
    if clamped != position:
        logger.debug(
            "yaw clampé #1479: %.4f → %.4f (limite soft ±%.3f rad)",
            position,
            clamped,
            lim,
        )
    return clamped


def prepare_goto_sleep(robot: Any | None, *, use_sim: bool) -> dict[str, Any]:
    """Garde explicite avant sleep : re-check pose + option hold torque.

    Returns:
        Dict diagnostic (pose_ok, hold_torque_applied, warnings).
    """
    report: dict[str, Any] = {
        "pose_ok": True,
        "hold_torque_applied": False,
        "warnings": [],
        "sim": bool(use_sim),
    }
    if use_sim or robot is None:
        report["warnings"].append("simulation_or_no_robot")
        logger.debug("goto_sleep: sim/no robot — skip garde hardware")
        return report

    # Re-check pose (best-effort)
    try:
        if hasattr(robot, "get_current_head_pose"):
            pose = robot.get_current_head_pose()
            report["head_pose_present"] = pose is not None
        elif hasattr(robot, "get_current_joint_positions"):
            joints = robot.get_current_joint_positions()
            report["joints_present"] = joints is not None
        else:
            report["warnings"].append("no_pose_api")
    except (AttributeError, RuntimeError, OSError, ConnectionError, TypeError) as exc:
        report["pose_ok"] = False
        report["warnings"].append(f"pose_check_failed:{exc}")
        logger.warning(
            "goto_sleep: re-check pose échoué (%s) — poursuite prudente "
            "(droop #1478 = exposé firmware Pollen)",
            exc,
        )

    logger.warning(
        "goto_sleep: le droop tête après coupe couple (Pollen #1478) reste "
        "exposé firmware — BBIA ne re-torque pas agressivement par défaut"
    )

    if sleep_hold_torque_requested():
        try:
            if hasattr(robot, "enable_motors"):
                try:
                    robot.enable_motors()
                except TypeError:
                    robot.enable_motors()  # type: ignore[call-arg]
                report["hold_torque_applied"] = True
                report["warnings"].append("hold_torque_best_effort_not_a_firmware_fix")
                logger.info(
                    "BBIA_SLEEP_HOLD_TORQUE=1 : enable_motors() best-effort "
                    "avant sleep (ne garantit pas l'absence de droop)"
                )
            else:
                report["warnings"].append("enable_motors_absent")
                logger.warning(
                    "BBIA_SLEEP_HOLD_TORQUE=1 mais enable_motors absent — "
                    "exposé Pollen uniquement"
                )
        except (
            AttributeError,
            RuntimeError,
            OSError,
            ConnectionError,
            TypeError,
        ) as exc:
            report["warnings"].append(f"hold_torque_failed:{exc}")
            logger.warning("hold torque soft échoué: %s", exc)

    return report
