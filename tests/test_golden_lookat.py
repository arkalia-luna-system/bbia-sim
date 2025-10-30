#!/usr/bin/env python3
"""Golden test pour look_at_world: vérifie la pose retournée/estimée.

Ce test utilise la simulation (use_sim=True) et ne dépend pas du SDK physique.
"""

import math

import numpy as np
import pytest

from src.bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

REF_TARGET = (0.5, 0.2, 0.3)
TOL_ANGLE = 0.25  # radians


def _extract_pitch_yaw(pose: np.ndarray) -> tuple[float, float]:
    # Approximation cohérente avec l'implémentation look_at_world interne
    pitch = math.atan2(-pose[2, 0], pose[2, 2])
    yaw = math.atan2(pose[1, 0], pose[0, 0])
    return float(pitch), float(yaw)


@pytest.mark.unit
def test_golden_look_at_world_pose() -> None:
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True
    x, y, z = REF_TARGET
    pose = backend.look_at_world(x, y, z, duration=0.5, perform_movement=True)
    assert pose is not None
    assert isinstance(pose, np.ndarray) and pose.shape == (4, 4)

    # Vérification angles (approx)
    pitch, yaw = _extract_pitch_yaw(pose)
    dist = math.sqrt(x**2 + y**2 + z**2)
    pitch_ref = 0.0 if dist == 0 else -math.asin(z / dist)
    yaw_ref = 0.0 if dist == 0 else math.atan2(y, x)
    assert abs(pitch - pitch_ref) <= TOL_ANGLE
    assert abs(yaw - yaw_ref) <= TOL_ANGLE

    backend.disconnect()
