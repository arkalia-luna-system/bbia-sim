#!/usr/bin/env python3
"""
Vérifie que RobotAPI.step() est non bloquant (pas de sleep long côté API abstraite).
"""

import time

import pytest

from bbia_sim.robot_factory import RobotFactory


@pytest.mark.unit
@pytest.mark.fast
def test_robot_api_step_non_blocking() -> None:
    robot = RobotFactory.create_backend("reachy_mini")
    assert robot is not None
    assert robot.connect() is True

    try:
        iterations = 100
        t0 = time.perf_counter()
        for _ in range(iterations):
            assert robot.step() is True
        elapsed = time.perf_counter() - t0

        # Seuil large pour CI: 100 steps doivent être rapides (< 0.5s)
        assert elapsed < 0.5, f"step() semble bloquant: {elapsed:.3f}s/100"
    finally:
        robot.disconnect()
