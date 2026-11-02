#!/usr/bin/env python3
"""Smoke RobotAPI mockés — remplacent les skips CI subprocess de test_robot_api_smoke.

Pas de hardware / e2e / ML. Exécutables en CI.
"""

from __future__ import annotations

import pytest

from bbia_sim.robot_api import RobotAPI
from bbia_sim.robot_factory import RobotFactory


class _SmokeRobot(RobotAPI):
    def __init__(self, n_joints: int = 7) -> None:
        super().__init__()
        self.is_connected = False
        self._joints = {f"j{i}": 0.0 for i in range(n_joints)}
        if n_joints >= 1:
            self._joints = {
                "yaw_body": 0.0,
                **{f"joint_{i}": 0.0 for i in range(max(0, n_joints - 1))},
            }

    def connect(self) -> bool:
        self.is_connected = True
        return True

    def disconnect(self) -> bool:
        self.is_connected = False
        return True

    def get_available_joints(self) -> list[str]:
        return list(self._joints.keys())

    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        if joint_name not in self._joints:
            return False
        self._joints[joint_name] = position
        return True

    def get_joint_pos(self, joint_name: str) -> float | None:
        return self._joints.get(joint_name)

    def step(self) -> bool:
        return True

    def emergency_stop(self) -> bool:
        return True


def test_smoke_emotion_like_behavior_mock() -> None:
    robot = _SmokeRobot(n_joints=7)
    assert robot.connect() is True
    assert robot.run_behavior("greeting", duration=0.05) is True
    assert len(robot.get_available_joints()) == 7
    assert robot.disconnect() is True


def test_smoke_backend_info_switching_mock() -> None:
    mujoco = RobotFactory.get_backend_info("mujoco")
    reachy = RobotFactory.get_backend_info("reachy")
    assert "name" in mujoco and "name" in reachy
    assert mujoco["name"] != reachy["name"]


def test_normalize_backend_aliases_smoke() -> None:
    assert RobotFactory._normalize_backend_type("MuJoCo") == "mujoco"
    assert RobotFactory._normalize_backend_type("reachy_mini") == "reachy_mini"


def test_smoke_invalid_joint_rejected_mock() -> None:
    robot = _SmokeRobot(n_joints=7)
    assert robot.connect() is True
    assert robot.set_joint_pos("invalid_joint", 0.1) is False
    assert robot.get_joint_pos("invalid_joint") is None


def test_smoke_factory_unknown_backend_returns_none(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Équivalent mock du smoke subprocess ``--backend invalid_backend``."""
    monkeypatch.delenv("BBIA_ALLOW_HARDWARE", raising=False)
    assert RobotFactory.create_backend("invalid_backend") is None


def test_smoke_factory_auto_without_hardware_uses_mujoco(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("BBIA_ALLOW_HARDWARE", raising=False)
    backend = RobotFactory.create_backend("auto")
    assert backend is not None
    info = RobotFactory.get_backend_info("mujoco")
    assert info["name"]


def test_smoke_performance_path_mock() -> None:
    """Remplace le smoke performance subprocess (pose joints + step)."""
    robot = _SmokeRobot(n_joints=7)
    assert robot.connect() is True
    for name in robot.get_available_joints():
        assert robot.set_joint_pos(name, 0.05) is True
        assert robot.step() is True
    assert robot.emergency_stop() is True
    assert robot.disconnect() is True
