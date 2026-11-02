#!/usr/bin/env python3
"""Vertical slices mockés — remplacent les skips CI subprocess de test_vertical_slices.

Couvre emotion / voice / vision / behavior au niveau RobotAPI + factory, sans demos.
"""

from __future__ import annotations

from bbia_sim.robot_api import RobotAPI
from bbia_sim.robot_factory import RobotFactory


class _SliceRobot(RobotAPI):
    def __init__(self) -> None:
        super().__init__()
        self.is_connected = False
        self._joints = {
            "yaw_body": 0.0,
            "stewart_1": 0.0,
            "stewart_2": 0.0,
            "stewart_3": 0.0,
            "antenna_left": 0.0,
            "antenna_right": 0.0,
        }
        self.events: list[str] = []

    def connect(self) -> bool:
        self.is_connected = True
        self.events.append("connect")
        return True

    def disconnect(self) -> bool:
        self.is_connected = False
        self.events.append("disconnect")
        return True

    def get_available_joints(self) -> list[str]:
        return list(self._joints.keys())

    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        if joint_name not in self._joints:
            return False
        self._joints[joint_name] = float(position)
        self.events.append(f"set:{joint_name}")
        return True

    def get_joint_pos(self, joint_name: str) -> float | None:
        return self._joints.get(joint_name)

    def step(self) -> bool:
        self.events.append("step")
        return True

    def emergency_stop(self) -> bool:
        self.events.append("estop")
        return True


def test_slice_emotion_nod_headless_mock() -> None:
    robot = _SliceRobot()
    assert robot.connect() is True
    assert robot.run_behavior("nod", duration=0.05) is True
    assert "yaw_body" in robot.get_available_joints()
    assert robot.disconnect() is True


def test_slice_behavior_greeting_mock() -> None:
    robot = _SliceRobot()
    assert robot.connect() is True
    assert robot.run_behavior("greeting", duration=0.05) is True
    assert robot.emergency_stop() is True


def test_slice_vision_noop_joints_mock() -> None:
    """Vision slice sans ML : joints lisibles + step stable."""
    robot = _SliceRobot()
    assert robot.connect() is True
    before = robot.get_joint_pos("yaw_body")
    assert robot.step() is True
    assert robot.get_joint_pos("yaw_body") == before


def test_slice_voice_path_no_audio_mock(monkeypatch) -> None:
    """Voice slice : audio désactivé → pas d'appel media dangereux."""
    monkeypatch.setenv("BBIA_DISABLE_AUDIO", "1")
    from bbia_sim.media_guard import is_audio_disabled, resolve_media_backend

    assert is_audio_disabled() is True
    assert resolve_media_backend("default") == "no_media"
    assert resolve_media_backend("gstreamer") == "no_media"


def test_slice_backend_infos_available() -> None:
    for name in ("mujoco", "reachy", "reachy_mini"):
        info = RobotFactory.get_backend_info(name)
        assert isinstance(info, dict)
        assert "name" in info


def test_all_slices_smoke_mock() -> None:
    robot = _SliceRobot()
    assert robot.connect() is True
    for behavior in ("greeting", "nod", "wake_up"):
        assert robot.run_behavior(behavior, duration=0.02) is True
    assert robot.disconnect() is True
    assert "disconnect" in robot.events
