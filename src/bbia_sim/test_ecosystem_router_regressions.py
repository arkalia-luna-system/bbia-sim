"""Tests de non-regression pour le routeur ecosystem."""

from typing import Any

import pytest

from bbia_sim.daemon.app.routers import ecosystem


class _DummyRobot:
    def __init__(self, fail_on_set: bool = False) -> None:
        self.fail_on_set = fail_on_set
        self.connect_calls = 0
        self.disconnect_calls = 0

    def connect(self) -> bool:
        self.connect_calls += 1
        return True

    def disconnect(self) -> bool:
        self.disconnect_calls += 1
        return True

    def set_emotion(self, _emotion: str, _intensity: float) -> None:
        if self.fail_on_set:
            raise RuntimeError("set_emotion failed")


class _DummyEmotions:
    def __init__(self) -> None:
        self.emotions: dict[str, Any] = {"happy": {}}


@pytest.mark.asyncio
async def test_start_demo_mode_disconnects_robot_even_when_emotion_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    robot = _DummyRobot(fail_on_set=True)
    monkeypatch.setattr(ecosystem, "BBIAEmotions", _DummyEmotions)
    monkeypatch.setattr(
        ecosystem.RobotFactory, "create_backend", lambda *_args, **_kwargs: robot
    )

    result = await ecosystem.start_demo_mode(
        mode="robot_real", duration=0.01, emotion="happy"
    )
    assert result["status"] == "started"
    assert "emotion_error" in result
    assert robot.connect_calls == 1
    assert robot.disconnect_calls == 1


@pytest.mark.asyncio
async def test_start_demo_mode_applies_trimmed_emotion(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    robot = _DummyRobot(fail_on_set=False)
    monkeypatch.setattr(ecosystem, "BBIAEmotions", _DummyEmotions)
    monkeypatch.setattr(
        ecosystem.RobotFactory, "create_backend", lambda *_args, **_kwargs: robot
    )

    result = await ecosystem.start_demo_mode(
        mode="robot_real", duration=0.01, emotion="  HAPPY "
    )
    assert result.get("emotion_applied") == "happy"
