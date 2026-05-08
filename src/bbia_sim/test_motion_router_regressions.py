"""Tests de non-regression pour le routeur motion."""

from typing import Any

import pytest
from fastapi import HTTPException

from bbia_sim.daemon.app.routers.motion import EmotionRequest, goto_pose, set_emotion


class _DummyRobot:
    def __init__(self, fail_on_goto: bool = False, set_emotion_result: bool = True) -> None:
        self.fail_on_goto = fail_on_goto
        self.set_emotion_result = set_emotion_result
        self.connect_calls = 0
        self.disconnect_calls = 0

    def connect(self) -> bool:
        self.connect_calls += 1
        return True

    def disconnect(self) -> bool:
        self.disconnect_calls += 1
        return True

    def goto_target(self, **_: Any) -> None:
        if self.fail_on_goto:
            raise RuntimeError("goto failed")

    def set_emotion(self, _emotion: str, _intensity: float) -> bool:
        return self.set_emotion_result


@pytest.mark.asyncio
async def test_goto_pose_disconnects_robot_even_on_failure(monkeypatch: pytest.MonkeyPatch):
    robot = _DummyRobot(fail_on_goto=True)

    class _Factory:
        @staticmethod
        def create_backend(_backend: str):
            return robot

    monkeypatch.setattr("bbia_sim.robot_factory.RobotFactory", _Factory)
    pose_payload = {"x": 0.1, "y": 0.0, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    from bbia_sim.daemon.models import Pose

    result = await goto_pose(Pose(**pose_payload))
    assert result["status"] == "error"
    assert robot.connect_calls == 1
    assert robot.disconnect_calls == 1


@pytest.mark.asyncio
async def test_set_emotion_rejects_invalid_intensity():
    with pytest.raises(HTTPException) as exc:
        await set_emotion(EmotionRequest(emotion="joy", intensity=1.2))
    assert exc.value.status_code == 422
