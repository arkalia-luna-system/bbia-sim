"""Tests de non-regression pour BackendAdapter."""

import pytest
from fastapi import HTTPException

from bbia_sim.daemon.app.backend_adapter import BackendAdapter


class _DummyRobot:
    def __init__(self, connect_result=True, raise_on_connect: Exception | None = None):
        self.connect_result = connect_result
        self.raise_on_connect = raise_on_connect
        self.connect_calls = 0
        self.disconnect_calls = 0

    def connect(self):
        self.connect_calls += 1
        if self.raise_on_connect:
            raise self.raise_on_connect
        return self.connect_result

    def disconnect(self):
        self.disconnect_calls += 1
        return True

    def get_available_joints(self):
        return []


def test_connect_if_needed_raises_when_connect_returns_false():
    adapter = BackendAdapter(robot=_DummyRobot(connect_result=False))
    with pytest.raises(HTTPException) as exc:
        adapter.connect_if_needed()
    assert exc.value.status_code == 503


def test_connect_if_needed_raises_when_connect_throws():
    adapter = BackendAdapter(robot=_DummyRobot(raise_on_connect=RuntimeError("boom")))
    with pytest.raises(HTTPException) as exc:
        adapter.connect_if_needed()
    assert exc.value.status_code == 503


def test_connect_if_needed_connects_only_once():
    robot = _DummyRobot(connect_result=True)
    adapter = BackendAdapter(robot=robot)
    adapter.connect_if_needed()
    adapter.connect_if_needed()
    assert robot.connect_calls == 1


def test_close_resets_connection_flag_even_after_connect():
    robot = _DummyRobot(connect_result=True)
    adapter = BackendAdapter(robot=robot)
    adapter.connect_if_needed()
    adapter.close()
    assert robot.disconnect_calls == 1
    assert adapter._connected is False
