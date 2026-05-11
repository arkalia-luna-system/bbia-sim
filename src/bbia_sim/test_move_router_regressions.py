"""Tests de non-regression pour le routeur move."""

import asyncio

import pytest
from fastapi import HTTPException
from pydantic import ValidationError

from bbia_sim.daemon.app.routers.move import (
    GotoModelRequest,
    batch_movements,
    create_move_task,
    move_listeners,
    stop_move_task,
)
from bbia_sim.daemon.models import BatchMovementRequest


class _DummyBackend:
    async def goto_target(self, head=None, antennas=None, duration=1.0):
        return None

    async def wake_up(self):
        return None

    async def goto_sleep(self):
        return None

    def set_target(self, head=None, antennas=None):
        return None


class _DummyWS:
    def __init__(self):
        self.messages = []

    async def send_json(self, payload):
        self.messages.append(payload)


@pytest.mark.asyncio
async def test_create_move_task_notifies_cancelled():
    ws = _DummyWS()
    move_listeners.append(ws)

    async def slow_move():
        await asyncio.sleep(1)

    move_uuid = create_move_task(slow_move()).uuid
    await stop_move_task(move_uuid)
    await asyncio.sleep(0)

    assert any(m["type"] == "move_cancelled" for m in ws.messages)
    if ws in move_listeners:
        move_listeners.remove(ws)


@pytest.mark.asyncio
async def test_batch_movements_rejects_unknown_type():
    req = BatchMovementRequest(movements=[{"type": "invalid"}])
    with pytest.raises(HTTPException) as exc:
        await batch_movements(req, _DummyBackend())
    assert exc.value.status_code == 422


@pytest.mark.asyncio
async def test_batch_movements_rejects_non_positive_goto_duration():
    req = BatchMovementRequest(movements=[{"type": "goto", "duration": 0}])
    with pytest.raises(HTTPException) as exc:
        await batch_movements(req, _DummyBackend())
    assert exc.value.status_code == 422


def test_goto_model_request_validates_positive_duration():
    with pytest.raises(ValidationError):
        GotoModelRequest(duration=0)
