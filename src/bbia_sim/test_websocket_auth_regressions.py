"""Tests de non-regression auth WebSocket en production."""

from typing import Any, cast

import pytest
from fastapi import WebSocket

from bbia_sim.daemon.app.routers.apps import ws_apps_manager
from bbia_sim.daemon.app.routers.move import ws_move_updates, ws_set_target
from bbia_sim.daemon.app.routers.state import ws_full_state
from bbia_sim.daemon.config import settings
from bbia_sim.daemon.ws.auth import ws_auth_rejected, ws_token_is_valid
from bbia_sim.daemon.ws.telemetry import websocket_endpoint


class _DummyWebSocket:
    def __init__(self) -> None:
        self.accepted = False
        self.closed = False
        self.close_code: int | None = None
        self.close_reason: str | None = None

    async def accept(self) -> None:
        self.accepted = True

    async def close(self, code: int = 1000, reason: str | None = None) -> None:
        self.closed = True
        self.close_code = code
        self.close_reason = reason

    async def receive_text(self) -> str:
        return ""

    async def send_text(self, _text: str) -> None:
        return

    async def send_json(self, _payload: Any) -> None:
        return


def test_ws_token_compare_digest_rejects_wrong_and_none(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(settings, "api_token", "secret-token")
    assert ws_token_is_valid("secret-token") is True
    assert ws_token_is_valid("wrong-token!") is False
    assert ws_token_is_valid(None) is False
    assert ws_token_is_valid("") is False
    # Longueur différente (compare_digest → False, pas d'exception)
    assert ws_token_is_valid("x") is False


def test_ws_auth_rejected_only_in_prod(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(settings, "api_token", "secret")
    monkeypatch.setattr(settings, "environment", "dev")
    assert ws_auth_rejected(None) is False
    assert ws_auth_rejected("wrong") is False
    monkeypatch.setattr(settings, "environment", "prod")
    assert ws_auth_rejected(None) is True
    assert ws_auth_rejected("wrong") is True
    assert ws_auth_rejected("secret") is False


@pytest.mark.asyncio
async def test_state_ws_requires_token_in_prod(monkeypatch: pytest.MonkeyPatch) -> None:
    ws = _DummyWebSocket()
    monkeypatch.setattr(settings, "environment", "prod")
    monkeypatch.setattr(settings, "api_token", "secret")
    await ws_full_state(websocket=cast(WebSocket, ws), token=None)
    assert ws.closed is True
    assert ws.accepted is False
    assert ws.close_code == 1008


@pytest.mark.asyncio
async def test_state_ws_accepts_valid_token_in_prod(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Token valide : auth OK ; fermeture éventuelle pour autre raison (freq/backend)."""
    ws = _DummyWebSocket()
    monkeypatch.setattr(settings, "environment", "prod")
    monkeypatch.setattr(settings, "api_token", "secret")

    def _boom(*_a: Any, **_k: Any) -> Any:
        raise RuntimeError("stop-after-auth")

    monkeypatch.setattr(
        "bbia_sim.daemon.app.routers.state.ws_get_backend_adapter",
        _boom,
    )
    with pytest.raises(RuntimeError, match="stop-after-auth"):
        await ws_full_state(
            websocket=cast(WebSocket, ws),
            token="secret",  # nosec B106
            frequency=1.0,
        )
    assert ws.accepted is True
    assert ws.closed is False


@pytest.mark.asyncio
async def test_move_ws_updates_requires_token_in_prod(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ws = _DummyWebSocket()
    monkeypatch.setattr(settings, "environment", "prod")
    monkeypatch.setattr(settings, "api_token", "secret")
    await ws_move_updates(
        websocket=cast(WebSocket, ws),
        token="wrong",  # nosec B106
    )
    assert ws.closed is True
    assert ws.accepted is False
    assert ws.close_code == 1008


@pytest.mark.asyncio
async def test_move_ws_set_target_requires_token_in_prod(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ws = _DummyWebSocket()
    monkeypatch.setattr(settings, "environment", "prod")
    monkeypatch.setattr(settings, "api_token", "secret")
    await ws_set_target(websocket=cast(WebSocket, ws), token=None)
    assert ws.closed is True
    assert ws.accepted is False
    assert ws.close_code == 1008


@pytest.mark.asyncio
async def test_apps_ws_requires_token_in_prod(monkeypatch: pytest.MonkeyPatch) -> None:
    ws = _DummyWebSocket()
    monkeypatch.setattr(settings, "environment", "prod")
    monkeypatch.setattr(settings, "api_token", "secret")
    await ws_apps_manager(
        websocket=cast(WebSocket, ws),
        job_id="job-1",
        token="",  # nosec B106
    )
    assert ws.closed is True
    assert ws.accepted is False
    assert ws.close_code == 1008


@pytest.mark.asyncio
async def test_telemetry_ws_requires_token_in_prod(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ws = _DummyWebSocket()
    monkeypatch.setattr(settings, "environment", "prod")
    monkeypatch.setattr(settings, "api_token", "secret")
    await websocket_endpoint(
        websocket=cast(WebSocket, ws),
        token="invalid",  # nosec B106
    )
    assert ws.closed is True
    assert ws.accepted is False
    assert ws.close_code == 1008
