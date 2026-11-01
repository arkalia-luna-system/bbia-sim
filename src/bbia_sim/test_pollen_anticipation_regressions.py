"""Régressions anticipations issues/GHSA Pollen (suite courte)."""

from __future__ import annotations

import asyncio
from typing import cast

import pytest
from fastapi import WebSocket

from bbia_sim.daemon.app import safe_media_upload
from bbia_sim.daemon.config import Settings, settings
from bbia_sim.daemon.ws.keepalive import (
    get_ws_keepalive_timeout,
    receive_text_with_keepalive,
    uvicorn_ws_keepalive_kwargs,
)


def test_ws_keepalive_timeout_configurable(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pollen #1494 : timeout WS configurable via BBIA_WS_KEEPALIVE_TIMEOUT."""
    monkeypatch.setenv("BBIA_WS_KEEPALIVE_TIMEOUT", "45")
    fresh = Settings()
    assert fresh.ws_keepalive_timeout == 45.0


def test_uvicorn_ws_kwargs_wired_to_settings(monkeypatch: pytest.MonkeyPatch) -> None:
    """#1494 : uvicorn reçoit ws_ping_timeout depuis settings (pas orphelin)."""
    monkeypatch.setattr(settings, "ws_keepalive_timeout", 45.0)
    kwargs = uvicorn_ws_keepalive_kwargs()
    assert kwargs["ws_ping_timeout"] == 45.0
    assert kwargs["ws_ping_interval"] == 22.5
    assert get_ws_keepalive_timeout() == 45.0


def test_uvicorn_ws_kwargs_disable_when_nonpositive(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(settings, "ws_keepalive_timeout", 0.0)
    kwargs = uvicorn_ws_keepalive_kwargs()
    assert kwargs["ws_ping_timeout"] is None
    assert kwargs["ws_ping_interval"] is None


@pytest.mark.asyncio
async def test_receive_text_with_keepalive_times_out(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """#1494 : receive applicatif borné par le timeout settings."""

    class _SilentWS:
        async def receive_text(self) -> str:
            await asyncio.sleep(3600)
            return "never"

    monkeypatch.setattr(settings, "ws_keepalive_timeout", 0.05)
    with pytest.raises((asyncio.TimeoutError, TimeoutError)):
        await receive_text_with_keepalive(cast(WebSocket, _SilentWS()))


def test_sound_allowlist_accepts_wav_magic() -> None:
    content = b"RIFF" + b"\x00" * 8 + b"WAVEfmt "
    name = safe_media_upload.validate_sound_payload("beep.wav", content)
    assert name == "beep.wav"


def test_no_bluetooth_cmd_surface_in_bbia() -> None:
    """GHSA-vg3x / GHSA-993g : BBIA ne doit pas embarquer un handler CMD_/PIN_."""
    import bbia_sim

    # Smoke : le package principal ne réexporte pas de bluetooth_service
    assert not hasattr(bbia_sim, "bluetooth_service")
    assert not hasattr(bbia_sim, "BluetoothCommandService")
