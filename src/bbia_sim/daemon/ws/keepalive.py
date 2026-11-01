"""Keepalive WebSocket configurable (anticipation Pollen #1494).

`BBIA_WS_KEEPALIVE_TIMEOUT` / `Settings.ws_keepalive_timeout` pilote :
- les kwargs uvicorn `ws_ping_interval` / `ws_ping_timeout`
- les `receive_text` applicatifs bornés (timeout client silencieux)
"""

from __future__ import annotations

import asyncio
from typing import Any

from fastapi import WebSocket


def get_ws_keepalive_timeout() -> float:
    """Retourne le timeout keepalive WS courant (secondes)."""
    from bbia_sim.daemon.config import settings

    return float(settings.ws_keepalive_timeout)


def uvicorn_ws_keepalive_kwargs() -> dict[str, Any]:
    """Kwargs uvicorn dérivés de `BBIA_WS_KEEPALIVE_TIMEOUT`.

    `timeout <= 0` désactive les pings protocole (None).
    """
    timeout = get_ws_keepalive_timeout()
    if timeout <= 0:
        return {"ws_ping_interval": None, "ws_ping_timeout": None}
    interval = max(1.0, min(timeout / 2.0, timeout))
    return {
        "ws_ping_interval": interval,
        "ws_ping_timeout": timeout,
    }


async def receive_text_with_keepalive(
    websocket: WebSocket,
    *,
    timeout: float | None = None,
) -> str:
    """`receive_text` borné par le keepalive configuré (#1494).

    Raises:
        asyncio.TimeoutError: si aucun message avant le timeout
            (alias de builtins.TimeoutError dès Python 3.11).
    """
    limit = get_ws_keepalive_timeout() if timeout is None else float(timeout)
    if limit <= 0:
        return await websocket.receive_text()
    return await asyncio.wait_for(websocket.receive_text(), timeout=limit)
