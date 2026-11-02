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
        TimeoutError: si aucun message avant le timeout (builtin, y compris sous
            Python 3.10 où ``asyncio.TimeoutError`` n'est pas encore un alias).
    """
    limit = get_ws_keepalive_timeout() if timeout is None else float(timeout)
    if limit <= 0:
        # Starlette stub: receive_text → Any ; forcer str pour mypy no-any-return
        text = await websocket.receive_text()
        return str(text)
    try:
        text = await asyncio.wait_for(websocket.receive_text(), timeout=limit)
        return str(text)
    except TimeoutError:
        raise
    except Exception as exc:
        # Py3.10 : asyncio.TimeoutError ≠ builtins.TimeoutError — normaliser
        # pour que les ``except TimeoutError`` (UP041) fonctionnent partout.
        if type(exc).__name__ == "TimeoutError":
            raise TimeoutError("keepalive timeout") from exc
        raise
