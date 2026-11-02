"""Package WebSocket daemon (télémétrie + keepalive #1494).

Le router monté par l'app est `bbia_sim.daemon.ws.telemetry`.
"""

from bbia_sim.daemon.ws.auth import ws_auth_rejected, ws_token_is_valid
from bbia_sim.daemon.ws.keepalive import (
    get_ws_keepalive_timeout,
    receive_text_with_keepalive,
    uvicorn_ws_keepalive_kwargs,
)
from bbia_sim.daemon.ws.telemetry import ConnectionManager

__all__ = [
    "ConnectionManager",
    "get_ws_keepalive_timeout",
    "receive_text_with_keepalive",
    "uvicorn_ws_keepalive_kwargs",
    "ws_auth_rejected",
    "ws_token_is_valid",
]
