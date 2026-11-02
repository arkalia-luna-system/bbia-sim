"""Package WebSocket daemon (télémétrie + keepalive #1494).

Le router monté par l'app est `bbia_sim.daemon.ws.telemetry`.
"""

from bbia_sim.daemon.ws.auth import ws_auth_rejected, ws_token_is_valid
from bbia_sim.daemon.ws.keepalive import (
    get_ws_keepalive_timeout,
    receive_text_with_keepalive,
    uvicorn_ws_keepalive_kwargs,
)
from bbia_sim.daemon.ws.telemetry import (
    ConnectionManager,
    get_telemetry_info,
    manager,
    start_telemetry,
    stop_telemetry,
    websocket_endpoint,
)

__all__ = [
    "ConnectionManager",
    "get_telemetry_info",
    "get_ws_keepalive_timeout",
    "manager",
    "receive_text_with_keepalive",
    "start_telemetry",
    "stop_telemetry",
    "uvicorn_ws_keepalive_kwargs",
    "websocket_endpoint",
    "ws_auth_rejected",
    "ws_token_is_valid",
]
