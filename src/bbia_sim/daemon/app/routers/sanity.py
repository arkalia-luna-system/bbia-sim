"""Routeurs Sanity Checks (statut rapide + e-stop)."""

import logging
from datetime import datetime
from typing import Any

from fastapi import APIRouter

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend
from bbia_sim.daemon.simulation_service import simulation_service

router = APIRouter(prefix="/api/sanity", tags=["sanity"])


def _reachy_alive() -> dict[str, Any]:
    backend = ReachyMiniBackend(use_sim=True, timeout=1.0)
    ok = False
    try:
        ok = backend.connect()
        status = {
            "connected": ok,
            "available_joints": backend.get_available_joints(),
        }
        return {"ok": ok, "status": status}
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": str(e)}
    finally:
        try:
            backend.disconnect()
        except Exception as cleanup_error:
            logging.getLogger(__name__).warning(
                "Cleanup disconnect failed in _reachy_alive: %s",
                cleanup_error,
            )


@router.get("/status")
async def sanity_status() -> dict[str, Any]:
    return {
        "timestamp": datetime.now().isoformat(),
        "simulation_ready": simulation_service.is_simulation_ready(),
        "reachy": _reachy_alive(),
        "warnings": [],
    }


@router.post("/emergency_stop")
async def sanity_emergency_stop() -> dict[str, Any]:
    backend = ReachyMiniBackend(use_sim=True, timeout=1.0)
    ok = False
    try:
        backend.connect()
        ok = backend.emergency_stop()
        return {"ok": ok, "ts": datetime.now().isoformat()}
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": str(e)}
    finally:
        try:
            backend.disconnect()
        except Exception as cleanup_error:
            logging.getLogger(__name__).warning(
                "Cleanup disconnect failed in sanity_emergency_stop: %s",
                cleanup_error,
            )
