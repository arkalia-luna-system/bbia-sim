#!/usr/bin/env python3
"""Router pour métriques et health checks."""

import logging
import os
import time
from typing import Any

try:
    import psutil

    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False
    psutil = None  # type: ignore[assignment]

from fastapi import APIRouter
from fastapi.responses import Response

try:
    from prometheus_client import Counter, Gauge, Histogram, generate_latest

    PROMETHEUS_AVAILABLE = True
except ImportError:
    PROMETHEUS_AVAILABLE = False

from ...simulation_service import simulation_service

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/metrics", tags=["metrics"])

# Métriques Prometheus (si disponible)
if PROMETHEUS_AVAILABLE:
    request_count = Counter(
        "bbia_requests_total", "Total requests", ["method", "endpoint", "status"]
    )
    request_latency = Histogram(
        "bbia_request_duration_seconds",
        "Request latency",
        ["method", "endpoint"],
        buckets=[0.01, 0.05, 0.1, 0.5, 1.0, 5.0],
    )
    cpu_usage = Gauge("bbia_cpu_usage_percent", "CPU usage percentage")
    memory_usage = Gauge("bbia_memory_usage_bytes", "Memory usage in bytes")
    simulation_fps = Gauge("bbia_simulation_fps", "Simulation FPS")
    active_connections = Gauge(
        "bbia_active_connections", "Active WebSocket connections"
    )

# Métriques système en mémoire (fallback si Prometheus non disponible)
_metrics_cache: dict[str, Any] = {
    "requests": {"total": 0, "by_endpoint": {}},
    "latency": {"p50": 0.0, "p95": 0.0, "p99": 0.0},
    "system": {"cpu": 0.0, "memory": 0},
    "simulation": {"fps": 0.0},
    "connections": {"active": 0},
}
_metrics_cache_lock = time.time()


@router.get("/prometheus")
async def get_prometheus_metrics() -> Response:
    """Endpoint Prometheus pour métriques.

    Returns:
        Métriques au format Prometheus

    """
    if not PROMETHEUS_AVAILABLE:
        return Response(
            content="# Prometheus non disponible - installer prometheus_client",
            media_type="text/plain",
            status_code=503,
        )

    # Mettre à jour métriques système
    try:
        if PSUTIL_AVAILABLE and psutil:
            process = psutil.Process(os.getpid())
            cpu_usage.set(process.cpu_percent(interval=0.1))
            memory_usage.set(process.memory_info().rss)
        else:
            cpu_usage.set(0.0)
            memory_usage.set(0)

        # FPS simulation (si disponible)
        if simulation_service.is_simulation_ready():
            # Essayer de récupérer FPS depuis le simulateur
            try:
                fps = getattr(simulation_service.simulator, "fps", 0.0)
                simulation_fps.set(fps)
            except Exception:
                simulation_fps.set(0.0)
        else:
            simulation_fps.set(0.0)

        # Connexions actives (à implémenter si manager disponible)
        active_connections.set(0)  # TODO: Récupérer depuis ConnectionManager

    except Exception as e:
        logger.warning(f"Erreur mise à jour métriques: {e}")

    return Response(content=generate_latest(), media_type="text/plain")


@router.get("/healthz")
async def healthz() -> dict[str, Any]:
    """Liveness probe - vérifie si le service est vivant.

    Returns:
        Statut de santé du service

    """
    return {
        "status": "ok",
        "timestamp": time.time(),
        "service": "bbia-sim",
    }


@router.get("/readyz")
async def readyz() -> dict[str, Any]:
    """Readiness probe - vérifie si le service est prêt.

    Returns:
        Statut de disponibilité du service

    """
    try:
        # Vérifier simulation
        sim_ready = simulation_service.is_simulation_ready()

        # Vérifier système
        if PSUTIL_AVAILABLE and psutil:
            process = psutil.Process(os.getpid())
            memory_ok = process.memory_info().rss < 2_000_000_000  # < 2GB
        else:
            memory_ok = True  # Si psutil non disponible, considérer OK

        ready = sim_ready and memory_ok

        return {
            "status": "ready" if ready else "not_ready",
            "timestamp": time.time(),
            "checks": {
                "simulation": sim_ready,
                "memory": memory_ok,
            },
        }
    except Exception as e:
        logger.error(f"Erreur readiness check: {e}")
        return {
            "status": "not_ready",
            "timestamp": time.time(),
            "error": str(e),
        }


@router.get("/health")
async def health() -> dict[str, Any]:
    """Health check détaillé (compatible avec endpoint existant).

    Returns:
        Statut de santé détaillé

    """
    try:
        if PSUTIL_AVAILABLE and psutil:
            process = psutil.Process(os.getpid())
            memory_mb = process.memory_info().rss / 1_048_576
            system_info = {
                "cpu_percent": process.cpu_percent(interval=0.1),
                "memory_mb": round(memory_mb, 2),
                "memory_percent": process.memory_percent(),
            }
        else:
            system_info = {
                "cpu_percent": 0.0,
                "memory_mb": 0.0,
                "memory_percent": 0.0,
                "note": "psutil non disponible",
            }

        return {
            "status": "healthy",
            "timestamp": time.time(),
            "services": {
                "api": "running",
                "simulator": (
                    "available"
                    if simulation_service.is_simulation_ready()
                    else "unavailable"
                ),
                "robot": "ready",
            },
            "system": system_info,
        }
    except Exception as e:
        logger.error(f"Erreur health check: {e}")
        return {
            "status": "unhealthy",
            "timestamp": time.time(),
            "error": str(e),
        }
