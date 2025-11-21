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
    psutil = None  # type: ignore[assignment, unused-ignore]

from fastapi import APIRouter
from fastapi.responses import Response

try:
    from prometheus_client import Counter, Gauge, Histogram, generate_latest, REGISTRY

    PROMETHEUS_AVAILABLE = True
except ImportError:
    PROMETHEUS_AVAILABLE = False
    REGISTRY = None  # type: ignore[assignment, misc]

from ...simulation_service import simulation_service

logger = logging.getLogger(__name__)

# Import ConnectionManager pour métriques connexions actives
try:
    from ...ws.telemetry import manager as telemetry_manager

    TELEMETRY_MANAGER_AVAILABLE = True
except ImportError:
    TELEMETRY_MANAGER_AVAILABLE = False
    telemetry_manager = None  # type: ignore[assignment]

router = APIRouter(prefix="/metrics", tags=["metrics"])

# Métriques Prometheus (si disponible)
# OPTIMISATION: Vérifier si les métriques existent déjà pour éviter les doublons dans les tests
if PROMETHEUS_AVAILABLE and REGISTRY:
    # Fonction helper pour créer une métrique seulement si elle n'existe pas
    def _get_or_create_metric(metric_class, name, *args, **kwargs):
        """Crée une métrique seulement si elle n'existe pas déjà."""
        # Chercher si la métrique existe déjà
        for collector in list(REGISTRY._collector_to_names.keys()):
            if hasattr(collector, "_name") and collector._name == name:
                return collector
        # Créer la métrique si elle n'existe pas
        return metric_class(name, *args, **kwargs)

    try:
        request_count = _get_or_create_metric(
            Counter, "bbia_requests_total", "Total requests", ["method", "endpoint", "status"]
        )
        request_latency = _get_or_create_metric(
            Histogram,
            "bbia_request_duration_seconds",
            "Request latency",
            ["method", "endpoint"],
            buckets=[0.01, 0.05, 0.1, 0.5, 1.0, 5.0],
        )
        cpu_usage = _get_or_create_metric(
            Gauge, "bbia_cpu_usage_percent", "CPU usage percentage"
        )
        memory_usage = _get_or_create_metric(
            Gauge, "bbia_memory_usage_bytes", "Memory usage in bytes"
        )
        simulation_fps = _get_or_create_metric(
            Gauge, "bbia_simulation_fps", "Simulation FPS"
        )
        active_connections = _get_or_create_metric(
            Gauge, "bbia_active_connections", "Active WebSocket connections"
        )
    except (ValueError, KeyError) as e:
        # Si erreur de duplication, réutiliser les métriques existantes
        logger.warning(f"Métriques Prometheus déjà enregistrées, réutilisation: {e}")
        # Récupérer les métriques existantes depuis le registre
        request_count = None
        request_latency = None
        cpu_usage = None
        memory_usage = None
        simulation_fps = None
        active_connections = None
        for collector in list(REGISTRY._collector_to_names.keys()):
            if hasattr(collector, "_name"):
                name = collector._name
                if name == "bbia_requests_total":
                    request_count = collector
                elif name == "bbia_request_duration_seconds":
                    request_latency = collector
                elif name == "bbia_cpu_usage_percent":
                    cpu_usage = collector
                elif name == "bbia_memory_usage_bytes":
                    memory_usage = collector
                elif name == "bbia_simulation_fps":
                    simulation_fps = collector
                elif name == "bbia_active_connections":
                    active_connections = collector
else:
    request_count = None
    request_latency = None
    cpu_usage = None
    memory_usage = None
    simulation_fps = None
    active_connections = None

# Métriques système en mémoire (fallback si Prometheus non disponible)
# Historique des latences pour calcul p50/p95/p99 (garder dernières 1000 mesures)
_latency_history: list[float] = []
_MAX_LATENCY_HISTORY = 1000

_metrics_cache: dict[str, Any] = {
    "requests": {"total": 0, "by_endpoint": {}},
    "latency": {"p50": 0.0, "p95": 0.0, "p99": 0.0},
    "system": {"cpu": 0.0, "memory": 0},
    "simulation": {"fps": 0.0},
    "connections": {"active": 0},
}
_metrics_cache_lock = time.time()


def _calculate_percentiles(latencies: list[float]) -> dict[str, float]:
    """Calcule les percentiles p50, p95, p99 depuis une liste de latences."""
    if not latencies:
        return {"p50": 0.0, "p95": 0.0, "p99": 0.0}

    sorted_latencies = sorted(latencies)
    n = len(sorted_latencies)

    return {
        "p50": sorted_latencies[int(n * 0.50)] if n > 0 else 0.0,
        "p95": sorted_latencies[int(n * 0.95)] if n > 1 else sorted_latencies[-1],
        "p99": sorted_latencies[int(n * 0.99)] if n > 1 else sorted_latencies[-1],
    }


def _record_latency(latency_ms: float) -> None:
    """Enregistre une latence dans l'historique."""
    _latency_history.append(latency_ms)
    # Garder seulement les dernières N mesures
    if len(_latency_history) > _MAX_LATENCY_HISTORY:
        _latency_history.pop(0)


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
        if cpu_usage and memory_usage:
            if PSUTIL_AVAILABLE and psutil:
                process = psutil.Process(os.getpid())
                cpu_usage.set(process.cpu_percent(interval=0.1))
                memory_usage.set(process.memory_info().rss)
            else:
                cpu_usage.set(0.0)
                memory_usage.set(0)

        # FPS simulation (si disponible)
        if simulation_fps:
            if simulation_service.is_simulation_ready():
                # Essayer de récupérer FPS depuis le simulateur
                try:
                    fps = getattr(simulation_service.simulator, "fps", 0.0)
                    simulation_fps.set(fps)
                except (AttributeError, TypeError, RuntimeError):
                    simulation_fps.set(0.0)
            else:
                simulation_fps.set(0.0)

        # Connexions actives (récupérées depuis ConnectionManager)
        if active_connections:
            if TELEMETRY_MANAGER_AVAILABLE and telemetry_manager:
                active_connections.set(len(telemetry_manager.active_connections))
            else:
                active_connections.set(0)

    except Exception as e:
        logger.warning("Erreur mise à jour métriques: %s", e)

    return Response(content=generate_latest(), media_type="text/plain")


@router.get("/performance")
async def get_performance_metrics() -> dict[str, Any]:
    """Endpoint pour métriques de performance détaillées (p50/p95/p99).

    Returns:
        Métriques de performance avec percentiles
    """
    # Calculer percentiles depuis historique
    percentiles = _calculate_percentiles(_latency_history)

    # Mettre à jour cache
    _metrics_cache["latency"] = percentiles

    # Métriques système
    system_info: dict[str, Any] = {}
    if PSUTIL_AVAILABLE and psutil:
        process = psutil.Process(os.getpid())
        system_info = {
            "cpu_percent": process.cpu_percent(interval=0.1),
            "memory_mb": round(process.memory_info().rss / 1_048_576, 2),
            "memory_percent": process.memory_percent(),
        }
    else:
        system_info = {
            "cpu_percent": 0.0,
            "memory_mb": 0.0,
            "memory_percent": 0.0,
            "note": "psutil non disponible",
        }

    # FPS simulation
    fps = 0.0
    if simulation_service.is_simulation_ready():
        try:
            fps = getattr(simulation_service.simulator, "fps", 0.0)
        except (AttributeError, TypeError, RuntimeError):
            fps = 0.0

    return {
        "timestamp": time.time(),
        "latency": {
            "p50_ms": percentiles["p50"],
            "p95_ms": percentiles["p95"],
            "p99_ms": percentiles["p99"],
            "samples": len(_latency_history),
        },
        "system": system_info,
        "simulation": {"fps": fps},
    }


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
        logger.exception("Erreur readiness check: %s", e)
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
        logger.exception("Erreur health check: %s", e)
        return {
            "status": "unhealthy",
            "timestamp": time.time(),
            "error": str(e),
        }
