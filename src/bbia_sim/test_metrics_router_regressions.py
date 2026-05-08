"""Tests de non-regression pour le routeur metrics."""

import pytest

from bbia_sim.daemon.app.routers import metrics


def test_record_latency_ignores_invalid_values() -> None:
    metrics._latency_history.clear()
    metrics._record_latency(10.0)
    metrics._record_latency(-1.0)
    metrics._record_latency(float("nan"))
    metrics._record_latency(float("inf"))
    assert list(metrics._latency_history) == [10.0]


def test_calculate_percentiles_filters_invalid_values() -> None:
    values = [10.0, -2.0, float("nan"), 50.0, float("inf")]
    pct = metrics._calculate_percentiles(values)
    assert pct["p50"] == 50.0
    assert pct["p95"] == 50.0
    assert pct["p99"] == 50.0


@pytest.mark.asyncio
async def test_get_prometheus_metrics_returns_503_on_generate_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(metrics, "PROMETHEUS_AVAILABLE", True)

    def _boom() -> bytes:
        raise RuntimeError("generation failed")

    monkeypatch.setattr(metrics, "generate_latest", _boom)
    response = await metrics.get_prometheus_metrics()
    assert response.status_code == 503
