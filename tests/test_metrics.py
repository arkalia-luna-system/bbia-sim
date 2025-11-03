#!/usr/bin/env python3
"""Tests pour router metrics."""

import pytest
from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app

client = TestClient(app)


@pytest.mark.unit
@pytest.mark.fast
class TestMetrics:
    """Tests pour endpoints métriques."""

    def test_healthz(self):
        """Test endpoint /metrics/healthz."""
        response = client.get("/metrics/healthz")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "ok"
        assert "timestamp" in data
        assert data["service"] == "bbia-sim"

    def test_readyz(self):
        """Test endpoint /metrics/readyz."""
        response = client.get("/metrics/readyz")
        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert data["status"] in ["ready", "not_ready"]
        assert "timestamp" in data
        assert "checks" in data

    def test_health(self):
        """Test endpoint /metrics/health."""
        response = client.get("/metrics/health")
        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert data["status"] in ["healthy", "unhealthy"]
        assert "timestamp" in data
        assert "services" in data
        assert "system" in data

    def test_prometheus_metrics(self):
        """Test endpoint /metrics/prometheus."""
        response = client.get("/metrics/prometheus")
        # Peut retourner 200 (si prometheus disponible) ou 503 (si non disponible)
        assert response.status_code in [200, 503]
        if response.status_code == 200:
            assert "text/plain" in response.headers["content-type"]
        else:
            assert "Prometheus non disponible" in response.text
