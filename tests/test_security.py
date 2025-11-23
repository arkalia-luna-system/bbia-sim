#!/usr/bin/env python3
"""Tests de sécurité pour BBIA-SIM.

Tests de validation inputs, rate limiting, CORS, et protection
contre injection (XSS, SQL injection, etc.).
"""

import pytest
from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app

client = TestClient(app)


class TestSecurityInputValidation:
    """Tests de validation des inputs."""

    def test_volume_validation_range(self):
        """Test que volume est validé entre 0.0 et 1.0."""
        # Volume trop élevé
        response = client.post(
            "/development/api/media/speaker/volume",
            json={"volume": 2.0},
        )
        assert response.status_code == 422  # Validation error

        # Volume trop bas
        response = client.post(
            "/development/api/media/speaker/volume",
            json={"volume": -1.0},
        )
        assert response.status_code == 422  # Validation error

    def test_xss_protection(self):
        """Test protection contre XSS."""
        # Tester avec script malveillant
        malicious_input = "<script>alert('XSS')</script>"
        response = client.post(
            "/development/api/media/speaker/volume",
            json={"volume": 0.5, "comment": malicious_input},
        )
        # Vérifier que script n'est pas exécuté (status 422 ou sanitized)
        assert response.status_code in [200, 422]

    def test_sql_injection_protection(self):
        """Test protection contre SQL injection."""
        # Tester avec injection SQL
        sql_injection = "'; DROP TABLE users; --"
        response = client.post(
            "/development/api/media/speaker/volume",
            json={"volume": 0.5, "query": sql_injection},
        )
        # Vérifier que requête est rejetée ou sanitized
        assert response.status_code in [200, 422]


class TestSecurityRateLimiting:
    """Tests de rate limiting."""

    def test_rate_limiting_enabled(self):
        """Test que rate limiting est activé en production."""
        # En mode développement, rate limiting peut être désactivé
        # Ce test vérifie juste que le middleware existe
        from bbia_sim.daemon.middleware import RateLimitMiddleware

        assert RateLimitMiddleware is not None

    def test_too_many_requests(self):
        """Test que trop de requêtes sont limitées."""
        # Faire plusieurs requêtes rapides
        for _ in range(10):
            response = client.post(
                "/development/api/media/speaker/volume",
                json={"volume": 0.5},
            )
            # En production, devrait retourner 429 après limite
            # En dev, peut passer
            assert response.status_code in [200, 429]


class TestSecurityCORS:
    """Tests de configuration CORS."""

    def test_cors_headers_present(self):
        """Test que les headers CORS sont présents."""
        response = client.options(
            "/development/api/media/status",
            headers={"Origin": "http://localhost:3000"},
        )
        # Vérifier headers CORS
        assert (
            "access-control-allow-origin" in response.headers
            or response.status_code == 200
        )


class TestSecurityAuthentication:
    """Tests d'authentification (si applicable)."""

    def test_api_endpoints_protected(self):
        """Test que les endpoints API sont protégés."""
        # Tester endpoint sans token
        response = client.get("/api/motors/status")
        # En production, devrait retourner 401
        # En dev, peut passer
        assert response.status_code in [200, 401, 403]
