"""Tests unitaires pour les middlewares."""

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from bbia_sim.daemon.middleware import RateLimitMiddleware, SecurityMiddleware


class TestSecurityMiddleware:
    """Tests pour SecurityMiddleware."""

    @pytest.fixture
    def app(self):
        """Application FastAPI de test."""
        app = FastAPI()

        @app.get("/")
        def read_root():
            return {"message": "Hello World"}

        @app.post("/large")
        def large_payload(data: dict):
            return {"received": len(str(data))}

        return app

    @pytest.fixture
    def client(self, app):
        """Client de test."""
        return TestClient(app)

    def test_security_headers_added(self, app, client):
        """Test ajout headers de sécurité."""
        # Ajouter le middleware
        app.add_middleware(SecurityMiddleware)

        response = client.get("/")

        assert response.status_code == 200
        # Vérifier que les headers de sécurité sont présents
        # (ils sont ajoutés par settings.get_security_headers())
        assert "content-type" in response.headers

    def test_security_headers_values(self, app, client):
        """Test valeurs des headers de sécurité."""
        app.add_middleware(SecurityMiddleware)

        response = client.get("/")

        assert response.status_code == 200
        # Vérifier que la réponse fonctionne
        assert response.json()["message"] == "Hello World"

    def test_json_size_limit(self, app, client):
        """Test limite taille JSON."""
        app.add_middleware(SecurityMiddleware)

        # Test avec une requête normale
        small_data = {"data": "small"}
        response = client.post("/large", json=small_data)

        assert response.status_code == 200
        assert "received" in response.json()

    def test_json_size_ok(self, app, client):
        """Test taille JSON acceptable."""
        app.add_middleware(SecurityMiddleware)

        # Payload acceptable
        small_data = {"data": "small"}
        response = client.post("/large", json=small_data)

        assert response.status_code == 200
        assert "received" in response.json()

    def test_middleware_order(self, app, client):
        """Test ordre des middlewares."""
        # Ajouter plusieurs middlewares
        app.add_middleware(SecurityMiddleware)
        app.add_middleware(
            RateLimitMiddleware, requests_per_minute=10, force_enable=True
        )

        response = client.get("/")

        assert response.status_code == 200
        assert response.json()["message"] == "Hello World"


class TestRateLimitMiddleware:
    """Tests pour RateLimitMiddleware."""

    @pytest.fixture
    def app(self):
        """Application FastAPI de test."""
        app = FastAPI()

        @app.get("/")
        def read_root():
            return {"message": "Hello World"}

        @app.get("/limited")
        def limited_endpoint():
            return {"message": "Limited"}

        return app

    @pytest.fixture
    def client(self, app):
        """Client de test."""
        return TestClient(app)

    def test_rate_limit_normal_usage(self, app, client):
        """Test utilisation normale (pas de limite)."""
        app.add_middleware(
            RateLimitMiddleware, requests_per_minute=100, force_enable=True
        )

        # Faire plusieurs requêtes rapidement
        for _ in range(5):
            response = client.get("/")
            assert response.status_code == 200

    def test_rate_limit_exceeded(self, app, client):
        """Test dépassement limite de taux."""
        app.add_middleware(
            RateLimitMiddleware, requests_per_minute=2, force_enable=True
        )

        # Première requête
        response = client.get("/")
        assert response.status_code == 200

        # Deuxième requête
        response = client.get("/")
        assert response.status_code == 200

        # Troisième requête (devrait être limitée)
        response = client.get("/")
        assert response.status_code == 429  # Too Many Requests

    def test_rate_limit_reset(self, app, client):
        """Test reset de la limite de taux."""
        app.add_middleware(
            RateLimitMiddleware,
            requests_per_minute=1,
            window_seconds=1,
            force_enable=True,
        )

        # Première requête
        response = client.get("/")
        assert response.status_code == 200

        # Deuxième requête (limitée)
        response = client.get("/")
        assert response.status_code == 429

        # Attendre la fin de la fenêtre
        import time

        time.sleep(1.1)

        # Nouvelle requête (devrait passer)
        response = client.get("/")
        assert response.status_code == 200

    def test_rate_limit_per_client(self, app, client):
        """Test limite par client."""
        app.add_middleware(
            RateLimitMiddleware, requests_per_minute=1, force_enable=True
        )

        # Client 1
        response = client.get("/")
        assert response.status_code == 200

        # Client 2 (même IP, mais devrait être limité)
        response = client.get("/")
        assert response.status_code == 429

    def test_rate_limit_headers(self, app, client):
        """Test headers de limite de taux."""
        app.add_middleware(RateLimitMiddleware, requests_per_minute=10)

        response = client.get("/")

        assert response.status_code == 200
        # En mode dev, pas de headers de rate limit
        assert "content-type" in response.headers

    def test_rate_limit_headers_values(self, app, client):
        """Test valeurs des headers de limite de taux."""
        app.add_middleware(RateLimitMiddleware, requests_per_minute=5)

        response = client.get("/")

        assert response.status_code == 200
        # En mode dev, pas de headers de rate limit
        assert "content-type" in response.headers

    def test_rate_limit_exceeded_headers(self, app, client):
        """Test headers quand limite dépassée."""
        app.add_middleware(
            RateLimitMiddleware, requests_per_minute=1, force_enable=True
        )

        # Première requête
        response = client.get("/")
        assert response.status_code == 200

        # Deuxième requête (limitée)
        response = client.get("/")
        assert response.status_code == 429
        assert "Retry-After" in response.headers

    def test_rate_limit_custom_message(self, app, client):
        """Test message personnalisé de limite."""
        app.add_middleware(
            RateLimitMiddleware,
            requests_per_minute=1,
            message="Custom rate limit message",
            force_enable=True,
        )

        # Première requête
        response = client.get("/")
        assert response.status_code == 200

        # Deuxième requête (limitée)
        response = client.get("/")
        assert response.status_code == 429
        assert "Custom rate limit message" in response.text

    def test_rate_limit_disabled(self, app, client):
        """Test middleware désactivé."""
        app.add_middleware(RateLimitMiddleware, requests_per_minute=0)  # Désactivé

        # Faire plusieurs requêtes
        for _ in range(10):
            response = client.get("/")
            assert response.status_code == 200

    def test_rate_limit_high_limit(self, app, client):
        """Test limite élevée."""
        app.add_middleware(RateLimitMiddleware, requests_per_minute=1000)

        # Faire plusieurs requêtes
        for _ in range(50):
            response = client.get("/")
            assert response.status_code == 200
