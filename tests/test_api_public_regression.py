#!/usr/bin/env python3
"""
Test de non-régression pour l'API publique.
Vérifie que les endpoints critiques de l'API publique fonctionnent correctement
et que leurs signatures ne changent pas de manière incompatibles.
"""

import pytest
from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app


@pytest.fixture
def api_client() -> TestClient:
    """Client de test pour l'API."""
    return TestClient(app)


@pytest.mark.unit
@pytest.mark.fast
def test_api_root_endpoint(api_client: TestClient) -> None:
    """Test endpoint racine."""
    # Le endpoint / peut retourner HTML (dashboard) ou JSON (fallback)
    response = api_client.get("/")
    assert response.status_code == 200
    # Si c'est HTML, tester l'endpoint JSON alternatif
    if "text/html" in response.headers.get("content-type", ""):
        response = api_client.get("/api")
        assert response.status_code == 200
    data = response.json()
    assert "message" in data or "version" in data


@pytest.mark.unit
@pytest.mark.fast
def test_api_health_check(api_client: TestClient) -> None:
    """Test endpoint health."""
    response = api_client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert "status" in data


@pytest.mark.unit
@pytest.mark.fast
def test_api_info_endpoint(api_client: TestClient) -> None:
    """Test endpoint info."""
    response = api_client.get("/api/info")
    assert response.status_code == 200
    data = response.json()
    # Vérifier champs essentiels
    assert "name" in data or "version" in data or "phase" in data


@pytest.mark.unit
@pytest.mark.fast
def test_api_ecosystem_capabilities(api_client: TestClient) -> None:
    """Test endpoint capabilities."""
    response = api_client.get("/api/ecosystem/capabilities")
    assert response.status_code == 200
    data = response.json()
    # Vérifier structure de base
    assert isinstance(data, dict)


@pytest.mark.unit
@pytest.mark.fast
def test_api_ecosystem_status(api_client: TestClient) -> None:
    """Test endpoint status."""
    response = api_client.get("/api/ecosystem/status")
    assert response.status_code == 200
    data = response.json()
    # Vérifier champs essentiels
    assert isinstance(data, dict)


@pytest.mark.unit
@pytest.mark.fast
def test_api_emotions_available(api_client: TestClient) -> None:
    """Test endpoint emotions disponibles."""
    response = api_client.get("/api/ecosystem/emotions/available")
    assert response.status_code == 200
    data = response.json()
    # Devrait retourner une liste d'émotions
    assert "emotions" in data
    assert isinstance(data["emotions"], list)


@pytest.mark.unit
@pytest.mark.fast
def test_api_behaviors_available(api_client: TestClient) -> None:
    """Test endpoint behaviors disponibles."""
    response = api_client.get("/api/ecosystem/behaviors/available")
    assert response.status_code == 200
    data = response.json()
    # Devrait retourner une liste de comportements
    assert "behaviors" in data
    assert isinstance(data["behaviors"], list)


@pytest.mark.unit
@pytest.mark.fast
def test_api_openapi_spec(api_client: TestClient) -> None:
    """Test spécification OpenAPI."""
    response = api_client.get("/openapi.json")
    assert response.status_code == 200
    data = response.json()
    # Vérifier structure OpenAPI
    assert "openapi" in data or "info" in data
    assert "paths" in data


@pytest.mark.unit
@pytest.mark.fast
def test_api_endpoints_signatures_stable(api_client: TestClient) -> None:
    """Test que les signatures des endpoints ne changent pas."""
    endpoints = [
        ("GET", "/"),
        ("GET", "/health"),
        ("GET", "/api/info"),
        ("GET", "/api/ecosystem/capabilities"),
        ("GET", "/api/ecosystem/status"),
        ("GET", "/api/ecosystem/emotions/available"),
        ("GET", "/api/ecosystem/behaviors/available"),
    ]

    for method, endpoint in endpoints:
        response = api_client.request(method, endpoint)
        # Tous devraient retourner 200 (pas 404, 500, etc.)
        assert response.status_code == 200, f"Endpoint {endpoint} échoué"


@pytest.mark.unit
@pytest.mark.fast
def test_api_response_formats(api_client: TestClient) -> None:
    """Test que les formats de réponse sont corrects."""
    # Tester plusieurs endpoints pour vérifier la cohérence
    endpoints = [
        "/api/info",
        "/api/ecosystem/capabilities",
        "/api/ecosystem/status",
    ]

    for endpoint in endpoints:
        response = api_client.get(endpoint)
        assert response.status_code == 200
        data = response.json()
        # Devrait toujours être un dict pour ces endpoints
        assert isinstance(data, dict), f"Format incorrect pour {endpoint}"
