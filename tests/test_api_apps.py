"""Tests pour les endpoints /api/apps."""

import pytest
from fastapi.testclient import TestClient

from src.bbia_sim.daemon.app.main import app
from src.bbia_sim.daemon.config import settings

client = TestClient(app)


@pytest.fixture
def api_token() -> str:
    """Token d'authentification pour les tests."""
    return settings.api_token


class TestAppsEndpoints:
    """Tests pour les endpoints apps."""

    def test_list_all_available_apps(self, api_token: str) -> None:
        """Test GET /api/apps/list-available."""
        response = client.get(
            "/api/apps/list-available",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)
        if len(data) > 0:
            assert "name" in data[0]

    def test_list_available_apps_by_source(self, api_token: str) -> None:
        """Test GET /api/apps/list-available/{source_kind}."""
        response = client.get(
            "/api/apps/list-available/huggingface",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)

    def test_install_app(self, api_token: str) -> None:
        """Test POST /api/apps/install."""
        app_info = {"name": "test_app", "source_kind": "huggingface"}
        response = client.post(
            "/api/apps/install",
            json=app_info,
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "job_id" in data

    def test_remove_app(self, api_token: str) -> None:
        """Test POST /api/apps/remove/{app_name}."""
        response = client.post(
            "/api/apps/remove/test_app",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "job_id" in data

    def test_job_status(self, api_token: str) -> None:
        """Test GET /api/apps/job-status/{job_id}."""
        response = client.get(
            "/api/apps/job-status/test_job_123",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "job_id" in data
        assert "status" in data

    def test_start_app(self, api_token: str) -> None:
        """Test POST /api/apps/start-app/{app_name}."""
        response = client.post(
            "/api/apps/start-app/bbia_chat",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "name" in data
        assert "status" in data
        assert "running" in data

    def test_restart_current_app(self, api_token: str) -> None:
        """Test POST /api/apps/restart-current-app."""
        # D'abord démarrer une app
        client.post(
            "/api/apps/start-app/bbia_chat",
            headers={"Authorization": f"Bearer {api_token}"},
        )

        response = client.post(
            "/api/apps/restart-current-app",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "name" in data
        assert data["status"] == "running"

    def test_stop_current_app(self, api_token: str) -> None:
        """Test POST /api/apps/stop-current-app."""
        # D'abord démarrer une app
        client.post(
            "/api/apps/start-app/bbia_chat",
            headers={"Authorization": f"Bearer {api_token}"},
        )

        response = client.post(
            "/api/apps/stop-current-app",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200

    def test_current_app_status(self, api_token: str) -> None:
        """Test GET /api/apps/current-app-status."""
        response = client.get(
            "/api/apps/current-app-status",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        # Peut être None ou un dict avec status
        data = response.json()
        assert data is None or ("name" in data and "status" in data)
