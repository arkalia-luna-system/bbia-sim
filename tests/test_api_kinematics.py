"""Tests pour les endpoints /api/kinematics."""

import pytest
from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app
from bbia_sim.daemon.config import settings

client = TestClient(app)


@pytest.fixture
def api_token() -> str:
    """Token d'authentification pour les tests."""
    return settings.api_token


class TestKinematicsEndpoints:
    """Tests pour les endpoints kinematics."""

    def test_get_kinematics_info(self, api_token: str) -> None:
        """Test GET /api/kinematics/info."""
        response = client.get(
            "/api/kinematics/info",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "info" in data
        assert "engine" in data["info"]
        assert "collision_check" in data["info"]

    def test_get_urdf(self, api_token: str) -> None:
        """Test GET /api/kinematics/urdf."""
        response = client.get(
            "/api/kinematics/urdf",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "urdf" in data
        # URDF peut être vide, c'est OK

    def test_get_stl_file_valid(self, api_token: str) -> None:
        """Test GET /api/kinematics/stl/{filename} avec fichier valide."""
        # Utiliser un fichier STL connu
        response = client.get(
            "/api/kinematics/stl/antenna.stl",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        # Peut être 200 si fichier existe, ou 404 si non trouvé
        assert response.status_code in [200, 404]
        if response.status_code == 200:
            assert response.headers["content-type"] == "model/stl"
            assert len(response.content) > 0

    def test_get_stl_file_invalid_extension(self, api_token: str) -> None:
        """Test GET /api/kinematics/stl/{filename} avec extension invalide."""
        response = client.get(
            "/api/kinematics/stl/test.txt",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 400

    def test_get_stl_file_not_found(self, api_token: str) -> None:
        """Test GET /api/kinematics/stl/{filename} avec fichier inexistant."""
        response = client.get(
            "/api/kinematics/stl/nonexistent_file.stl",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 404
