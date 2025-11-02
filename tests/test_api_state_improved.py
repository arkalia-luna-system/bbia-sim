"""Tests améliorés pour /api/state avec nouveaux paramètres."""

import pytest
from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app
from bbia_sim.daemon.config import settings

client = TestClient(app)


@pytest.fixture
def api_token() -> str:
    """Token d'authentification pour les tests."""
    return settings.api_token


class TestStateFull:
    """Tests pour GET /api/state/full avec paramètres optionnels."""

    def test_full_state_default(self, api_token: str) -> None:
        """Test full state avec paramètres par défaut."""
        response = client.get(
            "/api/state/full",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "timestamp" in data

    def test_full_state_with_control_mode(self, api_token: str) -> None:
        """Test full state avec control_mode."""
        response = client.get(
            "/api/state/full?with_control_mode=true",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        # Peut avoir control_mode si disponible

    def test_full_state_use_pose_matrix(self, api_token: str) -> None:
        """Test full state avec use_pose_matrix."""
        response = client.get(
            "/api/state/full?use_pose_matrix=true",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        # Vérifier format si head_pose présent
        if "head_pose" in data:
            # Peut être dict avec "m" (matrice) ou "x"/"y"/"z" (xyzrpy)
            assert isinstance(data["head_pose"], dict)


class TestStatePresentHeadPose:
    """Tests pour GET /api/state/present_head_pose avec use_pose_matrix."""

    def test_present_head_pose_default(self, api_token: str) -> None:
        """Test present_head_pose par défaut (xyzrpy)."""
        response = client.get(
            "/api/state/present_head_pose",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        # Format xyzrpy par défaut - data contient {"head_pose": {...}}
        assert "head_pose" in data
        head_pose = data["head_pose"]
        # head_pose peut être dict avec "x"/"y"/"z" (xyzrpy) ou "m" (matrice)
        assert "x" in head_pose or "m" in head_pose

    def test_present_head_pose_matrix(self, api_token: str) -> None:
        """Test present_head_pose avec use_pose_matrix=true."""
        response = client.get(
            "/api/state/present_head_pose?use_pose_matrix=true",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        # Format matrice 4x4 - data contient {"head_pose": {...}}
        assert "head_pose" in data
        head_pose = data["head_pose"]
        assert "m" in head_pose
        assert len(head_pose["m"]) == 16
