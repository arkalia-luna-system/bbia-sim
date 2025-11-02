"""Tests pour les endpoints /api/move conformes SDK officiel."""

import pytest
from fastapi.testclient import TestClient
from uuid import UUID

from bbia_sim.daemon.app.main import app
from bbia_sim.daemon.config import settings

client = TestClient(app)


@pytest.fixture
def api_token() -> str:
    """Token d'authentification pour les tests."""
    return settings.api_token


class TestMoveGoto:
    """Tests pour POST /api/move/goto (conforme SDK)."""

    def test_goto_with_head_pose(self, api_token: str) -> None:
        """Test goto avec head_pose."""
        request_data = {
            "head_pose": {
                "x": 0.1,
                "y": 0.0,
                "z": 0.3,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.1,
            },
            "duration": 2.0,
            "interpolation": "minjerk",
        }
        response = client.post(
            "/api/move/goto",
            json=request_data,
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "uuid" in data
        # UUID doit être valide
        UUID(data["uuid"])

    def test_goto_with_antennas(self, api_token: str) -> None:
        """Test goto avec antennas uniquement."""
        request_data = {
            "antennas": [0.1, -0.1],
            "duration": 1.5,
            "interpolation": "linear",
        }
        response = client.post(
            "/api/move/goto",
            json=request_data,
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "uuid" in data

    def test_goto_all_interpolation_modes(self, api_token: str) -> None:
        """Test tous les modes d'interpolation."""
        modes = ["linear", "minjerk", "ease", "cartoon"]
        for mode in modes:
            request_data = {
                "antennas": [0.0, 0.0],
                "duration": 1.0,
                "interpolation": mode,
            }
            response = client.post(
                "/api/move/goto",
                json=request_data,
                headers={"Authorization": f"Bearer {api_token}"},
            )
            assert response.status_code == 200
            data = response.json()
            assert "uuid" in data


class TestMoveRunning:
    """Tests pour GET /api/move/running."""

    def test_get_running_moves_empty(self, api_token: str) -> None:
        """Test récupération liste moves vide."""
        response = client.get(
            "/api/move/running",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)


class TestMoveStop:
    """Tests pour POST /api/move/stop."""

    def test_stop_move_invalid_uuid(self, api_token: str) -> None:
        """Test stop avec UUID invalide."""
        response = client.post(
            "/api/move/stop",
            json={"uuid": "invalid-uuid"},
            headers={"Authorization": f"Bearer {api_token}"},
        )
        # FastAPI/Pydantic retourne 422 pour validation échouée (UUID invalide)
        assert response.status_code == 422

    def test_stop_move_not_found(self, api_token: str) -> None:
        """Test stop avec UUID non trouvé."""
        import uuid

        fake_uuid = str(uuid.uuid4())
        response = client.post(
            "/api/move/stop",
            json={"uuid": fake_uuid},
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 404


class TestMovePlay:
    """Tests pour /api/move/play/*."""

    def test_play_wake_up(self, api_token: str) -> None:
        """Test POST /api/move/play/wake_up."""
        response = client.post(
            "/api/move/play/wake_up",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "uuid" in data

    def test_play_goto_sleep(self, api_token: str) -> None:
        """Test POST /api/move/play/goto_sleep."""
        response = client.post(
            "/api/move/play/goto_sleep",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "uuid" in data


class TestMoveSetTarget:
    """Tests pour /api/move/set_target."""

    def test_set_target(self, api_token: str) -> None:
        """Test POST /api/move/set_target."""
        target_data = {
            "target_head_pose": {
                "x": 0.1,
                "y": 0.0,
                "z": 0.3,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.1,
            },
            "target_antennas": [0.1, -0.1],
        }
        response = client.post(
            "/api/move/set_target",
            json=target_data,
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "ok"
