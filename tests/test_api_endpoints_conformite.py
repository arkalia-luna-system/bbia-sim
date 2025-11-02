"""Tests pour les nouveaux endpoints REST conformes au SDK officiel."""

import pytest

from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app

client = TestClient(app)


@pytest.fixture
def api_token() -> str:
    """Token d'authentification pour les tests."""
    from src.bbia_sim.daemon.config import settings

    token: str = str(settings.api_token)
    return token


class TestMotorsEndpoints:
    """Tests pour les endpoints /api/motors."""

    def test_get_motor_status(self, api_token: str) -> None:
        """Test GET /api/motors/status."""
        response = client.get(
            "/api/motors/status",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "mode" in data
        assert data["mode"] in ["enabled", "disabled", "gravity_compensation"]

    def test_set_motor_mode_enabled(self, api_token: str) -> None:
        """Test POST /api/motors/set_mode/enabled."""
        response = client.post(
            "/api/motors/set_mode/enabled",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        assert "status" in response.json()

    def test_set_motor_mode_disabled(self, api_token: str) -> None:
        """Test POST /api/motors/set_mode/disabled."""
        response = client.post(
            "/api/motors/set_mode/disabled",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200


class TestDaemonEndpoints:
    """Tests pour les endpoints /api/daemon."""

    def test_get_daemon_status(self, api_token: str) -> None:
        """Test GET /api/daemon/status."""
        response = client.get(
            "/api/daemon/status",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "simulation_running" in data

    def test_start_daemon(self, api_token: str) -> None:
        """Test POST /api/daemon/start."""
        response = client.post(
            "/api/daemon/start?wake_up=false",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        # Peut être 200 ou 409 si déjà démarré
        assert response.status_code in [200, 409]

    def test_stop_daemon(self, api_token: str) -> None:
        """Test POST /api/daemon/stop."""
        response = client.post(
            "/api/daemon/stop?goto_sleep=false",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200


class TestMotionEndpoints:
    """Tests pour les nouveaux endpoints /api/motion."""

    def test_wake_up(self, api_token: str) -> None:
        """Test POST /api/motion/wake_up."""
        response = client.post(
            "/api/motion/wake_up",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "status" in data

    def test_goto_sleep(self, api_token: str) -> None:
        """Test POST /api/motion/goto_sleep."""
        response = client.post(
            "/api/motion/goto_sleep",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "status" in data


class TestStateEndpoints:
    """Tests pour les nouveaux endpoints /api/state."""

    def test_get_present_head_pose(self, api_token: str) -> None:
        """Test GET /api/state/present_head_pose."""
        response = client.get(
            "/api/state/present_head_pose",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "head_pose" in data

    def test_get_present_body_yaw(self, api_token: str) -> None:
        """Test GET /api/state/present_body_yaw."""
        response = client.get(
            "/api/state/present_body_yaw",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "body_yaw" in data
        assert "unit" in data

    def test_get_present_antenna_joint_positions(self, api_token: str) -> None:
        """Test GET /api/state/present_antenna_joint_positions."""
        response = client.get(
            "/api/state/present_antenna_joint_positions",
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert "antennas" in data or ("left" in data and "right" in data)


class TestGotoPoseInterpolation:
    """Tests pour goto_pose avec interpolation."""

    def test_goto_pose_minjerk(self, api_token: str) -> None:
        """Test POST /api/motion/goto_pose avec minjerk."""
        pose = {"x": 0.1, "y": 0.0, "z": 0.3, "roll": 0.0, "pitch": 0.0, "yaw": 0.1}
        response = client.post(
            "/api/motion/goto_pose?duration=2.0&interpolation=minjerk",
            json=pose,
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert data.get("interpolation") == "minjerk"

    def test_goto_pose_linear(self, api_token: str) -> None:
        """Test POST /api/motion/goto_pose avec linear."""
        pose = {"x": 0.0, "y": 0.0, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        response = client.post(
            "/api/motion/goto_pose?duration=1.5&interpolation=linear",
            json=pose,
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert data.get("interpolation") == "linear"
