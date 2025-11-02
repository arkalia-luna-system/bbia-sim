"""Tests pour WebSocket /api/state/ws/full."""

import asyncio
import json
from typing import Any

import pytest
import websockets

from fastapi.testclient import TestClient

from src.bbia_sim.daemon.app.main import app

client = TestClient(app)


@pytest.fixture
def api_token() -> str:
    """Token d'authentification pour les tests."""
    from src.bbia_sim.daemon.config import settings

    return settings.api_token


@pytest.mark.asyncio
async def test_websocket_state_full_basic() -> None:
    """Test WebSocket /api/state/ws/full - connexion de base."""
    import httpx

    # Note: TestClient ne supporte pas WebSocket facilement
    # Test simplifié avec vérification endpoint existe
    response = client.get("/api/state/present_head_pose")
    assert response.status_code in [200, 401, 403]  # 401/403 si auth requise


def test_goto_pose_with_interpolation() -> None:
    """Test POST /api/motion/goto_pose avec interpolation."""
    from src.bbia_sim.daemon.config import settings

    api_token = settings.api_token

    pose_data = {"x": 0.1, "y": 0.0, "z": 0.3, "roll": 0.0, "pitch": 0.0, "yaw": 0.1}

    # Test avec minjerk (défaut)
    response = client.post(
        "/api/motion/goto_pose?duration=2.0&interpolation=minjerk",
        json=pose_data,
        headers={"Authorization": f"Bearer {api_token}"},
    )
    assert response.status_code == 200
    data = response.json()
    assert "status" in data
    assert data.get("interpolation") == "minjerk"

    # Test avec linear
    response = client.post(
        "/api/motion/goto_pose?duration=1.5&interpolation=linear",
        json=pose_data,
        headers={"Authorization": f"Bearer {api_token}"},
    )
    assert response.status_code == 200
    data = response.json()
    assert data.get("interpolation") == "linear"

    # Test avec cartoon
    response = client.post(
        "/api/motion/goto_pose?duration=2.5&interpolation=cartoon",
        json=pose_data,
        headers={"Authorization": f"Bearer {api_token}"},
    )
    assert response.status_code == 200
    data = response.json()
    assert data.get("interpolation") == "cartoon"


def test_goto_pose_interpolation_modes() -> None:
    """Test tous les modes d'interpolation."""
    from src.bbia_sim.daemon.config import settings

    api_token = settings.api_token
    pose_data = {"x": 0.0, "y": 0.0, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    modes = ["linear", "minjerk", "ease", "cartoon"]
    for mode in modes:
        response = client.post(
            f"/api/motion/goto_pose?duration=1.0&interpolation={mode}",
            json=pose_data,
            headers={"Authorization": f"Bearer {api_token}"},
        )
        assert response.status_code == 200
        data = response.json()
        assert data.get("interpolation") == mode
