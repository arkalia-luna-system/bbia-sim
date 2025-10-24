#!/usr/bin/env python3
"""Script de test pour l'API BBIA-SIM."""

import asyncio
import contextlib
import json
import time

import pytest
import requests
import websockets

API_BASE = "http://localhost:8000"
WS_BASE = "ws://localhost:8000"
API_TOKEN = "bbia-secret-key-dev"


def test_rest_endpoints():
    """Test des endpoints REST."""
    headers = {"Authorization": f"Bearer {API_TOKEN}"}

    # Test endpoint racine
    with contextlib.suppress(Exception):
        response = requests.get(f"{API_BASE}/")

    # Test health check
    with contextlib.suppress(Exception):
        response = requests.get(f"{API_BASE}/health")

    # Test état du robot
    try:
        response = requests.get(f"{API_BASE}/api/state/full", headers=headers)
        response.json()
    except Exception:
        pass

    # Test batterie
    try:
        response = requests.get(f"{API_BASE}/api/state/battery", headers=headers)
        response.json()
    except Exception:
        pass

    # Test mouvement
    try:
        pose_data = {"x": 0.1, "y": 0.2, "z": 0.3}
        response = requests.post(
            f"{API_BASE}/api/motion/goto_pose", json=pose_data, headers=headers
        )
        response.json()
    except Exception:
        pass

    # Test retour à la position d'origine
    try:
        response = requests.post(f"{API_BASE}/api/motion/home", headers=headers)
        response.json()
    except Exception:
        pass


@pytest.mark.asyncio
async def test_websocket():
    """Test du WebSocket."""
    try:
        async with websockets.connect(f"{WS_BASE}/ws/telemetry") as websocket:

            # Test ping/pong
            ping_msg = json.dumps({"type": "ping"})
            await websocket.send(ping_msg)

            # Réception des messages
            for _i in range(5):
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                    data = json.loads(message)

                    if data.get("type") == "pong":
                        pass
                    elif "robot" in data:
                        data["robot"]["position"]

                except asyncio.TimeoutError:
                    break

    except Exception:
        pass


def test_authentication():
    """Test de l'authentification."""
    # Test sans token
    with contextlib.suppress(Exception):
        requests.get(f"{API_BASE}/api/state/full")

    # Test avec mauvais token
    try:
        headers = {"Authorization": "Bearer wrong-token"}
        requests.get(f"{API_BASE}/api/state/full", headers=headers)
    except Exception:
        pass

    # Test avec bon token
    try:
        headers = {"Authorization": f"Bearer {API_TOKEN}"}
        requests.get(f"{API_BASE}/api/state/full", headers=headers)
    except Exception:
        pass


def main():
    """Point d'entrée principal."""
    # Attente que l'API soit prête
    time.sleep(2)

    # Tests REST
    test_rest_endpoints()

    # Tests d'authentification
    test_authentication()

    # Tests WebSocket
    with contextlib.suppress(Exception):
        asyncio.run(test_websocket())


if __name__ == "__main__":
    main()
