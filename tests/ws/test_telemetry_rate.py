"""Tests pour vérifier la cadence de télémétrie WebSocket."""

import json
import os
import time

import pytest
from fastapi.testclient import TestClient

from src.bbia_sim.daemon.app.main import app


class TestTelemetryRate:
    """Tests pour vérifier la cadence de télémétrie WebSocket."""

    @pytest.mark.asyncio
    @pytest.mark.skipif(
        os.getenv("CI") is not None, reason="Test WebSocket flaky en CI"
    )
    async def test_telemetry_rate_stable(self):
        """Test que la télémétrie WebSocket envoie à ~10 Hz."""
        client = TestClient(app)

        # Ouvrir une connexion WebSocket
        with client.websocket_connect("/ws/telemetry") as websocket:
            messages = []
            start_time = time.time()

            # Collecter les messages pendant 1 seconde
            while time.time() - start_time < 1.0:
                try:
                    message = websocket.receive_text()
                    messages.append(message)
                except Exception:
                    break

            # Analyser la cadence
            duration = time.time() - start_time
            message_count = len(messages)
            rate = message_count / duration

            # Tolérance ±20% autour de 10 Hz
            expected_rate = 10.0
            min_rate = expected_rate * 0.8  # 8 Hz
            max_rate = expected_rate * 1.2  # 12 Hz

            assert min_rate <= rate <= max_rate, (
                f"Taux de télémétrie hors plage: {rate:.1f} Hz "
                f"(attendu: {expected_rate} Hz ±20%)"
            )

            # Vérifier que les messages sont valides
            for message in messages[:5]:  # Vérifier les premiers messages
                data = json.loads(message)
                assert "timestamp" in data
                assert "joints" in data

    @pytest.mark.asyncio
    @pytest.mark.skipif(
        os.getenv("CI") is not None, reason="Test WebSocket flaky en CI"
    )
    async def test_telemetry_message_format(self):
        """Test le format des messages de télémétrie."""
        client = TestClient(app)

        with client.websocket_connect("/ws/telemetry") as websocket:
            # Recevoir quelques messages
            messages = []
            for _ in range(3):
                try:
                    message = websocket.receive_text()
                    messages.append(message)
                except Exception:
                    break

            # Vérifier le format de chaque message
            for message in messages:
                data = json.loads(message)

                # Vérifier les champs requis
                assert "timestamp" in data
                assert "joints" in data

                # Vérifier le type des données (timestamp est une string ISO)
                assert isinstance(data["timestamp"], str)
                assert isinstance(data["joints"], dict)

                # Vérifier que les joints sont présents
                expected_joints = [
                    "neck_yaw",
                    "right_shoulder_pitch",
                    "right_elbow_pitch",
                    "right_gripper_joint",
                    "left_shoulder_pitch",
                    "left_elbow_pitch",
                    "left_gripper_joint",
                ]

                for joint in expected_joints:
                    assert joint in data["joints"], f"Joint {joint} manquant"
                    assert isinstance(data["joints"][joint], int | float)

    def test_websocket_connection_handling(self):
        """Test la gestion des connexions WebSocket."""
        client = TestClient(app)

        # Test connexion normale
        with client.websocket_connect("/ws/telemetry") as websocket:
            # La connexion devrait être établie
            assert websocket is not None

            # Test déconnexion propre
            websocket.close()
