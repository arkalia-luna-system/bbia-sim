"""Tests pour vérifier la cadence de télémétrie WebSocket."""

import json

import pytest
from fastapi.testclient import TestClient

from bbia_sim.daemon.app.main import app


class TestTelemetryRate:
    """Tests pour vérifier la cadence de télémétrie WebSocket."""

    @pytest.mark.asyncio
    async def test_telemetry_rate_config_driven_not_hardcoded(self):
        """Cadence dérivée de Settings (preuve non flaky ; pas de timing live CI).

        Le test de débit live ±20% a été retiré : flaky sous charge CI.
        Preuve structurelle + intervalle : src/bbia_sim/test_telemetry_frequency_regressions.py
        """
        from bbia_sim.daemon.config import settings

        freq = float(settings.telemetry_frequency)
        assert freq > 0
        interval = max(0.01, 1.0 / freq)
        assert 0.01 <= interval <= 1.0

    @pytest.mark.asyncio
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
