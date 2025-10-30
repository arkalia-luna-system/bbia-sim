#!/usr/bin/env python3
"""Tests unitaires pour les routers state et motion."""

import os
import sys
from unittest.mock import patch

import pytest
from fastapi.testclient import TestClient

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim.daemon.app.main import app


class TestStateRouter:
    """Tests pour le router state."""

    @pytest.fixture
    def client(self):
        """Client de test FastAPI."""
        return TestClient(app)

    @patch("src.bbia_sim.daemon.app.routers.state.simulation_service")
    def test_get_full_state(self, mock_service, client):
        """Test endpoint state/full."""
        # Mock du service
        mock_service.get_robot_state.return_value = {
            "joint_positions": {"yaw_body": 0.0},
            "time": 1.0,
            "n_joints": 1,
        }

        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.get("/api/state/full", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "position" in data
        assert "status" in data
        assert "battery" in data
        assert "temperature" in data
        assert "timestamp" in data

    @patch("src.bbia_sim.daemon.app.routers.state.simulation_service")
    def test_get_battery_info(self, mock_service, client):
        """Test endpoint state/battery."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.get("/api/state/battery", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "level" in data
        assert "unit" in data
        assert "status" in data
        assert "estimated_time" in data

    @patch("src.bbia_sim.daemon.app.routers.state.simulation_service")
    def test_get_joints_state(self, mock_service, client):
        """Test endpoint state/joints."""
        # Mock du service - retourner les joints officiels Reachy Mini
        mock_service.get_joint_positions.return_value = {
            "yaw_body": 0.5,
            "stewart_1": 0.2,
            "stewart_2": 0.0,
            "stewart_3": 0.0,
            "stewart_4": 0.0,
            "stewart_5": 0.0,
            "stewart_6": 0.0,
            "left_antenna": 0.0,
            "right_antenna": 0.0,
        }

        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.get("/api/state/joints", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "joints" in data
        assert "timestamp" in data
        assert (
            len(data["joints"]) == 9
        )  # Nombre réel de joints dans le modèle Reachy Mini : yaw_body + 6 stewart + 2 antennes

    def test_get_position(self, client):
        """Test endpoint state/position."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.get("/api/state/position", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "position" in data
        assert "orientation" in data
        assert "timestamp" in data

    def test_get_temperature(self, client):
        """Test endpoint state/temperature."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.get("/api/state/temperature", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "temperature" in data
        assert "unit" in data
        assert "status" in data
        assert "timestamp" in data

    def test_get_status(self, client):
        """Test endpoint state/status."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.get("/api/state/status", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "mode" in data
        assert "errors" in data
        assert "warnings" in data
        assert "timestamp" in data

    def test_get_sensor_data(self, client):
        """Test endpoint state/sensors."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.get("/api/state/sensors", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "camera" in data
        assert "microphone" in data
        assert "imu" in data
        assert "timestamp" in data


class TestMotionRouter:
    """Tests pour le router motion."""

    @pytest.fixture
    def client(self):
        """Client de test FastAPI."""
        return TestClient(app)

    @patch("src.bbia_sim.daemon.app.routers.motion.simulation_service")
    def test_goto_pose(self, mock_service, client):
        """Test endpoint motion/goto_pose."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        payload = {"x": 0.5, "y": 0.5, "z": 1.0}
        response = client.post("/api/motion/goto_pose", json=payload, headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "target_pose" in data
        assert "estimated_time" in data
        assert "timestamp" in data

    @patch("src.bbia_sim.daemon.app.routers.motion.simulation_service")
    def test_go_home(self, mock_service, client):
        """Test endpoint motion/home."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.post("/api/motion/home", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "estimated_time" in data
        assert "timestamp" in data

    @patch("src.bbia_sim.daemon.app.routers.motion.simulation_service")
    def test_set_joint_positions(self, mock_service, client):
        """Test endpoint motion/joints."""
        # Mock du service
        mock_service.get_available_joints.return_value = ["yaw_body"]
        mock_service.set_joint_position.return_value = True

        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        payload = [{"joint_name": "yaw_body", "position": 0.5}]
        response = client.post("/api/motion/joints", json=payload, headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "joints" in data
        assert "success_count" in data
        assert "total_count" in data
        assert "estimated_time" in data
        assert "timestamp" in data

    @patch("src.bbia_sim.daemon.app.routers.motion.simulation_service")
    def test_control_gripper(self, mock_service, client):
        """Test endpoint motion/gripper/{side}."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.post("/api/motion/gripper/left?action=open", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "gripper" in data
        assert "action" in data
        assert "estimated_time" in data
        assert "timestamp" in data

    @patch("src.bbia_sim.daemon.app.routers.motion.simulation_service")
    def test_control_head(self, mock_service, client):
        """Test endpoint motion/head."""
        # Mock du service
        mock_service.get_available_joints.return_value = ["yaw_body", "head_pitch"]
        mock_service.set_joint_position.return_value = True

        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        payload = {"yaw": 0.5, "pitch": 0.2}
        response = client.post("/api/motion/head", json=payload, headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "target" in data
        assert "success" in data
        assert "estimated_time" in data
        assert "timestamp" in data

    def test_stop_motion(self, client):
        """Test endpoint motion/stop."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.post("/api/motion/stop", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "message" in data
        assert "timestamp" in data

    def test_get_motion_status(self, client):
        """Test endpoint motion/status."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.get("/api/motion/status", headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "current_action" in data
        assert "queue" in data
        assert "timestamp" in data

    @patch("src.bbia_sim.daemon.app.routers.motion.simulation_service")
    def test_execute_custom_command(self, mock_service, client):
        """Test endpoint motion/custom."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        payload = {"command": "test_command", "parameters": {"param1": "value1"}}
        response = client.post("/api/motion/custom", json=payload, headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "command" in data
        assert "parameters" in data
        assert "estimated_time" in data
        assert "timestamp" in data

    @patch("src.bbia_sim.daemon.app.routers.motion.simulation_service")
    def test_set_joint_positions_invalid_joint(self, mock_service, client):
        """Test endpoint motion/joints avec joint invalide."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        payload = [{"joint_name": "invalid_joint", "position": 0.5}]
        response = client.post("/api/motion/joints", json=payload, headers=headers)

        assert response.status_code == 422
        data = response.json()
        assert "detail" in data
        # L'erreur vient du modèle Pydantic, vérifions le message
        assert any("invalid_joint" in str(error) for error in data["detail"])

    @patch("src.bbia_sim.daemon.app.routers.motion.simulation_service")
    @patch("src.bbia_sim.daemon.app.routers.motion.clamp_joint_angle")
    def test_set_joint_positions_angle_clamping(self, mock_clamp, mock_service, client):
        """Test endpoint motion/joints avec clamp des angles."""
        # Mock du service
        mock_service.get_available_joints.return_value = ["yaw_body"]
        mock_service.set_joint_position.return_value = True

        # Mock du clamp pour retourner un angle différent
        mock_clamp.return_value = 0.3  # Angle clampé

        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        payload = [{"joint_name": "yaw_body", "position": 0.5}]
        response = client.post("/api/motion/joints", json=payload, headers=headers)

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "joints" in data
        assert "success_count" in data
        assert "total_count" in data

    def test_control_gripper_invalid_side(self, client):
        """Test endpoint motion/gripper/{side} avec côté invalide."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.post(
            "/api/motion/gripper/invalid?action=open", headers=headers
        )

        assert response.status_code == 400
        data = response.json()
        assert "error" in data
        assert "Côté invalide" in data["error"]

    def test_control_gripper_invalid_action(self, client):
        """Test endpoint motion/gripper/{side} avec action invalide."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.post(
            "/api/motion/gripper/left?action=invalid", headers=headers
        )

        assert response.status_code == 400
        data = response.json()
        assert "error" in data
        assert "Action invalide" in data["error"]
