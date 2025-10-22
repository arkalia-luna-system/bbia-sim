"""Tests unitaires pour les routers state et motion."""

import pytest
from unittest.mock import Mock, patch
from fastapi.testclient import TestClient

from src.bbia_sim.daemon.app.main import app


class TestStateRouter:
    """Tests pour le router state."""

    @pytest.fixture
    def client(self):
        """Client de test FastAPI."""
        return TestClient(app)

    @patch('src.bbia_sim.daemon.app.routers.state.simulation_service')
    def test_get_full_state(self, mock_service, client):
        """Test endpoint state/full."""
        # Mock du service
        mock_service.get_robot_state.return_value = {
            "joint_positions": {"neck_yaw": 0.0},
            "time": 1.0,
            "n_joints": 1
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

    @patch('src.bbia_sim.daemon.app.routers.state.simulation_service')
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

    @patch('src.bbia_sim.daemon.app.routers.state.simulation_service')
    def test_get_joints_state(self, mock_service, client):
        """Test endpoint state/joints."""
        # Mock du service
        mock_service.get_joint_positions.return_value = {
            "neck_yaw": 0.5, 
            "head_pitch": 0.2
        }
        
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.get("/api/state/joints", headers=headers)
        
        assert response.status_code == 200
        data = response.json()
        assert "joints" in data
        assert "timestamp" in data
        assert len(data["joints"]) == 2


class TestMotionRouter:
    """Tests pour le router motion."""

    @pytest.fixture
    def client(self):
        """Client de test FastAPI."""
        return TestClient(app)

    @patch('src.bbia_sim.daemon.app.routers.motion.simulation_service')
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

    @patch('src.bbia_sim.daemon.app.routers.motion.simulation_service')
    def test_go_home(self, mock_service, client):
        """Test endpoint motion/home."""
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        response = client.post("/api/motion/home", headers=headers)
        
        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "estimated_time" in data
        assert "timestamp" in data

    @patch('src.bbia_sim.daemon.app.routers.motion.simulation_service')
    def test_set_joint_positions(self, mock_service, client):
        """Test endpoint motion/joints."""
        # Mock du service
        mock_service.get_available_joints.return_value = ["neck_yaw"]
        mock_service.set_joint_position.return_value = True
        
        headers = {"Authorization": "Bearer bbia-secret-key-dev"}
        payload = [{"joint_name": "neck_yaw", "position": 0.5}]
        response = client.post("/api/motion/joints", json=payload, headers=headers)
        
        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "joints" in data
        assert "success_count" in data
        assert "total_count" in data
        assert "estimated_time" in data
        assert "timestamp" in data

    @patch('src.bbia_sim.daemon.app.routers.motion.simulation_service')
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

    @patch('src.bbia_sim.daemon.app.routers.motion.simulation_service')
    def test_control_head(self, mock_service, client):
        """Test endpoint motion/head."""
        # Mock du service
        mock_service.get_available_joints.return_value = ["neck_yaw", "head_pitch"]
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

    @patch('src.bbia_sim.daemon.app.routers.motion.simulation_service')
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
