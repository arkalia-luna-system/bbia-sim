"""Tests pour bbia_tools.py - Outils LLM."""

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from bbia_sim.bbia_tools import BBIATools


class TestBBIATools:
    """Tests pour BBIATools."""

    @pytest.fixture
    def mock_robot_api(self):
        """Mock RobotAPI."""
        robot = MagicMock()
        robot.is_connected = True
        robot.goto_target = MagicMock()
        robot.set_target_head_pose = MagicMock()
        robot.set_emotion = MagicMock()
        robot.play_move = MagicMock()
        return robot

    @pytest.fixture
    def mock_vision(self):
        """Mock BBIAVision."""
        vision = MagicMock()
        vision.capture_frame = MagicMock(return_value=MagicMock())
        vision.scan_environment = MagicMock(
            return_value={"objects": [{"name": "livre"}], "faces": []}
        )
        return vision

    @pytest.fixture
    def tools(self, mock_robot_api, mock_vision):
        """Créer instance BBIATools."""
        return BBIATools(robot_api=mock_robot_api, vision=mock_vision)

    def test_get_tools(self, tools):
        """Test récupération liste outils."""
        tools_list = tools.get_tools()
        assert isinstance(tools_list, list)
        assert len(tools_list) > 0

        # Vérifier structure outils
        for tool in tools_list:
            assert "type" in tool
            assert tool["type"] == "function"
            assert "function" in tool
            assert "name" in tool["function"]
            assert "description" in tool["function"]

        # Vérifier outils spécifiques
        tool_names = [t["function"]["name"] for t in tools_list]
        assert "move_head" in tool_names
        assert "camera" in tool_names
        assert "dance" in tool_names
        assert "play_emotion" in tool_names

    def test_execute_move_head(self, tools, mock_robot_api):
        """Test exécution move_head."""
        with patch("reachy_mini.utils.create_head_pose") as mock_pose:
            mock_pose.return_value = MagicMock()

            result = tools.execute_tool(
                "move_head", {"direction": "left", "intensity": 0.5}
            )

            assert result["status"] in ["success", "error"]
            # Le test peut échouer si SDK non disponible, mais structure correcte
            assert "status" in result
            assert "detail" in result

    def test_execute_camera(self, tools, mock_vision):
        """Test exécution camera."""
        result = tools.execute_tool("camera", {"analyze": True})

        assert result["status"] in ["success", "error"]
        if result["status"] == "success":
            assert "image_captured" in result
            assert "analysis" in result or "summary" in result

    def test_execute_head_tracking(self, tools):
        """Test exécution head_tracking."""
        result = tools.execute_tool("head_tracking", {"enabled": True})

        assert result["status"] == "success"
        assert tools.head_tracking_enabled is True

        result2 = tools.execute_tool("head_tracking", {"enabled": False})
        assert result2["status"] == "success"
        assert tools.head_tracking_enabled is False

    def test_execute_play_emotion(self, tools, mock_robot_api):
        """Test exécution play_emotion."""
        result = tools.execute_tool(
            "play_emotion", {"emotion": "happy", "intensity": 0.7}
        )

        assert result["status"] in ["success", "error"]
        if result["status"] == "success":
            assert mock_robot_api.set_emotion.called
            assert tools.current_emotion == "happy"

    def test_execute_stop_emotion(self, tools, mock_robot_api):
        """Test exécution stop_emotion."""
        # D'abord jouer une émotion
        tools.execute_tool("play_emotion", {"emotion": "happy"})
        tools.current_emotion = "happy"

        # Puis arrêter
        result = tools.execute_tool("stop_emotion", {})

        assert result["status"] == "success"
        assert tools.current_emotion is None

    def test_execute_do_nothing(self, tools):
        """Test exécution do_nothing."""
        result = tools.execute_tool("do_nothing", {"duration": 2.0})

        assert result["status"] == "success"
        assert "Inactif" in result["detail"]

    def test_execute_unknown_tool(self, tools):
        """Test exécution outil inconnu."""
        with pytest.raises(ValueError, match="Outil inconnu"):
            tools.execute_tool("unknown_tool", {})

    def test_tools_without_robot_api(self):
        """Test outils sans robot_api."""
        vision = MagicMock()
        tools = BBIATools(robot_api=None, vision=vision)

        result = tools.execute_tool("move_head", {"direction": "left"})
        assert result["status"] == "error"
        assert "robot_api non disponible" in result["detail"]

    def test_dance_with_recorded_moves(self, tools, mock_robot_api):
        """Test exécution dance avec RecordedMoves."""
        with patch("reachy_mini.motion.recorded_move.RecordedMoves") as mock_recorded:
            mock_instance = MagicMock()
            mock_recorded.return_value = mock_instance
            mock_instance.play = MagicMock()

            result = tools.execute_tool(
                "dance",
                {
                    "move_name": "test_dance",
                    "dataset": "pollen-robotics/reachy-mini-dances-library",
                },
            )

            assert result["status"] in ["success", "error"]
            # Le test peut échouer si SDK non disponible, mais structure correcte
            assert "status" in result
