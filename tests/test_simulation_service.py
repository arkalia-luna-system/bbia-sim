"""Tests unitaires pour le service de simulation."""

import pytest
import asyncio
from unittest.mock import Mock, patch, MagicMock
import threading
import time

from src.bbia_sim.daemon.simulation_service import SimulationService


class TestSimulationService:
    """Tests pour la classe SimulationService."""

    @pytest.fixture
    def mock_simulator(self):
        """Mock du simulateur MuJoCo."""
        mock_sim = Mock()
        mock_sim.get_robot_state.return_value = {
            "joint_positions": {"neck_yaw": 0.0, "right_shoulder_pitch": 0.0, "left_shoulder_pitch": 0.0, "right_elbow_pitch": 0.0, "left_elbow_pitch": 0.0, "right_gripper_joint": 0.0, "left_gripper_joint": 0.0},
            "time": 0.0,
            "qpos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "qvel": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "n_joints": 8,
            "n_bodies": 9
        }
        mock_sim.get_available_joints.return_value = ["neck_yaw", "right_shoulder_pitch", "left_shoulder_pitch", "right_elbow_pitch", "left_elbow_pitch", "right_gripper_joint", "left_gripper_joint"]
        mock_sim.set_joint_position.return_value = True
        return mock_sim

    @patch('src.bbia_sim.daemon.simulation_service.MuJoCoSimulator')
    def test_init_with_model_path(self, mock_simulator_class, mock_simulator):
        """Test initialisation avec chemin modèle."""
        mock_simulator_class.return_value = mock_simulator
        
        service = SimulationService(model_path="test_model.xml")
        
        assert service.model_path == "test_model.xml"
        assert service.simulator is None  # Le simulateur n'est créé qu'au démarrage
        assert service.is_running is False
        assert service._simulation_task is None

    @patch('src.bbia_sim.daemon.simulation_service.MuJoCoSimulator')
    @pytest.mark.asyncio
    async def test_start_simulation(self, mock_simulator_class, mock_simulator):
        """Test démarrage simulation."""
        mock_simulator_class.return_value = mock_simulator
        
        service = SimulationService(model_path="test_model.xml")
        result = await service.start_simulation()
        
        assert result is True
        assert service.is_running is True
        assert service._simulation_task is not None
        
        # Nettoyer
        await service.stop_simulation()

    @patch('src.bbia_sim.daemon.simulation_service.MuJoCoSimulator')
    @pytest.mark.asyncio
    async def test_stop_simulation(self, mock_simulator_class, mock_simulator):
        """Test arrêt simulation."""
        mock_simulator_class.return_value = mock_simulator
        
        service = SimulationService(model_path="test_model.xml")
        await service.start_simulation()
        
        # Attendre un peu que le thread démarre
        await asyncio.sleep(0.1)
        
        await service.stop_simulation()
        
        assert service.is_running is False
        # Attendre que le thread se termine
        if service._simulation_task:
            await asyncio.sleep(0.1)

    @patch('src.bbia_sim.daemon.simulation_service.MuJoCoSimulator')
    def test_get_robot_state(self, mock_simulator_class, mock_simulator):
        """Test récupération état robot."""
        mock_simulator_class.return_value = mock_simulator
        
        service = SimulationService(model_path="test_model.xml")
        state = service.get_robot_state()
        
        # Vérifier que l'état contient les bonnes clés
        assert "joint_positions" in state
        assert "time" in state
        assert "qpos" in state
        assert "qvel" in state

    @patch('src.bbia_sim.daemon.simulation_service.MuJoCoSimulator')
    def test_get_available_joints(self, mock_simulator_class, mock_simulator):
        """Test récupération articulations disponibles."""
        mock_simulator_class.return_value = mock_simulator
        
        service = SimulationService(model_path="test_model.xml")
        joints = service.get_available_joints()
        
        assert isinstance(joints, list)
        assert len(joints) > 0
        assert "neck_yaw" in joints

    @patch('src.bbia_sim.daemon.simulation_service.MuJoCoSimulator')
    def test_set_joint_position(self, mock_simulator_class, mock_simulator):
        """Test définition position articulation."""
        mock_simulator_class.return_value = mock_simulator
        
        service = SimulationService(model_path="test_model.xml")
        result = service.set_joint_position("neck_yaw", 0.5)
        
        # En mode non-running, ça devrait retourner False
        assert result is False

    def test_is_simulation_ready(self):
        """Test vérification simulation prête."""
        service = SimulationService(model_path="test_model.xml")
        ready = service.is_simulation_ready()
        
        # Sans simulateur initialisé, ça devrait être False
        assert ready is False

    @patch('src.bbia_sim.daemon.simulation_service.MuJoCoSimulator')
    @pytest.mark.asyncio
    async def test_simulation_thread_lifecycle(self, mock_simulator_class, mock_simulator):
        """Test cycle de vie du thread de simulation."""
        mock_simulator_class.return_value = mock_simulator
        
        service = SimulationService(model_path="test_model.xml")
        
        # Démarrer simulation
        await service.start_simulation()
        assert service.is_running is True
        
        # Arrêter simulation
        await service.stop_simulation()
        assert service.is_running is False

    @patch('src.bbia_sim.daemon.simulation_service.MuJoCoSimulator')
    @pytest.mark.asyncio
    async def test_multiple_start_stop(self, mock_simulator_class, mock_simulator):
        """Test démarrage/arrêt multiples."""
        mock_simulator_class.return_value = mock_simulator
        
        service = SimulationService(model_path="test_model.xml")
        
        # Premier cycle
        await service.start_simulation()
        assert service.is_running is True
        
        await service.stop_simulation()
        assert service.is_running is False
        
        # Deuxième cycle
        await service.start_simulation()
        assert service.is_running is True
        
        await service.stop_simulation()
        assert service.is_running is False

    @patch('src.bbia_sim.daemon.simulation_service.MuJoCoSimulator')
    def test_simulator_error_handling(self, mock_simulator_class):
        """Test gestion erreurs simulateur."""
        # Simuler une erreur lors de l'initialisation
        mock_simulator_class.side_effect = Exception("Simulator error")
        
        # L'initialisation ne devrait pas lever d'exception
        service = SimulationService(model_path="test_model.xml")
        assert service.model_path == "test_model.xml"

    @patch('src.bbia_sim.daemon.simulation_service.MuJoCoSimulator')
    @pytest.mark.asyncio
    async def test_simulation_thread_exception(self, mock_simulator_class, mock_simulator):
        """Test exception dans le thread de simulation."""
        mock_simulator_class.return_value = mock_simulator
        
        # Simuler une exception dans la boucle de simulation
        mock_simulator._step_simulation.side_effect = Exception("Step error")
        
        service = SimulationService(model_path="test_model.xml")
        
        # Le thread devrait gérer l'exception et s'arrêter proprement
        await service.start_simulation()
        await asyncio.sleep(0.1)  # Laisser le temps à l'exception de se produire
        
        # Le service devrait toujours être en cours d'exécution malgré l'exception
        assert service.is_running is True
        
        # Nettoyer
        await service.stop_simulation()
