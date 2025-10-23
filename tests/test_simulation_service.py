"""Tests unitaires pour le service de simulation."""

import asyncio
from unittest.mock import Mock, patch

import pytest

from src.bbia_sim.daemon.simulation_service import SimulationService


class TestSimulationService:
    """Tests pour la classe SimulationService."""

    @pytest.fixture
    def mock_simulator(self):
        """Mock du simulateur MuJoCo."""
        mock_sim = Mock()
        mock_sim.get_robot_state.return_value = {
            "joint_positions": {
                "neck_yaw": 0.0,
                "right_shoulder_pitch": 0.0,
                "left_shoulder_pitch": 0.0,
                "right_elbow_pitch": 0.0,
                "left_elbow_pitch": 0.0,
                "right_gripper_joint": 0.0,
                "left_gripper_joint": 0.0,
            },
            "time": 0.0,
            "qpos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "qvel": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "n_joints": 8,
            "n_bodies": 9,
        }
        mock_sim.get_available_joints.return_value = [
            "neck_yaw",
            "right_shoulder_pitch",
            "left_shoulder_pitch",
            "right_elbow_pitch",
            "left_elbow_pitch",
            "right_gripper_joint",
            "left_gripper_joint",
        ]
        mock_sim.set_joint_position.return_value = True
        return mock_sim

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_init_with_model_path(self, mock_simulator_class, mock_simulator):
        """Test initialisation avec chemin modèle."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")

        assert service.model_path == "test_model.xml"
        assert service.simulator is None  # Le simulateur n'est créé qu'au démarrage
        assert service.is_running is False
        assert service._simulation_task is None

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_start_simulation_already_running(
        self, mock_simulator_class, mock_simulator
    ):
        """Test démarrage simulation déjà en cours."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")

        # Démarrer une première fois
        await service.start_simulation()
        assert service.is_running is True

        # Essayer de redémarrer
        result = await service.start_simulation()
        assert result is True  # Devrait retourner True sans erreur

        # Nettoyer
        await service.stop_simulation()

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_start_simulation_graphical_mode(
        self, mock_simulator_class, mock_simulator
    ):
        """Test démarrage simulation en mode graphique."""
        mock_simulator_class.return_value = mock_simulator
        mock_simulator.launch_simulation = Mock()

        service = SimulationService(model_path="test_model.xml")
        result = await service.start_simulation(headless=False)

        assert result is True
        assert service.is_running is True
        assert service._simulation_task is not None

        # Nettoyer
        await service.stop_simulation()

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_start_simulation_error(self, mock_simulator_class):
        """Test erreur lors du démarrage de la simulation."""
        mock_simulator_class.side_effect = Exception("Simulator initialization error")

        service = SimulationService(model_path="test_model.xml")
        result = await service.start_simulation()

        assert result is False
        assert service.is_running is False
        assert service.simulator is None

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_stop_simulation_not_running(
        self, mock_simulator_class, mock_simulator
    ):
        """Test arrêt simulation non démarrée."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")
        result = await service.stop_simulation()

        assert result is True
        assert service.is_running is False

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_stop_simulation_error(self, mock_simulator_class, mock_simulator):
        """Test erreur lors de l'arrêt de la simulation."""
        mock_simulator_class.return_value = mock_simulator
        mock_simulator.close.side_effect = Exception("Close error")

        service = SimulationService(model_path="test_model.xml")
        await service.start_simulation()

        result = await service.stop_simulation()

        # Devrait retourner False à cause de l'erreur
        assert result is False

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_run_graphical_simulation_fallback(
        self, mock_simulator_class, mock_simulator
    ):
        """Test fallback vers headless en cas d'erreur graphique."""
        mock_simulator_class.return_value = mock_simulator
        mock_simulator.launch_simulation.side_effect = Exception("Viewer error")

        service = SimulationService(model_path="test_model.xml")

        # Démarrer en mode graphique (devrait fallback vers headless)
        result = await service.start_simulation(headless=False)

        assert result is True
        assert service.is_running is True

        # Nettoyer
        await service.stop_simulation()

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_get_robot_state_simulation_not_running(
        self, mock_simulator_class, mock_simulator
    ):
        """Test récupération état robot sans simulation."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")
        state = service.get_robot_state()

        # Devrait retourner l'état par défaut
        assert "joint_positions" in state
        assert "time" in state
        assert state["time"] == 0.0

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_get_robot_state_simulator_error(
        self, mock_simulator_class, mock_simulator
    ):
        """Test récupération état robot avec erreur simulateur."""
        mock_simulator_class.return_value = mock_simulator
        mock_simulator.get_robot_state.side_effect = Exception("State error")

        service = SimulationService(model_path="test_model.xml")
        service.simulator = mock_simulator
        service.is_running = True

        state = service.get_robot_state()

        # Devrait retourner l'état par défaut en cas d'erreur
        assert "joint_positions" in state
        assert "time" in state

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_get_joint_positions_simulation_not_running(
        self, mock_simulator_class, mock_simulator
    ):
        """Test récupération positions joints sans simulation."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")
        positions = service.get_joint_positions()

        # Devrait retourner les positions par défaut
        assert isinstance(positions, dict)
        assert "neck_yaw" in positions
        assert positions["neck_yaw"] == 0.0

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_get_joint_positions_simulator_error(
        self, mock_simulator_class, mock_simulator
    ):
        """Test récupération positions joints avec erreur simulateur."""
        mock_simulator_class.return_value = mock_simulator
        mock_simulator.get_robot_state.side_effect = Exception("State error")

        service = SimulationService(model_path="test_model.xml")
        service.simulator = mock_simulator
        service.is_running = True

        positions = service.get_joint_positions()

        # Devrait retourner les positions par défaut en cas d'erreur
        assert isinstance(positions, dict)
        assert "neck_yaw" in positions

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_set_joint_position_simulation_not_running(
        self, mock_simulator_class, mock_simulator
    ):
        """Test définition position joint sans simulation."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")
        result = service.set_joint_position("neck_yaw", 0.5)

        assert result is False

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_set_joint_position_simulator_error(
        self, mock_simulator_class, mock_simulator
    ):
        """Test définition position joint avec erreur simulateur."""
        mock_simulator_class.return_value = mock_simulator
        mock_simulator.set_joint_position.side_effect = Exception("Set position error")

        service = SimulationService(model_path="test_model.xml")
        service.simulator = mock_simulator
        service.is_running = True

        result = service.set_joint_position("neck_yaw", 0.5)

        assert result is False

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_set_joint_position_success(self, mock_simulator_class, mock_simulator):
        """Test définition position joint réussie."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")
        service.simulator = mock_simulator
        service.is_running = True

        result = service.set_joint_position("neck_yaw", 0.5)

        assert result is True
        mock_simulator.set_joint_position.assert_called_once_with("neck_yaw", 0.5)

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_get_available_joints_simulator_error(
        self, mock_simulator_class, mock_simulator
    ):
        """Test récupération joints disponibles avec erreur simulateur."""
        mock_simulator_class.return_value = mock_simulator
        mock_simulator.get_available_joints.side_effect = Exception("Joints error")

        service = SimulationService(model_path="test_model.xml")
        service.simulator = mock_simulator

        joints = service.get_available_joints()

        # Devrait retourner les joints par défaut en cas d'erreur
        assert isinstance(joints, list)
        assert "neck_yaw" in joints

    def test_get_default_state(self):
        """Test état par défaut."""
        service = SimulationService(model_path="test_model.xml")
        state = service._get_default_state()

        assert "joint_positions" in state
        assert "time" in state
        assert "qpos" in state
        assert "qvel" in state
        assert state["time"] == 0.0
        assert len(state["qpos"]) == 8
        assert len(state["qvel"]) == 8

    def test_get_default_joint_positions(self):
        """Test positions joints par défaut."""
        service = SimulationService(model_path="test_model.xml")
        positions = service._get_default_joint_positions()

        assert isinstance(positions, dict)
        assert "neck_yaw" in positions
        assert "right_shoulder_pitch" in positions
        assert "left_shoulder_pitch" in positions
        assert all(pos == 0.0 for pos in positions.values())

    def test_get_default_joint_names(self):
        """Test noms joints par défaut."""
        service = SimulationService(model_path="test_model.xml")
        names = service._get_default_joint_names()

        assert isinstance(names, list)
        assert "neck_yaw" in names
        assert "right_shoulder_pitch" in names
        assert "left_shoulder_pitch" in names

    def test_is_simulation_ready_with_simulator(self):
        """Test vérification simulation prête avec simulateur."""
        service = SimulationService(model_path="test_model.xml")
        service.simulator = Mock()
        service.is_running = True

        ready = service.is_simulation_ready()
        assert ready is True

    def test_is_simulation_ready_simulator_not_running(self):
        """Test vérification simulation prête sans simulateur en cours."""
        service = SimulationService(model_path="test_model.xml")
        service.simulator = Mock()
        service.is_running = False

        ready = service.is_simulation_ready()
        assert ready is False

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_get_available_joints(self, mock_simulator_class, mock_simulator):
        """Test récupération articulations disponibles."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")
        joints = service.get_available_joints()

        assert isinstance(joints, list)
        assert len(joints) > 0
        assert "neck_yaw" in joints

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_simulation_thread_lifecycle(
        self, mock_simulator_class, mock_simulator
    ):
        """Test cycle de vie du thread de simulation."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")

        # Démarrer simulation
        await service.start_simulation()
        assert service.is_running is True

        # Arrêter simulation
        await service.stop_simulation()
        assert service.is_running is False

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_simulator_error_handling(self, mock_simulator_class):
        """Test gestion erreurs simulateur."""
        # Simuler une erreur lors de l'initialisation
        mock_simulator_class.side_effect = Exception("Simulator error")

        # L'initialisation ne devrait pas lever d'exception
        service = SimulationService(model_path="test_model.xml")
        assert service.model_path == "test_model.xml"

    @patch("src.bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_simulation_thread_exception(
        self, mock_simulator_class, mock_simulator
    ):
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
