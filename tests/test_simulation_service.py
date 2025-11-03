"""Tests unitaires pour le service de simulation."""

import asyncio
from unittest.mock import Mock, patch

import pytest

from bbia_sim.daemon.simulation_service import SimulationService


class TestSimulationService:
    """Tests pour la classe SimulationService."""

    @pytest.fixture
    def mock_simulator(self):
        """Mock du simulateur MuJoCo."""
        mock_sim = Mock()
        mock_sim.get_robot_state.return_value = {
            "joint_positions": {
                "yaw_body": 0.0,
                "stewart_1": 0.0,
                "stewart_2": 0.0,
                "stewart_3": 0.0,
                "left_antenna": 0.0,
                "right_antenna": 0.0,
            },
            "time": 0.0,
            "qpos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "qvel": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "n_joints": 6,
            "n_bodies": 9,
        }
        mock_sim.get_available_joints.return_value = [
            "yaw_body",
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "left_antenna",
            "right_antenna",
        ]
        mock_sim.set_joint_position.return_value = True
        return mock_sim

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_init_with_model_path(self, mock_simulator_class, mock_simulator):
        """Test initialisation avec chemin modèle."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")

        assert service.model_path == "test_model.xml"
        assert service.simulator is None  # Le simulateur n'est créé qu'au démarrage
        assert service.is_running is False
        assert service._simulation_task is None

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_start_simulation_error(self, mock_simulator_class):
        """Test erreur lors du démarrage de la simulation."""
        mock_simulator_class.side_effect = Exception("Simulator initialization error")

        service = SimulationService(model_path="test_model.xml")
        result = await service.start_simulation()

        assert result is False
        assert service.is_running is False
        assert service.simulator is None

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_get_joint_positions_simulation_not_running(
        self, mock_simulator_class, mock_simulator
    ):
        """Test récupération positions joints sans simulation."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")
        positions = service.get_joint_positions()

        # Devrait retourner les positions par défaut
        assert isinstance(positions, dict)
        assert "yaw_body" in positions
        assert positions["yaw_body"] == 0.0

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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
        assert "yaw_body" in positions

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_set_joint_position_simulation_not_running(
        self, mock_simulator_class, mock_simulator
    ):
        """Test définition position joint sans simulation."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")
        result = service.set_joint_position("yaw_body", 0.5)

        assert result is False

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_set_joint_position_simulator_error(
        self, mock_simulator_class, mock_simulator
    ):
        """Test définition position joint avec erreur simulateur."""
        mock_simulator_class.return_value = mock_simulator
        mock_simulator.set_joint_position.side_effect = Exception("Set position error")

        service = SimulationService(model_path="test_model.xml")
        service.simulator = mock_simulator
        service.is_running = True

        result = service.set_joint_position("yaw_body", 0.5)

        assert result is False

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_set_joint_position_success(self, mock_simulator_class, mock_simulator):
        """Test définition position joint réussie."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")
        service.simulator = mock_simulator
        service.is_running = True

        result = service.set_joint_position("yaw_body", 0.5)

        assert result is True
        mock_simulator.set_joint_position.assert_called_once_with("yaw_body", 0.5)

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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
        assert "yaw_body" in joints

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
        assert "yaw_body" in positions
        # Vérifier que les joints principaux sont présents
        assert all(pos == 0.0 for pos in positions.values())

    def test_get_default_joint_names(self):
        """Test noms joints par défaut."""
        service = SimulationService(model_path="test_model.xml")
        names = service._get_default_joint_names()

        assert isinstance(names, list)
        assert "yaw_body" in names

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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_stop_simulation_running(self, mock_simulator_class, mock_simulator):
        """Test arrêt simulation en cours."""
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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_get_available_joints(self, mock_simulator_class, mock_simulator):
        """Test récupération articulations disponibles."""
        mock_simulator_class.return_value = mock_simulator

        service = SimulationService(model_path="test_model.xml")
        joints = service.get_available_joints()

        assert isinstance(joints, list)
        assert len(joints) > 0
        assert "yaw_body" in joints

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
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

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    def test_simulator_error_handling(self, mock_simulator_class):
        """Test gestion erreurs simulateur."""
        # Simuler une erreur lors de l'initialisation
        mock_simulator_class.side_effect = Exception("Simulator error")

        # L'initialisation ne devrait pas lever d'exception
        service = SimulationService(model_path="test_model.xml")
        assert service.model_path == "test_model.xml"

    @patch("bbia_sim.daemon.simulation_service.MuJoCoSimulator")
    @pytest.mark.asyncio
    async def test_simulation_thread_exception(
        self, mock_simulator_class, mock_simulator
    ):
        """Test exception dans le thread de simulation."""
        # OPTIMISATION RAM: Créer un fichier temporaire pour éviter l'erreur FileNotFoundError
        import tempfile

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="test">
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            # OPTIMISATION RAM: Configurer le mock avant création du service
            mock_simulator_class.return_value = mock_simulator

            # OPTIMISATION RAM: Simuler une exception après quelques appels seulement
            call_count = 0

            def side_effect():
                nonlocal call_count
                call_count += 1
                # Lever exception après 2 appels pour tester la gestion d'erreur
                if call_count > 2:
                    raise Exception("Step error")
                # Retourner None pour les premiers appels (simulation normale)

            mock_simulator._step_simulation.side_effect = side_effect

            service = SimulationService(model_path=temp_model)

            # Le thread devrait gérer l'exception et continuer (la boucle gère les exceptions)
            result = await service.start_simulation()
            # Vérifier que le démarrage a réussi
            assert result is True
            assert service.is_running is True

            # OPTIMISATION RAM: Réduire le temps d'attente mais assez pour que l'exception se produise
            await asyncio.sleep(0.15)  # Laisser le temps à l'exception de se produire

            # Le service devrait toujours être en cours d'exécution malgré l'exception
            # (car la boucle gère les exceptions et continue avec un sleep)
            assert service.is_running is True

            # Nettoyer rapidement pour libérer RAM
            await service.stop_simulation()
            await asyncio.sleep(0.05)  # Attendre que le nettoyage se termine

        finally:
            # Nettoyer le fichier temporaire
            import os

            if os.path.exists(temp_model):
                os.unlink(temp_model)
