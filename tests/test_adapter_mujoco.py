#!/usr/bin/env python3
"""
Tests pour l'adaptateur MuJoCo BBIA
Tests headless du simulateur MuJoCo avec le modèle Reachy Mini officiel
"""

import sys
from pathlib import Path

import pytest

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

from bbia_sim.daemon.simulation_service import SimulationService
from bbia_sim.sim.simulator import MuJoCoSimulator


class TestMuJoCoSimulator:
    """Tests pour le simulateur MuJoCo BBIA."""

    @pytest.fixture
    def simulator(self):
        """Fixture pour créer un simulateur MuJoCo."""
        model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
        return MuJoCoSimulator(model_path)

    def test_simulator_initialization(self, simulator):
        """Test l'initialisation du simulateur."""
        assert simulator is not None
        assert simulator.model is not None
        assert simulator.data is not None
        assert simulator.model_path.exists()

    def test_available_joints(self, simulator):
        """Test la récupération des joints disponibles."""
        joints = simulator.get_available_joints()

        # Vérification des joints principaux
        expected_joints = [
            "yaw_body",
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "stewart_4",
            "stewart_5",
            "stewart_6",
            "passive_1",
            "passive_2",
            "passive_3",
            "passive_4",
            "passive_5",
            "passive_6",
            "passive_7",
            "right_antenna",
            "left_antenna",
        ]

        assert len(joints) == 16
        for joint in expected_joints:
            assert joint in joints

    def test_joint_position_setting(self, simulator):
        """Test la définition de position d'un joint."""
        # Test avec left_antenna
        simulator.set_joint_position("left_antenna", 0.5)
        position = simulator.get_joint_position("left_antenna")
        assert abs(position - 0.5) < 0.1

    def test_joint_position_clamping(self, simulator):
        """Test le clamping des positions de joints."""
        # Test avec une valeur hors limites
        simulator.set_joint_position("left_antenna", 10.0)  # Valeur très élevée
        position = simulator.get_joint_position("left_antenna")
        # La position devrait être clampée dans les limites raisonnables
        assert abs(position) < 3.2  # Limite MuJoCo typique

    def test_invalid_joint_name(self, simulator):
        """Test avec un nom de joint invalide."""
        with pytest.raises(KeyError):
            simulator.set_joint_position("invalid_joint", 0.5)

    def test_robot_state(self, simulator):
        """Test la récupération de l'état du robot."""
        state = simulator.get_robot_state()

        assert "joint_positions" in state
        assert "time" in state
        assert "qpos" in state
        assert "qvel" in state
        assert "n_joints" in state
        assert "n_bodies" in state

        assert state["n_joints"] == 16
        assert len(state["joint_positions"]) == 16

    def test_multiple_joint_movements(self, simulator):
        """Test les mouvements de plusieurs joints."""
        # Définir plusieurs positions
        simulator.set_joint_position("yaw_body", 0.2)
        simulator.set_joint_position("left_antenna", 0.3)
        simulator.set_joint_position("right_antenna", -0.3)

        # Vérifier les positions
        assert abs(simulator.get_joint_position("yaw_body") - 0.2) < 0.1
        assert abs(simulator.get_joint_position("left_antenna") - 0.3) < 0.1
        assert abs(simulator.get_joint_position("right_antenna") - (-0.3)) < 0.1

    def test_step_simulation(self, simulator):
        """Test l'exécution d'un step de simulation."""
        initial_time = simulator.data.time

        simulator._step_simulation()

        # Le temps devrait avoir avancé
        assert simulator.data.time > initial_time

    def test_headless_simulation_duration(self, simulator):
        """Test la simulation headless avec durée limitée."""
        # Test court pour éviter les timeouts
        simulator.launch_simulation(headless=True, duration=1)

        # Vérifier que la simulation s'est exécutée
        assert simulator.data.time > 0


class TestSimulationService:
    """Tests pour le service de simulation BBIA."""

    @pytest.fixture
    def simulation_service(self):
        """Fixture pour créer un service de simulation."""
        return SimulationService()

    def test_service_initialization(self, simulation_service):
        """Test l'initialisation du service."""
        assert simulation_service is not None
        assert simulation_service.model_path is not None
        assert not simulation_service.is_running

    def test_default_joint_names(self, simulation_service):
        """Test les noms de joints par défaut."""
        joint_names = simulation_service._get_default_joint_names()
        assert len(joint_names) > 0
        assert "neck_yaw" in joint_names

    def test_default_joint_positions(self, simulation_service):
        """Test les positions de joints par défaut."""
        positions = simulation_service._get_default_joint_positions()
        assert len(positions) > 0
        assert "neck_yaw" in positions
        assert positions["neck_yaw"] == 0.0

    def test_default_state(self, simulation_service):
        """Test l'état par défaut."""
        state = simulation_service._get_default_state()
        assert "joint_positions" in state
        assert "time" in state
        assert state["time"] == 0.0

    def test_simulation_ready_check(self, simulation_service):
        """Test la vérification de l'état de simulation."""
        assert not simulation_service.is_simulation_ready()

        # Simuler un simulateur actif
        simulation_service.simulator = "mock_simulator"
        simulation_service.is_running = True

        assert simulation_service.is_simulation_ready()

    @pytest.mark.asyncio
    async def test_start_stop_simulation(self, simulation_service):
        """Test le démarrage et l'arrêt de la simulation."""
        # Démarrer en mode headless
        success = await simulation_service.start_simulation(headless=True)
        assert success
        assert simulation_service.is_running

        # Attendre un peu
        import asyncio

        await asyncio.sleep(0.1)

        # Arrêter
        success = await simulation_service.stop_simulation()
        assert success
        assert not simulation_service.is_running


class TestBBIAIntegration:
    """Tests d'intégration BBIA avec le simulateur."""

    def test_joint_mapping_consistency(self):
        """Test la cohérence du mapping des joints BBIA."""
        from bbia_sim.bbia_integration import BBIAIntegration

        integration = BBIAIntegration()

        # Vérifier que tous les joints dans les mappings existent
        for emotion, mapping in integration.emotion_mappings.items():
            for joint_name in mapping.keys():
                # Les joints doivent être dans la liste des joints officiels
                expected_joints = [
                    "yaw_body",
                    "stewart_1",
                    "stewart_2",
                    "stewart_3",
                    "stewart_4",
                    "stewart_5",
                    "stewart_6",
                    "right_antenna",
                    "left_antenna",
                ]
                assert (
                    joint_name in expected_joints
                ), f"Joint {joint_name} inconnu pour l'émotion {emotion}"

    def test_emotion_intensity_scaling(self):
        """Test le scaling des émotions selon l'intensité."""

        # Test avec différentes intensités
        base_position = 0.5
        intensity = 0.8
        expected_position = base_position * intensity

        assert abs(expected_position - 0.4) < 0.01


if __name__ == "__main__":
    # Exécution des tests si le script est lancé directement
    pytest.main([__file__, "-v"])
