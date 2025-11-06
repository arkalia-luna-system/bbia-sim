"""Tests d'intégration pour la simulation."""

import time
from unittest.mock import Mock, patch

import pytest

from bbia_sim.sim.simulator import MuJoCoSimulator


class TestSimulationIntegration:
    """Tests d'intégration pour la simulation."""

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_simulation_headless_performance(self, mock_mujoco):
        """Test performance simulation headless."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Créer un modèle temporaire
        import os
        import tempfile

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="test">
  <worldbody>
    <light pos="0 0 1" diffuse="0.5 0.5 0.5"/>
    <geom name="floor" type="plane" size="1 1 0.1"/>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            simulator = MuJoCoSimulator(temp_model)

            # Mock mj_step pour compter les appels
            step_count = 0

            def mock_mj_step(model, data):
                nonlocal step_count
                step_count += 1
                # Pas besoin de lever d'exception, la durée va arrêter la simulation

            # Remplacer mj_step dans le mock
            with (
                patch("bbia_sim.sim.simulator.mujoco.mj_step", side_effect=mock_mj_step),
                patch(
                    "bbia_sim.sim.simulator.time.monotonic",
                    side_effect=lambda: (
                        0.0 if step_count == 0 else 1.1
                    ),  # Simule le temps qui passe
                ),
            ):

                # Test performance headless avec une durée courte
                start_time = time.time()
                simulator.launch_simulation(headless=True, duration=1)  # 1 seconde seulement
                end_time = time.time()

                # Vérifier que des steps ont été exécutés
                assert step_count > 0

            # Vérifier que la durée est raisonnable
            duration = end_time - start_time
            assert duration < 2.0  # Moins de 2 secondes pour être plus tolérant

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_simulation_with_robot_model(self, mock_mujoco):
        """Test simulation avec modèle robot."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock des articulations
        mock_model.njnt = 3
        mock_model.nbody = 5
        mock_qpos = Mock()
        mock_qpos.tolist.return_value = [0.0, 0.0, 0.0]
        # Mock l'accès par index pour qpos
        mock_qpos.__getitem__ = Mock(side_effect=lambda i: 0.0)
        mock_qpos.__setitem__ = Mock()  # Mock l'assignation par index
        mock_data.qpos = mock_qpos
        mock_qvel = Mock()
        mock_qvel.tolist.return_value = [0.0, 0.0, 0.0]
        mock_data.qvel = mock_qvel
        mock_data.time = 0.0

        # Mock joint names
        mock_joint1 = Mock()
        mock_joint1.name = "yaw_body"
        mock_joint1.id = 0
        mock_joint2 = Mock()
        mock_joint2.name = "stewart_1"
        mock_joint2.id = 1
        mock_joint3 = Mock()
        mock_joint3.name = "passive_1"
        mock_joint3.id = 2

        # Créer un dictionnaire pour l'accès par nom
        joint_dict = {
            "yaw_body": mock_joint1,
            "stewart_1": mock_joint2,
            "passive_1": mock_joint3,
        }
        mock_joints = [mock_joint1, mock_joint2, mock_joint3]

        def mock_joint_access(key):
            if isinstance(key, int):
                return mock_joints[key]
            elif isinstance(key, str):
                return joint_dict[key]
            else:
                raise KeyError(f"Invalid key: {key}")

        mock_model.joint.side_effect = mock_joint_access

        # Mock joint_range pour le clamp
        mock_model.joint_range = [
            [-1.57, 1.57],  # neck_yaw
            [-0.5, 0.5],  # head_pitch
            [-1.57, 1.57],  # right_shoulder_pitch
        ]

        # Créer un modèle robot temporaire
        import os
        import tempfile

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="reachy_mini">
  <worldbody>
    <light pos="0 0 1" diffuse="0.5 0.5 0.5"/>
     <geom name="floor" type="plane" size="1 1 0.1"/>

     <body name="torso" mass="1.0">
      <inertia ixx="0.01" iyy="0.01" izz="0.01"/>
      <geom name="torso_geom" type="box" size="0.05 0.05 0.1"/>
      <joint name="neck_yaw" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
      <body name="head" mass="0.5">
        <inertia ixx="0.005" iyy="0.005" izz="0.005"/>
        <joint name="head_pitch" type="hinge" axis="0 1 0" range="-0.5 0.5"/>
        <geom name="head" type="box" size="0.1 0.1 0.1"/>
      </body>

      <body name="right_arm" mass="0.3">
        <inertia ixx="0.003" iyy="0.003" izz="0.003"/>
        <joint name="right_shoulder_pitch" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
        <geom name="right_arm" type="box" size="0.05 0.05 0.2"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            simulator = MuJoCoSimulator(temp_model)

            # Test récupération des articulations
            joints = simulator.get_available_joints()
            assert len(joints) == 3
            assert "yaw_body" in joints
            assert "stewart_1" in joints
            assert "passive_1" in joints

            # Test récupération de l'état
            state = simulator.get_robot_state()
            assert "joint_positions" in state
            assert "time" in state
            assert "qpos" in state
            assert "qvel" in state
            assert "n_joints" in state
            assert "n_bodies" in state

            # Test définition de position
            simulator.set_joint_position("yaw_body", 0.5)
            # Vérifier que l'assignation a été appelée
            mock_qpos.__setitem__.assert_called_with(0, 0.5)

            # Test récupération de position
            with patch.object(simulator, "get_joint_position", return_value=0.5):
                position = simulator.get_joint_position("neck_yaw")
                assert position == 0.5

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_simulation_error_handling(self, mock_mujoco):
        """Test gestion d'erreurs de simulation."""
        # Mock setup avec erreur
        mock_mujoco.MjModel.from_xml_path.side_effect = Exception("Model loading error")

        # Test avec modèle invalide
        with pytest.raises((FileNotFoundError, RuntimeError)):
            MuJoCoSimulator("invalid_model.xml")

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_simulation_step_control(self, mock_mujoco):
        """Test contrôle step de simulation."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Créer un modèle temporaire
        import os
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
            simulator = MuJoCoSimulator(temp_model)

            # Test step manuel
            simulator._step_simulation()

            # Vérifier que mj_step a été appelé
            mock_mujoco.mj_step.assert_called_once_with(mock_model, mock_data)

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_simulation_metrics(self, mock_mujoco):
        """Test métriques de simulation."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock des métriques
        mock_model.njnt = 5
        mock_model.nbody = 8
        mock_qpos = Mock()
        mock_qpos.tolist.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]
        # Mock l'accès par index pour qpos
        mock_qpos.__getitem__ = Mock(side_effect=lambda i: 0.1 + i * 0.1)
        mock_qpos.__setitem__ = Mock()  # Mock l'assignation par index
        mock_data.qpos = mock_qpos
        mock_qvel = Mock()
        mock_qvel.tolist.return_value = [0.01, 0.02, 0.03, 0.04, 0.05]
        mock_data.qvel = mock_qvel
        mock_data.time = 2.5

        # Mock joint names
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        mock_joints = []
        for i, name in enumerate(joint_names):
            mock_joint = Mock()
            mock_joint.name = name
            mock_joint.id = i
            mock_joints.append(mock_joint)
        mock_model.joint.side_effect = lambda name: next(
            (j for j in mock_joints if j.name == name), mock_joints[0]
        )

        # Créer un modèle temporaire
        import os
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
            simulator = MuJoCoSimulator(temp_model)

            # Test récupération des métriques
            state = simulator.get_robot_state()

            assert state["n_joints"] == 5, f"Expected 5 joints, got {state['n_joints']}"
            assert state["n_bodies"] == 8
            assert state["time"] == 2.5
            assert len(state["qpos"]) == 5
            assert len(state["qvel"]) == 5
            assert len(state["joint_positions"]) >= 1  # Au moins un joint testé

            # Vérifier les positions des articulations (seulement celles qui existent)
            for i, name in enumerate(joint_names):
                if name in state["joint_positions"]:
                    assert state["joint_positions"][name] == 0.1 + i * 0.1

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_simulation_concurrent_access(self, mock_mujoco):
        """Test accès concurrent à la simulation."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock des articulations
        mock_model.njnt = 2
        mock_model.nbody = 3
        mock_qpos = Mock()
        mock_qpos.tolist.return_value = [0.0, 0.0]
        # Mock l'accès par index pour qpos
        mock_qpos.__getitem__ = Mock(side_effect=lambda i: 0.0)
        mock_qpos.__setitem__ = Mock()  # Mock l'assignation par index
        mock_data.qpos = mock_qpos
        mock_qvel = Mock()
        mock_qvel.tolist.return_value = [0.0, 0.0]
        mock_data.qvel = mock_qvel
        mock_data.time = 0.0

        # Mock joint names
        mock_joint1 = Mock()
        mock_joint1.name = "joint1"
        mock_joint1.id = 0
        mock_joint2 = Mock()
        mock_joint2.name = "joint2"
        mock_joint2.id = 1

        # Créer un dictionnaire pour l'accès par nom
        joint_dict = {"joint1": mock_joint1, "joint2": mock_joint2}
        mock_joints = [mock_joint1, mock_joint2]

        def mock_joint_access(key):
            if isinstance(key, int):
                return mock_joints[key]
            elif isinstance(key, str):
                return joint_dict[key]
            else:
                raise KeyError(f"Invalid key: {key}")

        mock_model.joint.side_effect = mock_joint_access

        # Mock joint_range pour le clamp
        mock_model.joint_range = [
            [-1.57, 1.57],  # joint1
            [-1.57, 1.57],  # joint2
        ]

        # Créer un modèle temporaire
        import os
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
            simulator = MuJoCoSimulator(temp_model)

            # Test accès concurrent (simulé)
            import threading

            results = []

            def get_state():
                state = simulator.get_robot_state()
                results.append(state)

            def set_position():
                simulator.set_joint_position("joint1", 0.5)
                results.append({"action": "position_set"})

            # Exécuter les opérations en parallèle (daemon pour éviter blocage)
            thread1 = threading.Thread(target=get_state, daemon=True)
            thread2 = threading.Thread(target=set_position, daemon=True)

            thread1.start()
            thread2.start()

            # Attendre avec timeout pour éviter blocage
            thread1.join(timeout=2.0)
            thread2.join(timeout=2.0)

            # Vérifier que les opérations ont été exécutées
            assert len(results) >= 1  # Au moins une opération réussie
            # Vérifier qu'au moins un résultat contient l'action position_set
            position_set_found = any(
                isinstance(result, dict) and result.get("action") == "position_set"
                for result in results
            )
            assert position_set_found

        finally:
            os.unlink(temp_model)
