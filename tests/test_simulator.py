"""Tests unitaires pour le simulateur MuJoCo."""

import os
import tempfile
from unittest.mock import Mock, patch

import pytest

from src.bbia_sim.sim.simulator import MuJoCoSimulator


class TestMuJoCoSimulator:
    """Tests pour la classe MuJoCoSimulator."""

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_init_with_valid_model(self, mock_mujoco):
        """Test initialisation avec modèle valide."""
        # Mock des objets MuJoCo
        mock_model = Mock()
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Créer un modèle MJCF temporaire valide
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

            assert simulator.model_path.name == temp_model.split("/")[-1]
            assert simulator.model is not None
            assert simulator.data is not None

        finally:
            os.unlink(temp_model)

    def test_init_with_missing_model(self):
        """Test initialisation avec modèle manquant."""
        with pytest.raises(FileNotFoundError):
            MuJoCoSimulator("nonexistent_model.xml")

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_init_with_invalid_model(self, mock_mujoco):
        """Test initialisation avec modèle invalide."""
        mock_mujoco.MjModel.from_xml_path.side_effect = RuntimeError("Invalid MJCF")

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write("invalid xml content")
            temp_model = f.name

        try:
            with pytest.raises(RuntimeError):
                MuJoCoSimulator(temp_model)
        finally:
            os.unlink(temp_model)

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_launch_simulation_headless(self, mock_mujoco):
        """Test lancement simulation headless."""
        # Mock setup
        mock_model = Mock()
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Créer modèle temporaire
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

            # Mock mj_step pour simuler la boucle
            step_count = 0

            def mock_mj_step(model, data):
                nonlocal step_count
                step_count += 1
                if step_count >= 10:  # Arrêter après 10 steps
                    return  # Just return instead of raising KeyboardInterrupt

            mock_mujoco.mj_step = mock_mj_step

            # Test headless avec durée courte
            simulator.launch_simulation(headless=True, duration=1)

            # Vérifier que mj_step a été appelé
            assert step_count > 0

        finally:
            os.unlink(temp_model)

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_get_robot_state(self, mock_mujoco):
        """Test récupération état robot."""
        # Mock setup
        mock_model = Mock()
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock des articulations
        mock_model.njnt = 3
        mock_qpos = Mock()
        mock_qpos.tolist.return_value = [0.1, 0.2, 0.3]
        mock_qpos.__getitem__ = lambda self, i: [0.1, 0.2, 0.3][i]
        mock_data.qpos = mock_qpos

        mock_qvel = Mock()
        mock_qvel.tolist.return_value = [0.01, 0.02, 0.03]
        mock_data.qvel = mock_qvel

        mock_data.time = 1.0

        # Mock joint names
        mock_joint1 = Mock()
        mock_joint1.name = "joint1"
        mock_joint2 = Mock()
        mock_joint2.name = "joint2"
        mock_joint3 = Mock()
        mock_joint3.name = "joint3"
        mock_model.joint.side_effect = [mock_joint1, mock_joint2, mock_joint3]

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="test">
  <worldbody>
    <body name="body1">
      <joint name="joint1" type="hinge"/>
      <joint name="joint2" type="hinge"/>
      <joint name="joint3" type="hinge"/>
    </body>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            simulator = MuJoCoSimulator(temp_model)
            state = simulator.get_robot_state()

            assert isinstance(state, dict)
            assert "joint_positions" in state
            assert "time" in state
            assert "qpos" in state
            assert "qvel" in state
            assert "n_joints" in state
            assert "n_bodies" in state

        finally:
            os.unlink(temp_model)

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_set_joint_position(self, mock_mujoco):
        """Test définition position articulation."""
        # Mock setup
        mock_model = Mock()
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock joint
        mock_joint = Mock()
        mock_joint.id = 0
        mock_model.joint.return_value = mock_joint
        mock_data.qpos = [0.0]
        
        # Mock joint_range pour le clamp
        mock_model.joint_range = [[-1.57, 1.57]]  # Limites pour le joint

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="test">
  <worldbody>
    <body name="body1">
      <joint name="test_joint" type="hinge"/>
    </body>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            simulator = MuJoCoSimulator(temp_model)

            # Test avec articulation valide
            simulator.set_joint_position("test_joint", 0.5)
            assert mock_data.qpos[0] == 0.5
            mock_mujoco.mj_forward.assert_called_once()

        finally:
            os.unlink(temp_model)

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_get_joint_position(self, mock_mujoco):
        """Test récupération position articulation."""
        # Mock setup
        mock_model = Mock()
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock joint
        mock_joint = Mock()
        mock_joint.id = 0
        mock_model.joint.return_value = mock_joint
        mock_data.qpos = [0.5]

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="test">
  <worldbody>
    <body name="body1">
      <joint name="test_joint" type="hinge"/>
    </body>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            simulator = MuJoCoSimulator(temp_model)

            # Test récupération position
            position = simulator.get_joint_position("test_joint")
            assert position == 0.5

        finally:
            os.unlink(temp_model)

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_get_available_joints(self, mock_mujoco):
        """Test récupération articulations disponibles."""
        # Mock setup
        mock_model = Mock()
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock joints
        mock_model.njnt = 3
        mock_joint1 = Mock()
        mock_joint1.name = "joint1"
        mock_joint2 = Mock()
        mock_joint2.name = "joint2"
        mock_joint3 = Mock()
        mock_joint3.name = "joint3"
        mock_model.joint.side_effect = [mock_joint1, mock_joint2, mock_joint3]

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="test">
  <worldbody>
    <body name="body1">
      <joint name="joint1" type="hinge"/>
      <joint name="joint2" type="hinge"/>
      <joint name="joint3" type="hinge"/>
    </body>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            simulator = MuJoCoSimulator(temp_model)
            joints = simulator.get_available_joints()

            assert isinstance(joints, list)
            assert len(joints) == 3
            assert "joint1" in joints
            assert "joint2" in joints
            assert "joint3" in joints

        finally:
            os.unlink(temp_model)

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_step_simulation(self, mock_mujoco):
        """Test step simulation."""
        mock_model = Mock()
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

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

            # Test step
            simulator._step_simulation()

            # Vérifier que mj_step a été appelé
            mock_mujoco.mj_step.assert_called_once_with(mock_model, mock_data)

        finally:
            os.unlink(temp_model)
