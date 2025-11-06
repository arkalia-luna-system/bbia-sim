"""Tests unitaires pour le simulateur MuJoCo."""

import os
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from bbia_sim.sim.simulator import MuJoCoSimulator


class TestMuJoCoSimulator:
    """Tests pour la classe MuJoCoSimulator."""

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_init_with_valid_model(self, mock_mujoco):
        """Test initialisation avec modèle valide."""
        # Mock des objets MuJoCo
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
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

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_init_with_invalid_model(self, mock_mujoco):
        """Test initialisation avec modèle invalide."""
        mock_mujoco.MjModel.from_xml_path.side_effect = RuntimeError("Invalid MJCF")

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write("invalid xml content")
            temp_model = f.name

        try:
            # Le code lève Exception générique, mais peut lever RuntimeError aussi
            with pytest.raises((RuntimeError, Exception)):
                MuJoCoSimulator(temp_model)
        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_launch_simulation_headless(self, mock_mujoco):
        """Test lancement simulation headless."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
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

            # OPTIMISATION RAM: Mock simple avec limite de steps pour éviter boucle infinie
            step_count = 0
            call_count = [0]  # Compteur d'appels à monotonic

            def mock_mj_step(model, data):
                nonlocal step_count
                step_count += 1
                # Limiter à 5 steps pour économiser RAM et éviter timeout
                if step_count >= 5:
                    # Forcer la sortie en simulant que le temps est écoulé
                    pass

            def mock_monotonic():
                # Simuler le temps qui passe: première fois retourne 0, puis 0.02 (durée atteinte)
                call_count[0] += 1
                if call_count[0] == 1:
                    return 0.0  # start_time
                else:
                    # Après quelques steps, retourner une valeur > duration pour arrêter la boucle
                    return 0.02  # > 0.01 (duration)

            # Patcher mj_step et time.monotonic dans le module simulator
            with patch("bbia_sim.sim.simulator.mujoco.mj_step", side_effect=mock_mj_step):
                with patch("bbia_sim.sim.simulator.time.monotonic", side_effect=mock_monotonic):
                    # Test headless avec durée très courte (0.01s) pour économiser RAM
                    simulator.launch_simulation(headless=True, duration=0.01)

            # Vérifier que mj_step a été appelé au moins une fois
            assert step_count > 0

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_launch_simulation_graphical_macos_error(self, mock_mujoco):
        """Test gestion erreur macOS pour simulation graphique."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
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

            # Mock l'erreur macOS
            mock_mujoco.viewer.launch_passive.side_effect = RuntimeError("mjpython required")

            with patch("bbia_sim.sim.simulator.sys.platform", "darwin"):
                with pytest.raises(RuntimeError, match="Viewer MuJoCo non disponible sur macOS"):
                    simulator.launch_simulation(headless=False)

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_launch_simulation_graphical_other_error(self, mock_mujoco):
        """Test gestion autres erreurs pour simulation graphique."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
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

            # OPTIMISATION RAM: Mock une erreur autre que macOS pour test rapide
            # Le code lève RuntimeError avec le message original quand ce n'est pas macOS
            mock_mujoco.viewer.launch_passive.side_effect = RuntimeError("Other error")

            # S'assurer que l'exception est levée immédiatement (pas de timeout)
            with patch("bbia_sim.sim.simulator.sys.platform", "linux"):
                # Le code doit lever l'exception directement (pas de message spécifique, juste l'erreur originale)
                with pytest.raises(RuntimeError):
                    simulator.launch_simulation(headless=False)

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_run_graphical_simulation(self, mock_mujoco):
        """Test simulation graphique."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock viewer - permettre au moins un appel
        call_count = 0

        def mock_is_running():
            nonlocal call_count
            call_count += 1
            return call_count < 2  # Arrêter après le premier appel

        mock_viewer = Mock()
        mock_viewer.is_running.side_effect = mock_is_running
        mock_viewer.close = Mock()

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
            simulator.viewer = mock_viewer

            # Test simulation graphique
            simulator._run_graphical_simulation(duration=1)

            # Vérifier que mj_step et sync ont été appelés
            mock_mujoco.mj_step.assert_called()
            mock_viewer.sync.assert_called()
            mock_viewer.close.assert_called_once()

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_run_graphical_simulation_no_viewer(self, mock_mujoco):
        """Test simulation graphique sans viewer."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
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
            simulator.viewer = None  # Pas de viewer

            # Test simulation graphique sans viewer
            simulator._run_graphical_simulation(duration=1)

            # Ne devrait pas appeler mj_step
            mock_mujoco.mj_step.assert_not_called()

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_load_scene_success(self, mock_mujoco):
        """Test chargement de scène réussi."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock viewer
        mock_viewer = Mock()
        mock_viewer.update_model = Mock()

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
            simulator.viewer = mock_viewer

            # Créer une scène temporaire
            with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as scene_f:
                scene_f.write(
                    """<?xml version="1.0"?>
<mujoco model="scene">
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
  </worldbody>
</mujoco>"""
                )
                scene_path = scene_f.name

            try:
                # Test chargement de scène
                simulator.load_scene(scene_path)

                # Vérifier que le modèle a été rechargé
                assert mock_mujoco.MjModel.from_xml_path.call_count == 2
                mock_viewer.update_model.assert_called_once()

            finally:
                os.unlink(scene_path)

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_load_scene_not_found(self, mock_mujoco):
        """Test chargement de scène inexistante."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
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

            # Test chargement de scène inexistante
            simulator.load_scene("nonexistent_scene.xml")

            # Devrait recharger le modèle par défaut (2 appels au total)
            assert mock_mujoco.MjModel.from_xml_path.call_count == 2

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_load_scene_fatal_error(self, mock_mujoco):
        """Test chargement de scène avec erreur fatale."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()

        # Créer une exception FatalError réelle
        class FatalError(Exception):
            pass

        mock_mujoco.FatalError = FatalError
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

            # Créer une scène invalide qui existe dans le bon répertoire
            scenes_dir = Path("src/bbia_sim/sim/scenes")
            scenes_dir.mkdir(parents=True, exist_ok=True)

            scene_file = scenes_dir / "invalid_scene.xml"
            with open(scene_file, "w") as scene_f:
                scene_f.write("invalid xml content")

            try:
                # Mock l'erreur fatale directement
                mock_mujoco.MjModel.from_xml_path.side_effect = FatalError("Invalid MJCF")

                with pytest.raises(FatalError):
                    simulator.load_scene("invalid_scene.xml")

            finally:
                # Nettoyer le fichier de scène
                if scene_file.exists():
                    os.unlink(scene_file)

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_get_robot_state(self, mock_mujoco):
        """Test récupération état robot."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock des articulations
        mock_model.njnt = 3
        mock_model.nbody = 4
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
        mock_joint1.id = 0
        mock_joint2 = Mock()
        mock_joint2.name = "joint2"
        mock_joint2.id = 1
        mock_joint3 = Mock()
        mock_joint3.name = "joint3"
        mock_joint3.id = 2
        mock_model.joint.side_effect = lambda name: next(
            (j for j in [mock_joint1, mock_joint2, mock_joint3] if j.name == name),
            mock_joint1,
        )

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

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_set_joint_position(self, mock_mujoco):
        """Test définition position articulation."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock joint
        mock_joint = Mock()
        mock_joint.id = 0
        mock_model.joint.return_value = mock_joint

        # Mock qpos comme un array mutable
        mock_qpos = Mock()
        mock_qpos.__getitem__ = Mock(return_value=0.0)
        mock_qpos.__setitem__ = Mock()
        mock_data.qpos = mock_qpos

        # Mock joint_range pour le clamp
        mock_model.joint_range = [[-1.57, 1.57]]  # Limites pour le joint

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="test">
    <worldbody>
    <body name="body1" mass="1.0">
      <inertia ixx="0.01" iyy="0.01" izz="0.01"/>
      <joint name="test_joint" type="hinge"/>
      <geom name="body1_geom" type="box" size="0.05 0.05 0.05"/>
    </body>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            simulator = MuJoCoSimulator(temp_model)

            # Test avec articulation valide
            simulator.set_joint_position("test_joint", 0.5)
            mock_qpos.__setitem__.assert_called_with(0, 0.5)
            mock_mujoco.mj_forward.assert_called_once()

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_set_joint_position_clamping(self, mock_mujoco):
        """Test clamp des angles hors limites."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock joint
        mock_joint = Mock()
        mock_joint.id = 0
        mock_model.joint.return_value = mock_joint

        # Mock qpos comme un array mutable
        mock_qpos = Mock()
        mock_qpos.__getitem__ = Mock(return_value=0.0)
        mock_qpos.__setitem__ = Mock()
        mock_data.qpos = mock_qpos

        # Mock joint_range avec limites strictes
        mock_model.joint_range = [[-1.0, 1.0]]  # Limites étroites

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="test">
    <worldbody>
    <body name="body1" mass="1.0">
      <inertia ixx="0.01" iyy="0.01" izz="0.01"/>
      <joint name="test_joint" type="hinge"/>
      <geom name="body1_geom" type="box" size="0.05 0.05 0.05"/>
    </body>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            simulator = MuJoCoSimulator(temp_model)

            # Test avec angle trop grand (devrait être clampé)
            simulator.set_joint_position("test_joint", 2.0)
            mock_qpos.__setitem__.assert_called_with(0, 1.0)  # Clampé à la limite max

            # Test avec angle trop petit (devrait être clampé)
            simulator.set_joint_position("test_joint", -2.0)
            mock_qpos.__setitem__.assert_called_with(0, -1.0)  # Clampé à la limite min

            # Vérifier que mj_forward a été appelé
            assert mock_mujoco.mj_forward.call_count == 2

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_set_joint_position_invalid_joint(self, mock_mujoco):
        """Test définition position avec joint inexistant."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock joint qui lève KeyError
        mock_model.joint.side_effect = KeyError("Joint not found")

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

            # Test avec joint inexistant
            with pytest.raises(KeyError):
                simulator.set_joint_position("invalid_joint", 0.5)

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_get_joint_position(self, mock_mujoco):
        """Test récupération position articulation."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
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
    <body name="body1" mass="1.0">
      <inertia ixx="0.01" iyy="0.01" izz="0.01"/>
      <joint name="test_joint" type="hinge"/>
      <geom name="body1_geom" type="box" size="0.05 0.05 0.05"/>
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

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_get_joint_position_invalid_joint(self, mock_mujoco):
        """Test récupération position avec joint inexistant."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock joint qui lève KeyError
        mock_model.joint.side_effect = KeyError("Joint not found")

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

            # Test avec joint inexistant
            with pytest.raises(KeyError):
                simulator.get_joint_position("invalid_joint")

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_get_available_joints(self, mock_mujoco):
        """Test récupération articulations disponibles."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
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
        mock_joints = [mock_joint1, mock_joint2, mock_joint3]
        mock_model.joint.side_effect = lambda i: (
            mock_joints[i]
            if isinstance(i, int)
            else next((j for j in mock_joints if j.name == i), mock_joint1)
        )

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

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_step_simulation(self, mock_mujoco):
        """Test step simulation."""
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
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

            # Test step - mj_step doit être appelé depuis le module mujoco
            with patch("bbia_sim.sim.simulator.mujoco.mj_step") as mock_step:
                simulator._step_simulation()
                # Vérifier que mj_step a été appelé
                mock_step.assert_called_once_with(mock_model, mock_data)

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_close_with_viewer(self, mock_mujoco):
        """Test fermeture avec viewer actif."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Mock viewer
        mock_viewer = Mock()
        mock_viewer.close = Mock()

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
            simulator.viewer = mock_viewer

            # Test fermeture
            simulator.close()

            # Vérifier que le viewer a été fermé
            mock_viewer.close.assert_called_once()
            assert simulator.viewer is None

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_close_without_viewer(self, mock_mujoco):
        """Test fermeture sans viewer."""
        # Mock setup
        mock_model = Mock()
        mock_model.nu = 7  # Nombre d'actuateurs
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
            simulator.viewer = None  # Pas de viewer

            # Test fermeture sans viewer
            simulator.close()

            # Ne devrait pas lever d'erreur
            assert simulator.viewer is None

        finally:
            os.unlink(temp_model)
