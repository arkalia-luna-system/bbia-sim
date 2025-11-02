"""Test pour vérifier que la durée headless est respectée."""

import time
from unittest.mock import Mock, patch

from bbia_sim.sim.simulator import MuJoCoSimulator


class TestDurationFix:
    """Tests pour vérifier le respect de la durée en mode headless."""

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_headless_duration_respected(self, mock_mujoco):
        """Test que la durée headless est respectée avec une tolérance de 0.2s."""
        # Mock setup
        mock_model = Mock()
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

            # Test avec durée de 1 seconde
            start_time = time.monotonic()
            simulator.launch_simulation(headless=True, duration=1)
            end_time = time.monotonic()

            actual_duration = end_time - start_time

            # Vérifier que la durée est respectée avec une tolérance de 0.2s
            assert (
                0.8 <= actual_duration <= 1.2
            ), f"Durée attendue ~1s, obtenue {actual_duration:.2f}s"

        finally:
            os.unlink(temp_model)

    @patch("src.bbia_sim.sim.simulator.mujoco")
    def test_headless_no_duration_limit(self, mock_mujoco):
        """Test que sans durée spécifiée, la simulation s'arrête à la limite de sécurité."""
        # Mock setup
        mock_model = Mock()
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

            # Test sans durée (doit s'arrêter à 10000 steps)
            simulator.launch_simulation(headless=True, duration=None)

            # Vérifier que mj_step a été appelé environ 10000 fois
            assert mock_mujoco.mj_step.call_count >= 10000

        finally:
            os.unlink(temp_model)
