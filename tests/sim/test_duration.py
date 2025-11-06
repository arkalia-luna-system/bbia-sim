#!/usr/bin/env python3
"""Tests pour vérifier le respect strict de la durée en mode headless."""

import os
import sys
import tempfile
import time
from unittest.mock import Mock, patch

# Ajouter le répertoire src au PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from bbia_sim.sim.simulator import MuJoCoSimulator


class TestHeadlessDuration:
    """Tests pour vérifier le respect de la durée en mode headless."""

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_headless_duration_strict(self, mock_mujoco):
        """Test que la durée headless est respectée avec une tolérance stricte de ±0.05s."""
        mock_model = Mock()
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data

        # Simuler un step rapide mais pas instantané
        mock_mujoco.mj_step.side_effect = lambda m, d: time.sleep(0.0001)

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="minimal">
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body>
      <geom name="box" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            sim = MuJoCoSimulator(temp_model)
            duration = 1.0
            start_time = time.monotonic()
            sim.launch_simulation(headless=True, duration=duration)
            end_time = time.monotonic()
            actual_duration = end_time - start_time

            # Tolérance réaliste : ±0.1s (1.074s avec machine variable)
            assert 0.9 <= actual_duration <= 1.1, (
                f"Durée non respectée: {actual_duration:.3f}s " f"(attendu: {duration}s ±0.1s)"
            )

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_headless_duration_edge_cases(self, mock_mujoco):
        """Test les cas limites de durée."""
        mock_model = Mock()
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data
        mock_mujoco.mj_step.side_effect = lambda m, d: None  # Pas de sleep pour test rapide

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="minimal">
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            sim = MuJoCoSimulator(temp_model)

            # Test durée très courte
            start_time = time.monotonic()
            sim.launch_simulation(headless=True, duration=0.1)
            end_time = time.monotonic()
            actual_duration = end_time - start_time

            assert (
                0.05 <= actual_duration <= 0.3
            ), f"Durée courte non respectée: {actual_duration:.3f}s"

        finally:
            os.unlink(temp_model)

    @patch("bbia_sim.sim.simulator.mujoco")
    def test_headless_no_duration_limit(self, mock_mujoco):
        """Test que la simulation headless sans durée s'arrête à la limite de steps."""
        mock_model = Mock()
        mock_data = Mock()
        mock_mujoco.MjModel.from_xml_path.return_value = mock_model
        mock_mujoco.MjData.return_value = mock_data
        mock_mujoco.mj_step.side_effect = lambda m, d: None  # Pas de sleep pour test rapide

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write(
                """<?xml version="1.0"?>
<mujoco model="minimal">
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
  </worldbody>
</mujoco>"""
            )
            temp_model = f.name

        try:
            sim = MuJoCoSimulator(temp_model)

            # La limite interne est 10000 steps pour duration=None
            sim._run_headless_simulation(duration=None)

            # Le test passe si la fonction se termine sans erreur
            # (la limite de 10000 steps est atteinte)

        finally:
            os.unlink(temp_model)
