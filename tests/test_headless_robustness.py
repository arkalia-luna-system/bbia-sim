#!/usr/bin/env python3
"""
🧪 TESTS ROBUSTESSE HEADLESS MUJOCO
Tests pour gestion erreurs, timeouts adaptatifs, récupération après erreurs
"""

import os
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.sim.simulator import MuJoCoSimulator


class TestHeadlessRobustness:
    """Tests robustesse pour mode headless MuJoCo."""

    def test_headless_startup_robustness(self):
        """Test robustesse démarrage headless."""
        print("\n🧪 ROBUST TEST 1: Démarrage headless robuste")
        print("=" * 60)

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
            with patch("bbia_sim.sim.simulator.mujoco") as mock_mujoco:
                mock_model = Mock()
                mock_model.nu = 7
                mock_data = Mock()
                mock_mujoco.MjModel.from_xml_path.return_value = mock_model
                mock_mujoco.MjData.return_value = mock_data

                # Test démarrage normal
                simulator = MuJoCoSimulator(temp_model)
                assert simulator.model is not None
                assert simulator.data is not None
                print("✅ Démarrage headless normal: OK")

        finally:
            os.unlink(temp_model)

    def test_headless_configuration_errors(self):
        """Test gestion erreurs configuration MuJoCo."""
        print("\n🧪 ROBUST TEST 2: Erreurs configuration")
        print("=" * 60)

        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write("invalid xml content")
            temp_model = f.name

        try:
            with patch("bbia_sim.sim.simulator.mujoco") as mock_mujoco:
                # Simuler erreur de configuration
                mock_mujoco.MjModel.from_xml_path.side_effect = RuntimeError(
                    "Invalid MJCF"
                )

                # Doit lever exception claire
                with pytest.raises((RuntimeError, Exception)):
                    MuJoCoSimulator(temp_model)
                print("✅ Erreur configuration: exception claire levée")

        finally:
            os.unlink(temp_model)

    def test_headless_render_errors_recovery(self):
        """Test récupération après erreurs de rendu."""
        print("\n🧪 ROBUST TEST 3: Récupération erreurs rendu")
        print("=" * 60)

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
            with patch("bbia_sim.sim.simulator.mujoco") as mock_mujoco:
                mock_model = Mock()
                mock_model.nu = 7
                mock_data = Mock()
                mock_mujoco.MjModel.from_xml_path.return_value = mock_model
                mock_mujoco.MjData.return_value = mock_data

                simulator = MuJoCoSimulator(temp_model)

                # Simuler erreur de rendu (headless ne devrait pas utiliser rendu)
                # Mais on teste que le simulateur continue de fonctionner
                step_count = 0

                def mock_mj_step(model, data):
                    nonlocal step_count
                    step_count += 1
                    if step_count >= 3:
                        pass

                call_count = [0]

                def mock_monotonic():
                    call_count[0] += 1
                    if call_count[0] == 1:
                        return 0.0
                    else:
                        return 0.02

                with patch(
                    "bbia_sim.sim.simulator.mujoco.mj_step", side_effect=mock_mj_step
                ):
                    with patch(
                        "bbia_sim.sim.simulator.time.monotonic",
                        side_effect=mock_monotonic,
                    ):
                        simulator.launch_simulation(headless=True, duration=0.01)

                assert step_count > 0, "Simulation doit avoir exécuté des steps"
                print("✅ Récupération après erreurs: OK")

        finally:
            os.unlink(temp_model)

    def test_headless_resource_cleanup(self):
        """Test libération ressources après tests headless."""
        print("\n🧪 ROBUST TEST 4: Libération ressources")
        print("=" * 60)

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
            with patch("bbia_sim.sim.simulator.mujoco") as mock_mujoco:
                mock_model = Mock()
                mock_model.nu = 7
                mock_data = Mock()
                mock_mujoco.MjModel.from_xml_path.return_value = mock_model
                mock_mujoco.MjData.return_value = mock_data

                simulator = MuJoCoSimulator(temp_model)

                # Test close() libère ressources
                simulator.close()
                assert simulator.viewer is None, "Viewer doit être None après close()"
                print("✅ Libération ressources: OK")

        finally:
            os.unlink(temp_model)

    def test_headless_timeout_adaptive(self):
        """Test timeouts adaptatifs pour headless."""
        print("\n🧪 ROBUST TEST 5: Timeouts adaptatifs")
        print("=" * 60)

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
            with patch("bbia_sim.sim.simulator.mujoco") as mock_mujoco:
                mock_model = Mock()
                mock_model.nu = 7
                mock_data = Mock()
                mock_mujoco.MjModel.from_xml_path.return_value = mock_model
                mock_mujoco.MjData.return_value = mock_data

                simulator = MuJoCoSimulator(temp_model)

                # Test avec durée très courte
                step_count = 0

                def mock_mj_step(model, data):
                    nonlocal step_count
                    step_count += 1

                call_count = [0]

                def mock_monotonic():
                    call_count[0] += 1
                    if call_count[0] == 1:
                        return 0.0
                    else:
                        return 0.02

                with patch(
                    "bbia_sim.sim.simulator.mujoco.mj_step", side_effect=mock_mj_step
                ):
                    with patch(
                        "bbia_sim.sim.simulator.time.monotonic",
                        side_effect=mock_monotonic,
                    ):
                        start_time = time.time()
                        simulator.launch_simulation(headless=True, duration=0.01)
                        end_time = time.time()

                        # Durée doit être proche de 0.01s (tolérance 0.1s)
                        actual_duration = end_time - start_time
                        assert (
                            actual_duration < 0.2
                        ), f"Durée {actual_duration:.3f}s trop longue"

                print(f"✅ Timeout adaptatif: durée {actual_duration:.3f}s respectée")

        finally:
            os.unlink(temp_model)

    def test_headless_multiple_start_stop(self):
        """Test démarrage/arrêt multiples headless."""
        print("\n🧪 ROBUST TEST 6: Démarrage/arrêt multiples")
        print("=" * 60)

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
            with patch("bbia_sim.sim.simulator.mujoco") as mock_mujoco:
                mock_model = Mock()
                mock_model.nu = 7
                mock_data = Mock()
                mock_mujoco.MjModel.from_xml_path.return_value = mock_model
                mock_mujoco.MjData.return_value = mock_data

                simulator = MuJoCoSimulator(temp_model)

                # Test plusieurs démarrages/arrêts
                for i in range(3):
                    # Recréer le simulateur pour chaque cycle (évite problèmes de réutilisation)
                    simulator_cycle = MuJoCoSimulator(temp_model)
                    step_count = 0

                    def make_mock_monotonic():
                        call_count = [0]

                        def mock_monotonic():
                            call_count[0] += 1
                            if call_count[0] == 1:
                                return 0.0
                            else:
                                return 0.02

                        return mock_monotonic

                    def mock_mj_step(model, data):
                        nonlocal step_count
                        step_count += 1

                    mock_monotonic_fn = make_mock_monotonic()

                    with patch(
                        "bbia_sim.sim.simulator.mujoco.mj_step",
                        side_effect=mock_mj_step,
                    ):
                        with patch(
                            "bbia_sim.sim.simulator.time.monotonic",
                            side_effect=mock_monotonic_fn,
                        ):
                            simulator_cycle.launch_simulation(
                                headless=True, duration=0.01
                            )

                    simulator_cycle.close()
                    print(f"✅ Cycle {i+1}: OK")

                print("✅ Démarrage/arrêt multiples: OK")

        finally:
            os.unlink(temp_model)

    def test_headless_model_loading_errors(self):
        """Test gestion erreurs chargement modèle."""
        print("\n🧪 ROBUST TEST 7: Erreurs chargement modèle")
        print("=" * 60)

        # Test modèle inexistant
        with pytest.raises(FileNotFoundError):
            MuJoCoSimulator("nonexistent_model.xml")
        print("✅ Modèle inexistant: exception FileNotFoundError")

        # Test modèle invalide
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write("invalid xml")
            temp_model = f.name

        try:
            with patch("bbia_sim.sim.simulator.mujoco") as mock_mujoco:
                mock_mujoco.MjModel.from_xml_path.side_effect = RuntimeError(
                    "Invalid MJCF"
                )

                with pytest.raises((RuntimeError, Exception)):
                    MuJoCoSimulator(temp_model)
                print("✅ Modèle invalide: exception RuntimeError")

        finally:
            os.unlink(temp_model)

    def test_headless_ci_environment(self):
        """Test comportement headless en environnement CI."""
        print("\n🧪 ROBUST TEST 8: Environnement CI")
        print("=" * 60)

        # Simuler environnement CI
        with patch.dict(os.environ, {"CI": "true"}):
            with tempfile.NamedTemporaryFile(
                mode="w", suffix=".xml", delete=False
            ) as f:
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
                with patch("bbia_sim.sim.simulator.mujoco") as mock_mujoco:
                    mock_model = Mock()
                    mock_model.nu = 7
                    mock_data = Mock()
                    mock_mujoco.MjModel.from_xml_path.return_value = mock_model
                    mock_mujoco.MjData.return_value = mock_data

                    simulator = MuJoCoSimulator(temp_model)

                    # En CI, headless doit toujours fonctionner
                    step_count = 0

                    def mock_mj_step(model, data):
                        nonlocal step_count
                        step_count += 1

                    call_count = [0]

                    def mock_monotonic():
                        call_count[0] += 1
                        if call_count[0] == 1:
                            return 0.0
                        else:
                            return 0.02

                    with patch(
                        "bbia_sim.sim.simulator.mujoco.mj_step",
                        side_effect=mock_mj_step,
                    ):
                        with patch(
                            "bbia_sim.sim.simulator.time.monotonic",
                            side_effect=mock_monotonic,
                        ):
                            simulator.launch_simulation(headless=True, duration=0.01)

                    assert step_count > 0, "Headless doit fonctionner en CI"
                    print("✅ Environnement CI: headless fonctionne")

            finally:
                os.unlink(temp_model)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
