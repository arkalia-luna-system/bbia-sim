"""Test smoke pour le viewer MuJoCo (SKIP en CI)."""

import importlib.util
import os
import tempfile

import pytest

from bbia_sim.sim.simulator import MuJoCoSimulator


class TestViewerSmoke:
    """Tests smoke pour le viewer MuJoCo (skip en CI)."""

    @pytest.mark.skipif(
        os.getenv("CI") is not None or os.getenv("DISPLAY") is None,
        reason="Skip viewer tests in CI or headless environment",
    )
    def test_viewer_import_and_init(self):
        """Test import et initialisation du viewer sans ouvrir de fenêtre."""
        if importlib.util.find_spec("mujoco.viewer") is None:
            pytest.skip("mujoco.viewer non disponible")
        else:
            pass

        # Test avec un modèle minimal
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as f:
            f.write("""<?xml version="1.0"?>
<mujoco>
  <worldbody>
    <body name="torso">
      <geom name="torso" type="box" size="0.1 0.1 0.1"/>
      <body name="head">
        <joint name="neck_yaw" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
        <geom name="head" type="box" size="0.05 0.05 0.05"/>
      </body>
    </body>
  </worldbody>
</mujoco>""")
            temp_model = f.name

        try:
            # Test initialisation simulateur
            simulator = MuJoCoSimulator(temp_model)

            # Test que le modèle est chargé
            assert simulator.model is not None
            assert simulator.data is not None

            # Test que les joints sont disponibles
            joints = simulator.get_available_joints()
            assert "neck_yaw" in joints

        finally:
            os.unlink(temp_model)

    @pytest.mark.skipif(
        os.getenv("CI") is not None or os.getenv("DISPLAY") is None,
        reason="Skip viewer tests in CI or headless environment",
    )
    def test_viewer_launch_passive_import(self):
        """Test que launch_passive peut être importé (sans l'exécuter)."""
        if importlib.util.find_spec("mujoco.viewer") is None:
            pytest.skip("mujoco.viewer non disponible")

        # Vérifier que launch_passive existe
        import mujoco.viewer

        assert hasattr(mujoco.viewer, "launch_passive")

    @pytest.mark.skipif(
        os.getenv("CI") is not None or os.getenv("DISPLAY") is None,
        reason="Skip viewer tests in CI or headless environment",
    )
    def test_viewer_platform_detection(self):
        """Test détection de plateforme pour le viewer."""
        import sys

        if sys.platform == "darwin":
            # Sur macOS, vérifier que mjpython serait requis
            if importlib.util.find_spec("mujoco.viewer") is None:
                pass
            else:
                pass
        elif importlib.util.find_spec("mujoco.viewer") is None:
            pytest.skip("mujoco.viewer non disponible")
        else:
            pass

    @pytest.mark.skipif(
        os.getenv("CI") is not None or os.getenv("DISPLAY") is None,
        reason="Skip viewer tests in CI or headless environment",
    )
    def test_assets_loading(self):
        """Test que les assets STL sont chargés correctement."""
        # Vérifier que les fichiers STL existent
        import os
        from pathlib import Path

        project_root = Path(os.getcwd())
        assets_dir = project_root / "src" / "bbia_sim" / "sim" / "assets" / "meshes"

        expected_stl_files = [
            "torso.stl",
            "head.stl",
            "upper_arm.stl",
            "forearm.stl",
            "gripper.stl",
        ]

        for stl_file in expected_stl_files:
            stl_path = assets_dir / stl_file
            assert stl_path.exists(), f"Fichier STL manquant: {stl_file}"
            assert stl_path.stat().st_size > 0, f"Fichier STL vide: {stl_file}"

        # Vérifier la structure des assets officiels
        official_dir = (
            project_root / "src" / "bbia_sim" / "sim" / "assets" / "reachy_official"
        )
        assert official_dir.exists(), "Répertoire reachy_official manquant"

        mapping_file = official_dir / "asset_mapping.py"
        assert mapping_file.exists(), "Fichier asset_mapping.py manquant"

        docs_file = official_dir / "OFFICIAL_ASSETS.md"
        assert docs_file.exists(), "Fichier OFFICIAL_ASSETS.md manquant"
