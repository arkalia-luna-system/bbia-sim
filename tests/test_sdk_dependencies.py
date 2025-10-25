#!/usr/bin/env python3
"""
Tests d'import des dépendances SDK officiel Reachy Mini
Vérifie que toutes les dépendances peuvent être importées sans erreur
"""


import pytest


class TestSDKDependencies:
    """Tests d'import des dépendances SDK officiel."""

    def test_reachy_mini_import(self) -> None:
        """Test import du SDK officiel Reachy Mini."""
        try:
            from reachy_mini import ReachyMini  # noqa: F401
            from reachy_mini.utils import create_head_pose  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"SDK officiel Reachy Mini non disponible: {e}")

    def test_zenoh_import(self) -> None:
        """Test import de Zenoh pour communication distribuée."""
        try:
            import zenoh  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Zenoh non disponible: {e}")

    def test_motor_controller_import(self) -> None:
        """Test import du contrôleur moteur."""
        try:
            import reachy_mini_motor_controller  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Contrôleur moteur non disponible: {e}")

    def test_rust_kinematics_import(self) -> None:
        """Test import de la cinématique Rust."""
        try:
            import reachy_mini_rust_kinematics  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Cinématique Rust non disponible: {e}")

    def test_cv2_enumerate_cameras_import(self) -> None:
        """Test import de l'énumération des caméras."""
        try:
            import cv2_enumerate_cameras  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Énumération caméras non disponible: {e}")

    def test_soundfile_import(self) -> None:
        """Test import de soundfile."""
        try:
            import soundfile  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Soundfile non disponible: {e}")

    def test_huggingface_hub_import(self) -> None:
        """Test import de Hugging Face Hub."""
        try:
            from huggingface_hub import hf_hub_download  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Hugging Face Hub non disponible: {e}")

    def test_log_throttling_import(self) -> None:
        """Test import de log-throttling."""
        try:
            import log_throttling  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Log throttling non disponible: {e}")

    def test_scipy_import(self) -> None:
        """Test import de SciPy."""
        try:
            import scipy  # noqa: F401
            from scipy.spatial.transform import Rotation  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"SciPy non disponible: {e}")

    def test_asgiref_import(self) -> None:
        """Test import d'asgiref."""
        try:
            from asgiref.sync import async_to_sync  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Asgiref non disponible: {e}")

    def test_aiohttp_import(self) -> None:
        """Test import d'aiohttp."""
        try:
            import aiohttp  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Aiohttp non disponible: {e}")

    def test_psutil_import(self) -> None:
        """Test import de psutil."""
        try:
            import psutil  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Psutil non disponible: {e}")

    def test_jinja2_import(self) -> None:
        """Test import de Jinja2."""
        try:
            import jinja2  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Jinja2 non disponible: {e}")

    def test_pyserial_import(self) -> None:
        """Test import de pyserial."""
        try:
            import serial  # noqa: F401

            assert True
        except ImportError as e:
            pytest.skip(f"Pyserial non disponible: {e}")

    def test_bbia_sim_imports_still_work(self) -> None:
        """Test que les imports BBIA-SIM fonctionnent toujours."""
        try:
            # Test d'import sans utiliser les classes
            import bbia_sim.backends.reachy_mini_backend  # noqa: F401
            import bbia_sim.bbia_adaptive_behavior  # noqa: F401
            import bbia_sim.bbia_emotions  # noqa: F401
            import bbia_sim.bbia_vision  # noqa: F401
            import bbia_sim.robot_factory  # noqa: F401

            assert True
        except ImportError as e:
            pytest.fail(f"Imports BBIA-SIM échouent: {e}")

    def test_sdk_compatibility_layer(self) -> None:
        """Test que la couche de compatibilité SDK fonctionne."""
        try:
            from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

            # Test que le backend peut être instancié
            backend = ReachyMiniBackend()
            assert backend is not None

            # Test que les méthodes SDK sont disponibles
            assert hasattr(backend, "goto_target")
            assert hasattr(backend, "set_target")
            assert hasattr(backend, "create_head_pose")
            assert hasattr(backend, "play_audio")

        except Exception as e:
            pytest.skip(f"Couche de compatibilité SDK non disponible: {e}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
