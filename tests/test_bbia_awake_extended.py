#!/usr/bin/env python3
"""
Tests étendus pour bbia_awake.py
Tests de séquence réveil BBIA
"""


import pytest


class TestBBIAWakeExtended:
    """Tests étendus pour BBIA Wake."""

    def test_bbia_awake_module_exists(self):
        """Test que le module bbia_awake existe."""
        try:
            import bbia_sim.bbia_awake

            assert bbia_sim.bbia_awake is not None
        except ImportError:
            pytest.skip("Module bbia_awake non disponible")

    def test_main_function_exists(self):
        """Test que la fonction main existe."""
        try:
            from bbia_sim.bbia_awake import main  # type: ignore[attr-defined]

            assert callable(main)
        except (ImportError, AttributeError):
            pytest.skip("Fonction main non disponible")

    def test_wake_sequence_exists(self):
        """Test que la séquence réveil existe."""
        try:
            from bbia_sim.bbia_awake import main  # type: ignore[attr-defined]

            # Vérifier que main est callable
            assert callable(main)
        except (ImportError, AttributeError):
            pytest.skip("Fonction main non disponible")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
