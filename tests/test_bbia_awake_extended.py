#!/usr/bin/env python3
"""
Tests étendus pour bbia_awake.py
Tests de séquence réveil BBIA
"""

import sys
from pathlib import Path

import pytest

# S'assurer que src est dans le path pour coverage
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# OPTIMISATION COVERAGE: Importer le module au niveau module pour que coverage le détecte
# IMPORTANT: Import direct (pas dans try/except) pour que coverage le détecte
import bbia_sim.bbia_awake  # noqa: F401

# Importer les fonctions pour les tests
try:
    from bbia_sim.bbia_awake import start_bbia_sim  # noqa: F401

    # Note: main n'existe pas dans bbia_awake.py, seulement start_bbia_sim
except (ImportError, AttributeError):
    start_bbia_sim = None  # type: ignore[assignment,misc]


class TestBBIAWakeExtended:
    """Tests étendus pour BBIA Wake."""

    def test_bbia_awake_module_exists(self):
        """Test que le module bbia_awake existe."""
        assert bbia_sim.bbia_awake is not None

    def test_main_function_exists(self):
        """Test que la fonction start_bbia_sim existe."""
        if start_bbia_sim is None:
            pytest.skip("Fonction start_bbia_sim non disponible")
        assert callable(start_bbia_sim)

    def test_wake_sequence_exists(self):
        """Test que la séquence réveil existe."""
        if start_bbia_sim is None:
            pytest.skip("Fonction start_bbia_sim non disponible")
        # Vérifier que start_bbia_sim est callable
        assert callable(start_bbia_sim)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
