#!/usr/bin/env python3
"""
🧪 TESTS TIMESTEP ADAPTATIF
Tests pour garantir le bon fonctionnement du timestep adaptatif selon complexité scène.
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock

import mujoco
import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.adaptive_timestep import (
    DEFAULT_TIMESTEP,
    MAX_TIMESTEP,
    MIN_TIMESTEP,
    calculate_adaptive_timestep,
    calculate_scene_complexity,
    get_timestep_info,
)


class TestAdaptiveTimestep:
    """Tests pour le timestep adaptatif."""

    @pytest.fixture
    def simple_model(self):
        """Modèle MuJoCo simple (peu de joints/bodies)."""
        model = MagicMock(spec=mujoco.MjModel)
        model.njnt = 5
        model.nbody = 10
        model.ngeom = 15
        model.nu = 3
        model.nv = 5
        model.opt = MagicMock()
        model.opt.timestep = 0.01
        return model

    @pytest.fixture
    def complex_model(self):
        """Modèle MuJoCo complexe (beaucoup de joints/bodies)."""
        model = MagicMock(spec=mujoco.MjModel)
        model.njnt = 50
        model.nbody = 100
        model.ngeom = 150
        model.nu = 30
        model.nv = 50
        model.opt = MagicMock()
        model.opt.timestep = 0.01
        return model

    def test_calculate_scene_complexity_simple(self, simple_model):
        """Test que la complexité est faible pour une scène simple."""
        complexity = calculate_scene_complexity(simple_model)
        assert 0.0 <= complexity <= 1.0
        assert complexity < 0.5  # Scène simple devrait avoir complexité < 0.5
        print(f"✅ Complexité scène simple: {complexity:.2f}")

    def test_calculate_scene_complexity_complex(self, complex_model):
        """Test que la complexité est élevée pour une scène complexe."""
        complexity = calculate_scene_complexity(complex_model)
        assert 0.0 <= complexity <= 1.0
        assert complexity > 0.5  # Scène complexe devrait avoir complexité > 0.5
        print(f"✅ Complexité scène complexe: {complexity:.2f}")

    def test_calculate_adaptive_timestep_simple(self, simple_model):
        """Test que le timestep est petit pour une scène simple."""
        timestep = calculate_adaptive_timestep(simple_model)
        assert MIN_TIMESTEP <= timestep <= MAX_TIMESTEP
        # Scène simple → timestep petit (fréquence élevée)
        assert timestep < DEFAULT_TIMESTEP
        print(f"✅ Timestep scène simple: {timestep:.4f}s ({1.0/timestep:.1f} Hz)")

    def test_calculate_adaptive_timestep_complex(self, complex_model):
        """Test que le timestep est grand pour une scène complexe."""
        timestep = calculate_adaptive_timestep(complex_model)
        assert MIN_TIMESTEP <= timestep <= MAX_TIMESTEP
        # Scène complexe → timestep grand (fréquence faible)
        assert timestep > DEFAULT_TIMESTEP
        print(f"✅ Timestep scène complexe: {timestep:.4f}s ({1.0/timestep:.1f} Hz)")

    def test_adaptive_timestep_range(self, simple_model, complex_model):
        """Test que le timestep adaptatif reste dans les limites."""
        timestep_simple = calculate_adaptive_timestep(simple_model)
        timestep_complex = calculate_adaptive_timestep(complex_model)

        assert MIN_TIMESTEP <= timestep_simple <= MAX_TIMESTEP
        assert MIN_TIMESTEP <= timestep_complex <= MAX_TIMESTEP
        assert timestep_simple < timestep_complex  # Simple < Complexe

        print(
            f"✅ Timestep range: {timestep_simple:.4f}s - {timestep_complex:.4f}s"
        )

    def test_get_timestep_info(self, simple_model):
        """Test que get_timestep_info retourne les bonnes informations."""
        info = get_timestep_info(simple_model)

        assert "current_timestep" in info
        assert "current_frequency_hz" in info
        assert "complexity_score" in info
        assert "recommended_timestep" in info
        assert "recommended_frequency_hz" in info
        assert "min_timestep" in info
        assert "max_timestep" in info
        assert "scene_stats" in info

        assert info["min_timestep"] == MIN_TIMESTEP
        assert info["max_timestep"] == MAX_TIMESTEP
        assert "joints" in info["scene_stats"]
        assert "bodies" in info["scene_stats"]

        print("✅ get_timestep_info retourne toutes les informations.")

    def test_adaptive_timestep_custom_limits(self, simple_model):
        """Test que le timestep adaptatif respecte les limites personnalisées."""
        custom_min = 0.001
        custom_max = 0.05

        timestep = calculate_adaptive_timestep(
            simple_model, min_timestep=custom_min, max_timestep=custom_max
        )

        assert custom_min <= timestep <= custom_max
        print(f"✅ Timestep avec limites personnalisées: {timestep:.4f}s")

    def test_complexity_normalization(self, simple_model):
        """Test que la complexité est toujours normalisée entre 0.0 et 1.0."""
        # Tester avec différents modèles
        test_cases = [
            (5, 10, 15, 3, 5),  # Très simple
            (25, 50, 75, 15, 25),  # Moyen
            (50, 100, 150, 30, 50),  # Complexe
            (100, 200, 300, 60, 100),  # Très complexe
        ]

        for njnt, nbody, ngeom, nu, nv in test_cases:
            model = MagicMock(spec=mujoco.MjModel)
            model.njnt = njnt
            model.nbody = nbody
            model.ngeom = ngeom
            model.nu = nu
            model.nv = nv

            complexity = calculate_scene_complexity(model)
            assert 0.0 <= complexity <= 1.0, f"Complexité hors limites: {complexity}"

        print("✅ Normalisation complexité fonctionne pour tous les cas.")

    def test_timestep_frequency_relationship(self, simple_model, complex_model):
        """Test que la relation timestep ↔ fréquence est correcte."""
        timestep_simple = calculate_adaptive_timestep(simple_model)
        timestep_complex = calculate_adaptive_timestep(complex_model)

        freq_simple = 1.0 / timestep_simple
        freq_complex = 1.0 / timestep_complex

        # Scène simple → timestep petit → fréquence élevée
        assert freq_simple > freq_complex

        # Vérifier que les fréquences sont raisonnables
        assert 50 <= freq_simple <= 200  # 50-200 Hz pour scène simple
        assert 50 <= freq_complex <= 200  # 50-200 Hz pour scène complexe

        print(
            f"✅ Fréquences: simple={freq_simple:.1f}Hz, complexe={freq_complex:.1f}Hz"
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])

