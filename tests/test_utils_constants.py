#!/usr/bin/env python3
"""Tests pour utils/constants.py - Constantes utilitaires BBIA."""

from __future__ import annotations

from bbia_sim.utils.constants import (
    ASSETS_ROOT_PATH,
    MODELS_ROOT_PATH,
    URDF_ROOT_PATH,
)


class TestUtilsConstants:
    """Tests pour les constantes utilitaires."""

    def test_urdf_root_path(self) -> None:
        """Test chemin URDF root."""
        assert isinstance(URDF_ROOT_PATH, str)
        assert "sim/models" in URDF_ROOT_PATH or "bbia_sim" in URDF_ROOT_PATH

    def test_assets_root_path(self) -> None:
        """Test chemin assets root."""
        assert isinstance(ASSETS_ROOT_PATH, str)
        assert "sim/assets" in ASSETS_ROOT_PATH or "bbia_sim" in ASSETS_ROOT_PATH

    def test_models_root_path(self) -> None:
        """Test chemin models root."""
        assert isinstance(MODELS_ROOT_PATH, str)
        assert "sim/assets" in MODELS_ROOT_PATH or "bbia_sim" in MODELS_ROOT_PATH

    def test_paths_are_strings(self) -> None:
        """Test que tous les chemins sont des strings."""
        assert isinstance(URDF_ROOT_PATH, str)
        assert isinstance(ASSETS_ROOT_PATH, str)
        assert isinstance(MODELS_ROOT_PATH, str)

    def test_paths_not_empty(self) -> None:
        """Test que les chemins ne sont pas vides."""
        assert len(URDF_ROOT_PATH) > 0
        assert len(ASSETS_ROOT_PATH) > 0
        assert len(MODELS_ROOT_PATH) > 0
