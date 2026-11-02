#!/usr/bin/env python3
"""Tests étendus pour bbia_awake.py — séquence réveil BBIA."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import bbia_sim.bbia_awake  # noqa: F401
from bbia_sim.bbia_awake import start_bbia_sim


class TestBBIAWakeExtended:
    """Tests étendus pour BBIA Wake."""

    def test_bbia_awake_module_exists(self) -> None:
        """Le module bbia_awake est importable."""
        assert bbia_sim.bbia_awake is not None

    def test_start_bbia_sim_callable(self) -> None:
        """start_bbia_sim existe et est callable (pas de skip mockable)."""
        assert callable(start_bbia_sim)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
