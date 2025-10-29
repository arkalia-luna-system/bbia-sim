#!/usr/bin/env python3
"""Placeholder test: watchdog timeout réel (à activer avec robot).

Skip par défaut. Activer avec REACHY_REAL=1 quand le robot est connecté.
"""

import os

import pytest


@pytest.mark.unit
def test_watchdog_timeout_real_placeholder() -> None:
    if os.environ.get("REACHY_REAL", "0") != "1":
        pytest.skip("Test réel désactivé: définir REACHY_REAL=1 pour activer")
    # À implémenter au branchement robot: mesurer p50/p95 timeout réel
    assert True
