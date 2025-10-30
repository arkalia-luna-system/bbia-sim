#!/usr/bin/env python3
"""Placeholder test: latence caméra SDK + FPS 30s (réel).

Skip par défaut. Activer avec REACHY_REAL=1 quand le robot est connecté.
"""

import os

import pytest


@pytest.mark.unit
def test_camera_sdk_latency_and_fps_placeholder() -> None:
    if os.environ.get("REACHY_REAL", "0") != "1":
        pytest.skip("Test réel désactivé: définir REACHY_REAL=1 pour activer")

    # À implémenter au branchement robot: récupération frames via robot.media.camera
    # et mesure latence/frame + FPS sur 30s.
    assert True
