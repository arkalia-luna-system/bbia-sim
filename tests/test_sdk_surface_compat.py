#!/usr/bin/env python3
"""
Test de surface API Reachy Mini: échoue si la compatibilité de signatures se brise.

Le test est tolérant si le SDK n'est pas installé (skip), mais strict sinon.
"""

import importlib
import inspect

import pytest

REQUIRED_METHODS = {
    "look_at_world": ("x", "y", "z", "duration", "perform_movement"),
    "look_at_image": ("u", "v", "duration", "perform_movement"),
    "goto_target": ("head", "antennas", "duration", "method", "body_yaw"),
    "get_current_joint_positions": (),
    "set_target_head_pose": ("pose",),
    "set_target_body_yaw": ("body_yaw",),
}


@pytest.mark.unit
def test_sdk_surface_api_signatures():
    sdk = importlib.util.find_spec("reachy_mini")
    if sdk is None:
        pytest.skip("SDK reachy_mini non installé")

    reachy_mini = importlib.import_module("reachy_mini")
    ReachyMini = getattr(reachy_mini, "ReachyMini", None)
    assert ReachyMini is not None, "Classe ReachyMini absente du SDK"

    obj = ReachyMini if inspect.isclass(ReachyMini) else reachy_mini

    for method_name, required_params in REQUIRED_METHODS.items():
        assert hasattr(obj, method_name), f"Méthode {method_name} absente"
        func = getattr(obj, method_name)
        sig = inspect.signature(func)
        # Vérifier que tous les paramètres attendus existent dans l'ordre relatif
        params = [
            p.name
            for p in sig.parameters.values()
            if p.kind in (p.POSITIONAL_OR_KEYWORD, p.POSITIONAL_ONLY)
        ]
        # Autoriser self implicite
        if params and params[0] == "self":
            params = params[1:]
        for rp in required_params:
            assert (
                rp in params
            ), f"Paramètre requis {rp} manquant dans {method_name}({params})"
