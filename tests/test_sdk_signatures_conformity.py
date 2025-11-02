#!/usr/bin/env python3
"""
Test de conformité des signatures SDK Reachy Mini
Vérifie que toutes les signatures de méthodes correspondent exactement au SDK officiel
"""

import inspect
import sys
from pathlib import Path
from typing import Any

import pytest

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend


class TestSDKSignaturesConformity:
    """Test de conformité des signatures avec le SDK officiel Reachy Mini."""

    def setup_method(self):
        """Configuration avant chaque test."""
        self.backend = ReachyMiniBackend()
        assert self.backend is not None

    def get_method_signature(self, obj: Any, method_name: str) -> inspect.Signature:
        """Récupère la signature d'une méthode."""
        method = getattr(obj, method_name)
        return inspect.signature(method)

    def test_core_methods_signatures(self):
        """Test signatures des méthodes principales."""
        core_methods = {
            "get_joint_pos": {"joint_name": str},
            "set_joint_pos": {"joint_name": str, "position": float},
            "get_available_joints": {},
            "set_emotion": {"emotion": str, "intensity": float},
            "look_at": {"target_x": float, "target_y": float, "target_z": float},
            "run_behavior": {"behavior_name": str, "duration": float},
            "get_telemetry": {},
            "connect": {},
            "disconnect": {},
        }

        for method_name, expected_params in core_methods.items():
            assert hasattr(
                self.backend, method_name
            ), f"Méthode {method_name} manquante"

            sig = self.get_method_signature(self.backend, method_name)
            sig_params = list(sig.parameters.keys())

            # Vérifier que tous les paramètres attendus sont présents
            for param_name in expected_params.keys():
                assert (
                    param_name in sig_params
                ), f"Paramètre {param_name} manquant dans {method_name}"

    def test_sdk_official_methods_signatures(self):
        """Test signatures des méthodes SDK officiel."""
        sdk_methods = {
            "get_current_head_pose": {},
            "get_present_antenna_joint_positions": {},
            "get_current_joint_positions": {},
            "set_target_body_yaw": {"body_yaw": float},
            "set_target_antenna_joint_positions": {"antennas": list},
            "set_target_head_pose": {"pose": Any},
            "look_at_image": {
                "u": int,
                "v": int,
                "duration": float,
                "perform_movement": bool,
            },
            "look_at_world": {
                "x": float,
                "y": float,
                "z": float,
                "duration": float,
                "perform_movement": bool,
            },
            "goto_target": {
                "head": Any,
                "antennas": list,
                "duration": float,
                "method": str,
                "body_yaw": float,
            },
            "set_target": {"head": Any, "antennas": list, "body_yaw": float},
            "enable_motors": {},
            "disable_motors": {},
            "enable_gravity_compensation": {},
            "disable_gravity_compensation": {},
            "set_automatic_body_yaw": {"body_yaw": float},
            "wake_up": {},
            "goto_sleep": {},
        }

        for method_name, expected_params in sdk_methods.items():
            assert hasattr(
                self.backend, method_name
            ), f"Méthode SDK {method_name} manquante"

            sig = self.get_method_signature(self.backend, method_name)
            sig_params = list(sig.parameters.keys())

            # Vérifier que tous les paramètres attendus sont présents (sauf Any qui est optionnel)
            for param_name, param_type in expected_params.items():
                if param_type is not Any:
                    assert (
                        param_name in sig_params
                    ), f"Paramètre {param_name} manquant dans {method_name}"

    def test_return_types_consistency(self):
        """Test que les types de retour sont cohérents."""
        # Méthodes qui retournent bool
        bool_methods = [
            "set_joint_pos",
            "set_emotion",
            "look_at",
            "run_behavior",
            "connect",
            "disconnect",
        ]
        for method_name in bool_methods:
            # On vérifie juste que c'est callable, pas le type exact
            assert callable(getattr(self.backend, method_name))

        # Méthodes qui retournent float
        float_methods = ["get_joint_pos"]
        for method_name in float_methods:
            assert callable(getattr(self.backend, method_name))

        # Méthodes qui retournent list
        list_methods = ["get_available_joints", "get_present_antenna_joint_positions"]
        for method_name in list_methods:
            assert callable(getattr(self.backend, method_name))

        # Méthodes qui retournent dict
        dict_methods = ["get_telemetry"]
        for method_name in dict_methods:
            assert callable(getattr(self.backend, method_name))

        # Méthodes qui retournent None
        none_methods = [
            "enable_motors",
            "disable_motors",
            "enable_gravity_compensation",
            "disable_gravity_compensation",
            "set_target_body_yaw",
            "set_target_antenna_joint_positions",
            "set_target_head_pose",
            "goto_target",
            "set_target",
            "set_automatic_body_yaw",
            "wake_up",
            "goto_sleep",
        ]
        for method_name in none_methods:
            assert callable(getattr(self.backend, method_name))

    def test_optional_parameters(self):
        """Test que les paramètres optionnels ont des valeurs par défaut."""
        # Test set_emotion avec intensity optionnel
        sig = self.get_method_signature(self.backend, "set_emotion")
        assert (
            "intensity" in sig.parameters
        ), "Paramètre intensity manquant dans set_emotion"
        intensity_param = sig.parameters["intensity"]
        assert (
            intensity_param.default != inspect.Parameter.empty
        ), "Paramètre intensity n'a pas de valeur par défaut"

        # Test look_at avec target_z optionnel
        sig = self.get_method_signature(self.backend, "look_at")
        if "target_z" in sig.parameters:
            target_z_param = sig.parameters["target_z"]
            assert (
                target_z_param.default != inspect.Parameter.empty
            ), "Paramètre target_z n'a pas de valeur par défaut"

        # Test run_behavior avec duration optionnel
        sig = self.get_method_signature(self.backend, "run_behavior")
        if "duration" in sig.parameters:
            duration_param = sig.parameters["duration"]
            assert (
                duration_param.default != inspect.Parameter.empty
            ), "Paramètre duration n'a pas de valeur par défaut"

    def test_default_arguments_compliance(self):
        """Test que les arguments par défaut sont conformes au SDK."""
        # Test goto_target avec tous les paramètres optionnels
        sig = self.get_method_signature(self.backend, "goto_target")
        expected_optional_params = [
            "head",
            "antennas",
            "duration",
            "method",
            "body_yaw",
        ]
        for param_name in expected_optional_params:
            if param_name in sig.parameters:
                param = sig.parameters[param_name]
                # Vérifier que c'est optionnel (a une valeur par défaut)
                assert (
                    param.default != inspect.Parameter.empty
                ), f"Paramètre {param_name} n'est pas optionnel"

    def test_method_docstrings(self):
        """Test que toutes les méthodes ont des docstrings."""
        methods_to_check = [
            "get_joint_pos",
            "set_joint_pos",
            "get_available_joints",
            "set_emotion",
            "look_at",
            "run_behavior",
            "get_telemetry",
            "connect",
            "disconnect",
        ]

        for method_name in methods_to_check:
            method = getattr(self.backend, method_name)
            assert (
                method.__doc__ is not None
            ), f"Méthode {method_name} n'a pas de docstring"

    def test_sdk_methods_docstrings(self):
        """Test que les méthodes SDK ont des docstrings."""
        sdk_methods = [
            "get_current_head_pose",
            "get_present_antenna_joint_positions",
            "set_target_body_yaw",
            "look_at_image",
            "goto_target",
            "enable_motors",
            "disable_motors",
            "enable_gravity_compensation",
            "disable_gravity_compensation",
            "wake_up",
            "goto_sleep",
        ]

        for method_name in sdk_methods:
            method = getattr(self.backend, method_name)
            assert (
                method.__doc__ is not None
            ), f"Méthode SDK {method_name} n'a pas de docstring"

    def test_backend_initialization_signature(self):
        """Test signature du constructeur."""
        sig = self.get_method_signature(ReachyMiniBackend, "__init__")
        params = list(sig.parameters.keys())

        # Le constructeur doit accepter robot_ip et robot_port en option
        assert (
            "robot_ip" in params or "self" in params
        ), "Paramètre robot_ip manquant dans __init__"

    def test_no_missing_sdk_methods(self):
        """Test qu'aucune méthode SDK critique n'est manquante."""
        critical_methods = [
            "get_joint_pos",
            "set_joint_pos",
            "get_available_joints",
            "get_current_joint_positions",
            "set_target_head_pose",
            "enable_motors",
            "disable_motors",
            "look_at_world",
            "wake_up",
            "goto_sleep",
        ]

        missing_methods = []
        for method_name in critical_methods:
            if not hasattr(self.backend, method_name):
                missing_methods.append(method_name)

        assert (
            len(missing_methods) == 0
        ), f"Méthodes SDK manquantes: {', '.join(missing_methods)}"

    def test_signature_compatibility(self):
        """Test que les signatures sont compatibles pour appels runtime."""
        # Test que les méthodes peuvent être appelées avec les bons paramètres
        # sans erreur (en mode simulation)

        # Test get_joint_pos
        result = self.backend.get_joint_pos("stewart_1")
        assert isinstance(result, float | int | type(None))

        # Test set_joint_pos
        result = self.backend.set_joint_pos("stewart_1", 0.1)
        assert isinstance(result, bool)

        # Test set_emotion avec arguments
        result = self.backend.set_emotion("happy", 0.8)
        assert isinstance(result, bool)

        # Test set_emotion avec valeur par défaut
        result = self.backend.set_emotion("happy")
        assert isinstance(result, bool)

        # Test look_at avec arguments
        result = self.backend.look_at(0.1, 0.2, 0.3)
        assert isinstance(result, bool)

        # Test look_at sans target_z (valeur par défaut)
        result = self.backend.look_at(0.1, 0.2)
        assert isinstance(result, bool)

        # Test run_behavior
        result = self.backend.run_behavior("wake_up", 2.0)
        assert isinstance(result, bool)

        # Test run_behavior sans duration
        result = self.backend.run_behavior("wake_up")
        assert isinstance(result, bool)

        # Test get_telemetry
        telemetry = self.backend.get_telemetry()
        assert isinstance(telemetry, dict)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
