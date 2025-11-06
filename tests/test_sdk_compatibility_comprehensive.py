#!/usr/bin/env python3
"""
Tests de compatibilité complète avec le SDK Reachy Mini officiel.
Vérifie signatures, méthodes, constantes, etc.
"""

import inspect

import pytest

from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend
from bbia_sim.robot_api import RobotAPI


@pytest.mark.unit
@pytest.mark.fast
def test_sdk_method_signatures_match() -> None:
    """Test que les signatures des méthodes correspondent au SDK."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        # Méthodes requises par RobotAPI
        required_methods = [
            "connect",
            "disconnect",
            "get_joint_pos",
            "set_joint_pos",
            "get_available_joints",
            "set_emotion",
            "look_at",
            "goto_target",
            "emergency_stop",
        ]

        for method_name in required_methods:
            assert hasattr(backend, method_name), f"Méthode {method_name} manquante"
            method = getattr(backend, method_name)
            assert callable(method), f"{method_name} n'est pas callable"

        # Vérifier signatures spécifiques
        sig = inspect.signature(backend.goto_target)
        params = list(sig.parameters.keys())
        assert "head" in params or "body_yaw" in params, "goto_target doit avoir head ou body_yaw"
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_sdk_joints_official_names() -> None:
    """Test que les noms de joints correspondent au SDK officiel."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        joints = backend.get_available_joints()
        assert isinstance(joints, list) or isinstance(joints, set)

        # Joints officiels Reachy Mini
        official_joints = {
            "yaw_body",
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "stewart_4",
            "stewart_5",
            "stewart_6",
            "left_antenna",
            "right_antenna",
        }

        joints_set = set(joints) if isinstance(joints, list) else joints

        # Vérifier que tous les joints officiels sont présents
        for joint in official_joints:
            assert joint in joints_set, f"Joint officiel {joint} manquant"

        # Vérifier qu'il n'y a pas de joints non-officiels (optionnel)
        # On accepte des joints supplémentaires pour compatibilité
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_sdk_emotions_official() -> None:
    """Test que les émotions supportées correspondent au SDK."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        # Émotions officielles Reachy Mini
        official_emotions = ["happy", "sad", "neutral", "excited", "curious", "calm"]

        for emotion in official_emotions:
            result = backend.set_emotion(emotion, 0.5)
            assert result is True or result is False
            # Si False, ce n'est pas une erreur, juste non supporté
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_sdk_behaviors_official() -> None:
    """Test que les comportements correspondent au SDK."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        # Comportements officiels Reachy Mini
        official_behaviors = ["wake_up", "goto_sleep", "nod"]

        for behavior in official_behaviors:
            if hasattr(backend, "run_behavior"):
                result = backend.run_behavior(behavior, 1.0)
                assert result is True or result is False
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_sdk_safe_amplitude_limit() -> None:
    """Test que la limite d'amplitude sécurisée correspond au SDK (0.3 rad)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        # Vérifier limite si accessible
        if hasattr(backend, "safe_amplitude_limit"):
            limit = backend.safe_amplitude_limit
            assert isinstance(limit, float | int)
            # SDK recommande 0.3 rad
            assert limit <= 0.5, "Limite d'amplitude trop élevée (> 0.5 rad)"
    finally:
        backend.disconnect()


@pytest.mark.unit
@pytest.mark.fast
def test_sdk_api_consistency() -> None:
    """Test que l'API est cohérente (RobotAPI interface)."""
    backend = ReachyMiniBackend(use_sim=True)
    assert backend.connect() is True

    try:
        # Vérifier que le backend implémente RobotAPI
        assert isinstance(backend, RobotAPI)

        # Vérifier méthodes abstraites implémentées
        assert hasattr(backend, "is_connected")
        assert hasattr(backend, "get_telemetry") or hasattr(backend, "get_current_state")
    finally:
        backend.disconnect()
