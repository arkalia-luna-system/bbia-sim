#!/usr/bin/env python3
"""Tests complets pour robot_factory.py - Factory pour créer les backends RobotAPI."""

from __future__ import annotations

import os
from typing import Any
from unittest.mock import patch

from bbia_sim.backends.mujoco_backend import MuJoCoBackend
from bbia_sim.backends.reachy_backend import ReachyBackend
from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend
from bbia_sim.robot_factory import RobotFactory


class TestRobotFactory:
    """Tests pour RobotFactory."""

    def test_create_backend_mujoco_default(self) -> None:
        """Test création backend MuJoCo par défaut."""
        backend = RobotFactory.create_backend("mujoco")
        assert backend is not None
        assert isinstance(backend, MuJoCoBackend)

    def test_create_backend_mujoco_fast_mode(self) -> None:
        """Test création backend MuJoCo en mode rapide."""
        backend = RobotFactory.create_backend("mujoco", fast=True)
        assert backend is not None
        assert isinstance(backend, MuJoCoBackend)

    def test_create_backend_mujoco_custom_model_path(self) -> None:
        """Test création backend MuJoCo avec chemin modèle personnalisé."""
        backend = RobotFactory.create_backend(
            "mujoco",
            model_path="src/bbia_sim/sim/models/reachy_mini.xml",
        )
        assert backend is not None
        assert isinstance(backend, MuJoCoBackend)

    def test_create_backend_mujoco_fast_with_custom_path(self) -> None:
        """Test création backend MuJoCo fast avec chemin personnalisé."""
        backend = RobotFactory.create_backend(
            "mujoco",
            fast=True,
            model_path="src/bbia_sim/sim/models/reachy_mini.xml",
        )
        assert backend is not None
        assert isinstance(backend, MuJoCoBackend)

    def test_create_backend_reachy_default(self) -> None:
        """Test création backend Reachy par défaut."""
        backend = RobotFactory.create_backend("reachy")
        assert backend is not None
        assert isinstance(backend, ReachyBackend)

    def test_create_backend_reachy_with_ip_port(self) -> None:
        """Test création backend Reachy avec IP et port."""
        backend = RobotFactory.create_backend(
            "reachy",
            robot_ip="192.168.1.100",
            robot_port=9090,
        )
        assert backend is not None
        assert isinstance(backend, ReachyBackend)

    def test_create_backend_reachy_mini_default(self) -> None:
        """Test création backend ReachyMini par défaut."""
        backend = RobotFactory.create_backend("reachy_mini")
        assert backend is not None
        assert isinstance(backend, ReachyMiniBackend)

    def test_create_backend_reachy_mini_with_params(self) -> None:
        """Test création backend ReachyMini avec paramètres."""
        backend = RobotFactory.create_backend(
            "reachy_mini",
            localhost_only=False,
            spawn_daemon=True,
            use_sim=False,
            timeout=5.0,
            automatic_body_yaw=True,
            log_level="DEBUG",
            media_backend="opencv",
        )
        assert backend is not None
        assert isinstance(backend, ReachyMiniBackend)

    def test_create_backend_invalid_type(self) -> None:
        """Test création backend avec type invalide."""
        backend = RobotFactory.create_backend("invalid_backend")
        assert backend is None

    def test_create_backend_case_insensitive(self) -> None:
        """Test création backend insensible à la casse."""
        backend1 = RobotFactory.create_backend("MUJOCO")
        backend2 = RobotFactory.create_backend("Mujoco")
        backend3 = RobotFactory.create_backend("mUjOcO")

        assert backend1 is not None
        assert backend2 is not None
        assert backend3 is not None
        assert isinstance(backend1, MuJoCoBackend)
        assert isinstance(backend2, MuJoCoBackend)
        assert isinstance(backend3, MuJoCoBackend)

    def test_create_backend_exception_handling(self) -> None:
        """Test gestion exceptions lors de la création."""
        with patch(
            "bbia_sim.robot_factory.MuJoCoBackend",
            side_effect=Exception("Erreur initialisation"),
        ):
            backend = RobotFactory.create_backend("mujoco")
            assert backend is None

    def test_get_available_backends(self) -> None:
        """Test récupération backends disponibles."""
        backends = RobotFactory.get_available_backends()
        assert isinstance(backends, list)
        assert "mujoco" in backends
        assert "reachy" in backends
        assert "reachy_mini" in backends
        assert len(backends) == 3

    def test_get_backend_info_mujoco(self) -> None:
        """Test informations backend MuJoCo."""
        info = RobotFactory.get_backend_info("mujoco")
        assert isinstance(info, dict)
        assert info["name"] == "MuJoCo Simulator"
        assert info["description"] == "Simulateur MuJoCo pour développement et tests"
        assert info["supports_viewer"] is True
        assert info["supports_headless"] is True
        assert info["real_robot"] is False

    def test_get_backend_info_reachy(self) -> None:
        """Test informations backend Reachy."""
        info = RobotFactory.get_backend_info("reachy")
        assert isinstance(info, dict)
        assert info["name"] == "Reachy Real Robot"
        assert info["description"] == "Robot Reachy réel (implémentation mock)"
        assert info["supports_viewer"] is False
        assert info["supports_headless"] is True
        assert info["real_robot"] is True

    def test_get_backend_info_reachy_mini(self) -> None:
        """Test informations backend ReachyMini."""
        info = RobotFactory.get_backend_info("reachy_mini")
        assert isinstance(info, dict)
        assert info["name"] == "Reachy-Mini SDK Officiel"
        assert info["description"] == "Robot Reachy-Mini avec SDK officiel reachy_mini"
        assert info["supports_viewer"] is False
        assert info["supports_headless"] is True
        assert info["real_robot"] is True

    def test_get_backend_info_case_insensitive(self) -> None:
        """Test informations backend insensible à la casse."""
        info1 = RobotFactory.get_backend_info("MUJOCO")
        info2 = RobotFactory.get_backend_info("Mujoco")
        assert info1 == info2
        assert info1["name"] == "MuJoCo Simulator"

    def test_get_backend_info_invalid(self) -> None:
        """Test informations backend invalide."""
        info = RobotFactory.get_backend_info("invalid_backend")
        assert info == {}

    def test_create_robot_registry_default(self) -> None:
        """Test création registre robot par défaut."""
        # Sauvegarder variable d'environnement si elle existe
        old_robot_id = os.environ.get("BBIA_ROBOT_ID")

        try:
            # Supprimer variable pour test valeur par défaut
            if "BBIA_ROBOT_ID" in os.environ:
                del os.environ["BBIA_ROBOT_ID"]

            registry = RobotFactory.create_robot_registry()
            assert isinstance(registry, dict)
            assert registry["robot_id"] == "default"
            assert "hostname" in registry
            assert "port" in registry
            assert "backends_available" in registry
            assert isinstance(registry["backends_available"], list)
        finally:
            # Restaurer variable d'environnement
            if old_robot_id is not None:
                os.environ["BBIA_ROBOT_ID"] = old_robot_id

    def test_create_robot_registry_with_env(self) -> None:
        """Test création registre robot avec variable d'environnement."""
        old_robot_id = os.environ.get("BBIA_ROBOT_ID")

        try:
            os.environ["BBIA_ROBOT_ID"] = "test_robot_123"
            registry = RobotFactory.create_robot_registry()
            assert isinstance(registry, dict)
            assert registry["robot_id"] == "test_robot_123"
        finally:
            # Restaurer variable d'environnement
            if old_robot_id is not None:
                os.environ["BBIA_ROBOT_ID"] = old_robot_id
            elif "BBIA_ROBOT_ID" in os.environ:
                del os.environ["BBIA_ROBOT_ID"]

    def test_create_multi_backend_all(self) -> None:
        """Test création multi-backends (tous disponibles)."""
        multi_backends = RobotFactory.create_multi_backend()
        assert isinstance(multi_backends, dict)
        assert "mujoco" in multi_backends
        # Les autres peuvent être None si non disponibles
        assert multi_backends["mujoco"] is not None

    def test_create_multi_backend_specific(self) -> None:
        """Test création multi-backends spécifiques."""
        multi_backends = RobotFactory.create_multi_backend(backends=["mujoco"])
        assert isinstance(multi_backends, dict)
        assert "mujoco" in multi_backends
        assert len(multi_backends) == 1
        assert multi_backends["mujoco"] is not None

    def test_create_multi_backend_with_kwargs(self) -> None:
        """Test création multi-backends avec kwargs."""
        multi_backends = RobotFactory.create_multi_backend(
            backends=["mujoco"],
            fast=True,
        )
        assert isinstance(multi_backends, dict)
        assert "mujoco" in multi_backends
        assert multi_backends["mujoco"] is not None

    def test_create_multi_backend_partial_failure(self) -> None:
        """Test création multi-backends avec échec partiel."""

        # Simuler un backend qui lève une exception
        def mock_create_backend(backend_type: str, **kwargs: Any) -> Any:
            if backend_type == "mujoco":
                return MuJoCoBackend()
            # invalid lève une exception (sera catchée et ajoutée comme None)
            raise Exception("Backend invalide")

        with patch(
            "bbia_sim.robot_factory.RobotFactory.create_backend",
            side_effect=mock_create_backend,
        ):
            multi_backends = RobotFactory.create_multi_backend(
                backends=["mujoco", "invalid"],
            )
            assert isinstance(multi_backends, dict)
            assert "mujoco" in multi_backends
            assert "invalid" in multi_backends
            assert multi_backends["mujoco"] is not None
            assert multi_backends["invalid"] is None

    def test_create_multi_backend_exception_handling(self) -> None:
        """Test gestion exceptions dans create_multi_backend."""
        with patch(
            "bbia_sim.robot_factory.RobotFactory.create_backend",
            side_effect=Exception("Erreur création"),
        ):
            multi_backends = RobotFactory.create_multi_backend(backends=["mujoco"])
            assert isinstance(multi_backends, dict)
            assert "mujoco" in multi_backends
            assert multi_backends["mujoco"] is None
