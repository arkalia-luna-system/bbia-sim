#!/usr/bin/env python3
"""RobotFactory - Factory pour créer les backends RobotAPI."""

import logging
from typing import TYPE_CHECKING, Any

from .backends.mujoco_backend import MuJoCoBackend
from .backends.reachy_backend import ReachyBackend
from .backends.reachy_mini_backend import ReachyMiniBackend

if TYPE_CHECKING:
    from .robot_api import RobotAPI

logger = logging.getLogger(__name__)


class RobotFactory:
    """Factory pour créer les backends RobotAPI."""

    @staticmethod
    def create_backend(
        backend_type: str = "mujoco",
        **kwargs: Any,
    ) -> "RobotAPI | None":
        """Crée un backend RobotAPI.

        Args:
            backend_type: Type de backend ("mujoco", "reachy", ou "reachy_mini")
            **kwargs: Arguments spécifiques au backend
                - fast: Si True, utilise modèle simplifié (7 joints) pour tests rapides

        Returns:
            Instance du backend ou None si erreur

        """
        try:
            if backend_type.lower() == "mujoco":
                # Support mode rapide (modèle simplifié)
                if kwargs.get("fast", False):
                    model_path = kwargs.get(
                        "model_path",
                        "src/bbia_sim/sim/models/reachy_mini.xml",  # 7 joints
                    )
                else:
                    model_path = kwargs.get(
                        "model_path",
                        "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml",  # 16 joints
                    )
                return MuJoCoBackend(model_path=model_path)

            if backend_type.lower() == "reachy":
                robot_ip = kwargs.get("robot_ip", "localhost")
                robot_port = kwargs.get("robot_port", 8080)
                return ReachyBackend(robot_ip=robot_ip, robot_port=robot_port)

            if backend_type.lower() == "reachy_mini":
                # Paramètres SDK officiel ReachyMini
                # use_sim=True par défaut pour éviter timeout si pas de robot physique
                # L'utilisateur peut forcer use_sim=False pour chercher un robot réel
                return ReachyMiniBackend(
                    localhost_only=kwargs.get("localhost_only", True),
                    spawn_daemon=kwargs.get("spawn_daemon", False),
                    use_sim=kwargs.get("use_sim", True),  # Par défaut mode simulation
                    timeout=kwargs.get("timeout", 3.0),  # Timeout réduit à 3s
                    automatic_body_yaw=kwargs.get("automatic_body_yaw", False),
                    log_level=kwargs.get("log_level", "INFO"),
                    media_backend=kwargs.get("media_backend", "default"),
                )

            logger.error("Type de backend non supporté: %s", backend_type)
            return None

        except Exception:  # noqa: BLE001 - Erreur création backend
            logger.exception("Erreur création backend %s:", backend_type)
            return None

    @staticmethod
    def get_available_backends() -> list[str]:
        """Retourne la liste des backends disponibles."""
        return ["mujoco", "reachy", "reachy_mini"]

    @staticmethod
    def get_backend_info(backend_type: str) -> dict[str, Any]:
        """Retourne les informations sur un backend."""
        info = {
            "mujoco": {
                "name": "MuJoCo Simulator",
                "description": "Simulateur MuJoCo pour développement et tests",
                "supports_viewer": True,
                "supports_headless": True,
                "real_robot": False,
            },
            "reachy": {
                "name": "Reachy Real Robot",
                "description": "Robot Reachy réel (implémentation mock)",
                "supports_viewer": False,
                "supports_headless": True,
                "real_robot": True,
            },
            "reachy_mini": {
                "name": "Reachy-Mini SDK Officiel",
                "description": "Robot Reachy-Mini avec SDK officiel reachy_mini",
                "supports_viewer": False,
                "supports_headless": True,
                "real_robot": True,
            },
        }

        return info.get(backend_type.lower(), {})

    @staticmethod
    def create_robot_registry() -> dict[str, Any]:
        """Crée un registre pour gestion multi-robots (Issue #30).

        Returns:
            Dictionnaire avec informations robots disponibles

        Note:
            Infrastructure pour support multi-robots futur.
            Utilise BBIA_HOSTNAME et BBIA_PORT pour identification.

        """
        import os

        from .global_config import GlobalConfig

        robot_id = os.environ.get("BBIA_ROBOT_ID", "default")
        hostname = GlobalConfig.HOSTNAME
        port = GlobalConfig.DEFAULT_PORT

        return {
            "robot_id": robot_id,
            "hostname": hostname,
            "port": port,
            "backends_available": RobotFactory.get_available_backends(),
        }

    @staticmethod
    def create_multi_backend(
        backends: list[str] | None = None, **kwargs: Any
    ) -> dict[str, "RobotAPI | None"]:
        """Crée plusieurs backends simultanément pour support sim/robot réel.

        Args:
            backends: Liste des types de backends à créer. Si None, crée tous disponibles
            **kwargs: Arguments spécifiques aux backends

        Returns:
            Dictionnaire {backend_type: backend_instance}

        Note:
            Permet d'avoir sim ET robot réel actifs simultanément.
            Routing selon commande via API.

        """
        if backends is None:
            backends = RobotFactory.get_available_backends()

        multi_backends: dict[str, RobotAPI | None] = {}

        for backend_type in backends:
            try:
                backend = RobotFactory.create_backend(backend_type, **kwargs)
                if backend:
                    multi_backends[backend_type] = backend
                    logger.info("Backend %s créé avec succès", backend_type)
            except Exception as e:
                logger.warning("Impossible de créer backend %s: %s", backend_type, e)
                multi_backends[backend_type] = None

        return multi_backends
