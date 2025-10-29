#!/usr/bin/env python3
"""
RobotFactory - Factory pour créer les backends RobotAPI
"""

import logging
from typing import Optional

from .backends.mujoco_backend import MuJoCoBackend
from .backends.reachy_backend import ReachyBackend
from .backends.reachy_mini_backend import ReachyMiniBackend
from .robot_api import RobotAPI

logger = logging.getLogger(__name__)


class RobotFactory:
    """Factory pour créer les backends RobotAPI."""

    @staticmethod
    def create_backend(backend_type: str = "mujoco", **kwargs) -> Optional[RobotAPI]:
        """Crée un backend RobotAPI.

        Args:
            backend_type: Type de backend ("mujoco", "reachy", ou "reachy_mini")
            **kwargs: Arguments spécifiques au backend

        Returns:
            Instance du backend ou None si erreur
        """
        try:
            if backend_type.lower() == "mujoco":
                model_path = kwargs.get(
                    "model_path",
                    "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml",
                )
                return MuJoCoBackend(model_path=model_path)

            elif backend_type.lower() == "reachy":
                robot_ip = kwargs.get("robot_ip", "localhost")
                robot_port = kwargs.get("robot_port", 8080)
                return ReachyBackend(robot_ip=robot_ip, robot_port=robot_port)

            elif backend_type.lower() == "reachy_mini":
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

            else:
                logger.error(f"Type de backend non supporté: {backend_type}")
                return None

        except Exception as e:
            logger.error(f"Erreur création backend {backend_type}: {e}")
            return None

    @staticmethod
    def get_available_backends() -> list[str]:
        """Retourne la liste des backends disponibles."""
        return ["mujoco", "reachy", "reachy_mini"]

    @staticmethod
    def get_backend_info(backend_type: str) -> dict:
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
