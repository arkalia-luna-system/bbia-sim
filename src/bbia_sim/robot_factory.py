#!/usr/bin/env python3
"""
RobotFactory - Factory pour créer les backends RobotAPI
"""

import logging
from typing import Optional

from .backends.mujoco_backend import MuJoCoBackend
from .backends.reachy_backend import ReachyBackend
from .robot_api import RobotAPI

logger = logging.getLogger(__name__)


class RobotFactory:
    """Factory pour créer les backends RobotAPI."""

    @staticmethod
    def create_backend(backend_type: str = "mujoco", **kwargs) -> Optional[RobotAPI]:
        """Crée un backend RobotAPI.

        Args:
            backend_type: Type de backend ("mujoco" ou "reachy")
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

            else:
                logger.error(f"Type de backend non supporté: {backend_type}")
                return None

        except Exception as e:
            logger.error(f"Erreur création backend {backend_type}: {e}")
            return None

    @staticmethod
    def get_available_backends() -> list[str]:
        """Retourne la liste des backends disponibles."""
        return ["mujoco", "reachy"]

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
        }

        return info.get(backend_type.lower(), {})
