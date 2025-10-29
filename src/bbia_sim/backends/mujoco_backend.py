#!/usr/bin/env python3
"""
MuJoCoBackend - Implémentation MuJoCo de RobotAPI
Backend pour simulation MuJoCo
"""

import logging
import time
from pathlib import Path
from typing import Any, Optional

import mujoco
import mujoco.viewer

from ..robot_api import RobotAPI

logger = logging.getLogger(__name__)


class MuJoCoBackend(RobotAPI):
    """Backend MuJoCo pour RobotAPI."""

    def __init__(
        self, model_path: str = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    ) -> None:
        super().__init__()
        self.model_path = Path(model_path)
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.viewer: Optional[mujoco.viewer.MjViewer] = None
        self.joint_name_to_id: dict[str, int] = {}
        self.step_count = 0
        self.start_time: float = 0.0

    def connect(self) -> bool:
        """Connecte au simulateur MuJoCo."""
        try:
            if not self.model_path.exists():
                logger.error(f"Modèle MuJoCo introuvable: {self.model_path}")
                return False

            self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
            self.data = mujoco.MjData(self.model)

            # Construire le mapping joint name → id
            for i in range(self.model.njnt):
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                if name:
                    self.joint_name_to_id[name] = i
                    # Charger les limites du joint
                    joint_range = self.model.jnt_range[i]
                    self.joint_limits[name] = (joint_range[0], joint_range[1])

            self.is_connected = True
            self.start_time = time.time()
            logger.info(f"MuJoCo connecté: {self.model.njnt} joints détectés")
            return True

        except Exception as e:
            logger.error(f"Erreur connexion MuJoCo: {e}")
            return False

    def disconnect(self) -> bool:
        """Déconnecte du simulateur MuJoCo."""
        try:
            if self.viewer:
                self.viewer.close()
                self.viewer = None

            self.model = None
            self.data = None
            self.is_connected = False
            logger.info("MuJoCo déconnecté")
            return True

        except Exception as e:
            logger.error(f"Erreur déconnexion MuJoCo: {e}")
            return False

    def get_available_joints(self) -> list[str]:
        """Retourne la liste des joints disponibles."""
        if not self.is_connected:
            return []

        return list(self.joint_name_to_id.keys())

    def set_joint_pos(self, joint_name: str, position: float) -> bool:
        """Définit la position d'un joint."""
        if not self.is_connected:
            logger.error("MuJoCo non connecté")
            return False

        if joint_name not in self.joint_name_to_id:
            logger.error(f"Joint introuvable: {joint_name}")
            return False

        # Validation et clamp via RobotAPI
        is_valid, clamped_position = self._validate_joint_pos(joint_name, position)
        if not is_valid:
            return False

        # Appliquer la position
        joint_id = self.joint_name_to_id[joint_name]
        if self.data is not None:
            self.data.qpos[joint_id] = clamped_position

        logger.debug(f"Joint {joint_name} → {clamped_position:.3f} rad")
        return True

    def get_joint_pos(self, joint_name: str) -> Optional[float]:
        """Récupère la position actuelle d'un joint."""
        if not self.is_connected:
            return None

        if joint_name not in self.joint_name_to_id:
            return None

        joint_id = self.joint_name_to_id[joint_name]
        if self.data is not None:
            return float(self.data.qpos[joint_id])
        return None

    def step(self) -> bool:
        """Effectue un pas de simulation."""
        if not self.is_connected:
            return False

        try:
            mujoco.mj_step(self.model, self.data)
            self.step_count += 1
            return True
        except Exception as e:
            logger.error(f"Erreur step MuJoCo: {e}")
            return False

    def emergency_stop(self) -> bool:
        """Arrêt d'urgence pour simulation MuJoCo."""
        if not self.is_connected:
            logger.warning("Simulation non connectée - emergency_stop ignoré")
            return False

        try:
            # Mettre toutes les positions à 0
            if self.data:
                self.data.ctrl[:] = 0.0
                self.data.qvel[:] = 0.0
            self.is_connected = False
            logger.critical("🔴 ARRÊT D'URGENCE SIMULATION ACTIVÉ")
            return True
        except Exception as e:
            logger.error(f"Erreur emergency_stop: {e}")
            return False

    def launch_viewer(self, passive: bool = True) -> bool:
        """Lance le viewer MuJoCo."""
        if not self.is_connected:
            logger.error("MuJoCo non connecté")
            return False

        try:
            if passive:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            else:
                self.viewer = mujoco.viewer.launch(self.model, self.data)

            logger.info("Viewer MuJoCo lancé")
            return True
        except Exception as e:
            logger.error(f"Erreur lancement viewer: {e}")
            return False

    def sync_viewer(self) -> bool:
        """Synchronise le viewer."""
        if not self.viewer:
            return False

        try:
            self.viewer.sync()
            return True
        except Exception as e:
            logger.error(f"Erreur sync viewer: {e}")
            return False

    def is_viewer_running(self) -> bool:
        """Vérifie si le viewer est actif."""
        if not self.viewer:
            return False

        try:
            return self.viewer.is_running()  # type: ignore
        except Exception:
            return False

    def get_telemetry(self) -> dict[str, Any]:
        """Retourne les données de télémétrie."""
        if not self.is_connected:
            return {}

        current_time = time.time()
        elapsed_time = current_time - self.start_time if self.start_time else 0

        return {
            "step_count": self.step_count,
            "elapsed_time": elapsed_time,
            "steps_per_second": (
                self.step_count / elapsed_time if elapsed_time > 0 else 0
            ),
            "average_step_time": (
                elapsed_time / self.step_count if self.step_count > 0 else 0
            ),
            "current_qpos": self.data.qpos.copy() if self.data else [],
            "model_path": str(self.model_path),
        }
