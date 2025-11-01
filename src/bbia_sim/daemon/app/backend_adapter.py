"""Adaptateur Backend pour compatibilité SDK officiel.

Cet adaptateur permet d'utiliser RobotAPI de BBIA comme Backend du SDK officiel.
"""

import logging
from typing import Any

import numpy as np
import numpy.typing as npt
from fastapi import HTTPException

from ...robot_api import RobotAPI
from ...robot_factory import RobotFactory

logger = logging.getLogger(__name__)


class BackendAdapter:
    """Adaptateur qui convertit RobotAPI en interface Backend SDK."""

    def __init__(self, robot: RobotAPI | None = None) -> None:
        """Initialise l'adaptateur."""
        self._robot = robot or self._create_backend()
        self._connected = False

    def _create_backend(self) -> RobotAPI:
        """Crée un backend RobotAPI."""
        backend = RobotFactory.create_backend("mujoco")
        if backend is None:
            backend = RobotFactory.create_backend("reachy_mini")
        if backend is None:
            raise HTTPException(
                status_code=503, detail="Aucun backend robot disponible"
            )
        return backend

    def connect_if_needed(self) -> None:
        """Connecte le backend si nécessaire."""
        if not self._connected:
            self._robot.connect()
            self._connected = True

    def get_present_head_pose(self) -> npt.NDArray[np.float64]:
        """Récupère la pose actuelle de la tête (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "get_current_head_pose"):
            pose = self._robot.get_current_head_pose()
            if isinstance(pose, np.ndarray):
                return pose.astype(np.float64)
            return np.eye(4, dtype=np.float64)

        # Fallback
        return np.eye(4, dtype=np.float64)

    def get_present_body_yaw(self) -> float:
        """Récupère le yaw actuel du corps (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "get_current_body_yaw"):
            return float(self._robot.get_current_body_yaw())
        elif hasattr(self._robot, "get_present_body_yaw"):
            return float(self._robot.get_present_body_yaw())

        return 0.0

    def get_present_antenna_joint_positions(self) -> npt.NDArray[np.float64]:
        """Récupère les positions actuelles des antennes (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "get_present_antenna_joint_positions"):
            positions = self._robot.get_present_antenna_joint_positions()
            if isinstance(positions, list | tuple) and len(positions) >= 2:
                return np.array(
                    [float(positions[0]), float(positions[1])], dtype=np.float64
                )

        return np.array([0.0, 0.0], dtype=np.float64)

    def get_present_head_joint_positions(self) -> npt.NDArray[np.float64] | None:
        """Récupère les positions actuelles des joints de la tête (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "get_current_joint_positions"):
            try:
                head_pos, _ = self._robot.get_current_joint_positions()
                if isinstance(head_pos, list | tuple):
                    # Convertir en array numpy de 7 éléments [yaw, stewart_1-6]
                    arr = np.array([float(v) for v in head_pos[:7]], dtype=np.float64)
                    if len(arr) == 7:
                        return arr
            except Exception as e:
                logger.warning(f"Erreur get_present_head_joint_positions: {e}")

        return None

    def get_present_passive_joint_positions(self) -> dict[str, float] | None:
        """Récupère les positions des joints passifs (conforme SDK)."""
        # Les joints passifs ne sont généralement pas contrôlables
        return None

    def get_motor_control_mode(self) -> Any:
        """Récupère le mode de contrôle moteur (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "get_motor_control_mode"):
            mode = self._robot.get_motor_control_mode()
            return mode

        # Créer un mock MotorControlMode simple
        class SimpleMotorControlMode:
            value = "enabled"

        return SimpleMotorControlMode()

    @property
    def target_head_pose(self) -> npt.NDArray[np.float64] | None:
        """Récupère la pose cible de la tête (conforme SDK)."""
        # RobotAPI n'a pas cette propriété directement
        # Retourner None car non disponible
        return None

    @property
    def target_body_yaw(self) -> float | None:
        """Récupère le yaw cible du corps (conforme SDK)."""
        # RobotAPI n'a pas cette propriété directement
        return None

    @property
    def target_head_joint_positions(self) -> npt.NDArray[np.float64] | None:
        """Récupère les joints cibles de la tête (conforme SDK)."""
        return None

    @property
    def target_antenna_joint_positions(self) -> npt.NDArray[np.float64] | None:
        """Récupère les positions cibles des antennes (conforme SDK)."""
        return None

    async def goto_target(
        self,
        head: npt.NDArray[np.float64] | None = None,
        antennas: npt.NDArray[np.float64] | None = None,
        duration: float = 0.5,
        method: str = "minjerk",
        body_yaw: float | None = None,
    ) -> None:
        """Va vers une cible (conforme SDK - async)."""
        self.connect_if_needed()

        if hasattr(self._robot, "goto_target"):
            # goto_target dans ReachyMiniBackend est sync, on l'appelle comme async
            # en utilisant asyncio pour éviter de bloquer
            import asyncio

            def sync_goto() -> None:
                self._robot.goto_target(
                    head=head,
                    antennas=antennas,
                    duration=duration,
                    method=method,
                    body_yaw=body_yaw or 0.0,
                )

            # Exécuter dans un thread pool pour ne pas bloquer
            await asyncio.to_thread(sync_goto)

    async def wake_up(self) -> None:
        """Réveille le robot (conforme SDK - async)."""
        self.connect_if_needed()

        if hasattr(self._robot, "wake_up"):
            if hasattr(self._robot.wake_up, "__await__"):
                await self._robot.wake_up()
            else:
                self._robot.wake_up()

    async def goto_sleep(self) -> None:
        """Met le robot en veille (conforme SDK - async)."""
        self.connect_if_needed()

        if hasattr(self._robot, "goto_sleep"):
            if hasattr(self._robot.goto_sleep, "__await__"):
                await self._robot.goto_sleep()
            else:
                self._robot.goto_sleep()

    def set_target(
        self,
        head: npt.NDArray[np.float64] | None = None,
        antennas: npt.NDArray[np.float64] | None = None,
        body_yaw: float | None = None,
    ) -> None:
        """Définit une cible (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "set_target"):
            self._robot.set_target(
                head=head, antennas=antennas, body_yaw=body_yaw or 0.0
            )

    def play_move(self, move: object) -> None:
        """Joue un mouvement enregistré (conforme SDK officiel).

        Args:
            move: Objet Move du SDK reachy_mini.motion.move
        """
        self.connect_if_needed()

        if hasattr(self._robot, "play_move"):
            # Appel direct si disponible
            self._robot.play_move(move, play_frequency=100.0, initial_goto_duration=0.0)
        elif hasattr(self._robot, "async_play_move"):
            # Fallback sur async_play_move si play_move n'existe pas
            import asyncio

            async def play_async() -> None:
                await asyncio.to_thread(
                    self._robot.async_play_move,
                    move,
                    play_frequency=100.0,
                    initial_goto_duration=0.0,
                )

            # Exécuter de manière synchrone dans un thread
            asyncio.run(play_async())
        else:
            logger.warning("play_move non disponible sur ce backend")


def get_backend_adapter() -> BackendAdapter:
    """Dependency FastAPI pour obtenir un BackendAdapter."""
    return BackendAdapter()


def ws_get_backend_adapter(websocket: Any) -> BackendAdapter:
    """Dependency FastAPI WebSocket pour obtenir un BackendAdapter."""
    return BackendAdapter()
