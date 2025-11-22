"""Adaptateur Backend pour compatibilité SDK officiel.

Cet adaptateur permet d'utiliser RobotAPI de BBIA comme Backend du SDK officiel.
"""

import logging
from typing import Any

import numpy as np
import numpy.typing as npt
from fastapi import HTTPException, WebSocket  # type: ignore[import-untyped]

from bbia_sim.robot_api import RobotAPI
from bbia_sim.robot_factory import RobotFactory
from bbia_sim.utils.types import RobotStatus

logger = logging.getLogger(__name__)


class BackendAdapter:
    """Adaptateur qui convertit RobotAPI en interface Backend SDK (conforme Backend officiel)."""

    def __init__(self, robot: RobotAPI | None = None) -> None:
        """Initialise l'adaptateur."""
        self._robot = robot or self._create_backend()
        self._connected = False

        # Initialiser attributs target (conforme SDK - attributs directs, pas propriétés)
        self.target_head_pose: npt.NDArray[np.float64] | None = None
        self.target_body_yaw: float | None = None
        self.target_head_joint_positions: npt.NDArray[np.float64] | None = None
        self.target_antenna_joint_positions: (
            npt.NDArray[np.float64] | list[float] | None
        ) = None
        self.ik_required: bool = False  # Flag pour IK computation (conforme SDK)

    def _create_backend(self) -> RobotAPI:
        """Crée un backend RobotAPI."""
        backend = RobotFactory.create_backend("mujoco")
        if backend is None:
            backend = RobotFactory.create_backend("reachy_mini")
        if backend is None:
            raise HTTPException(
                status_code=503,
                detail="Aucun backend robot disponible",
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

    def get_current_head_pose(self) -> npt.NDArray[np.float64]:
        """Alias de get_present_head_pose (conforme SDK)."""
        return self.get_present_head_pose()

    def get_present_body_yaw(self) -> float:
        """Récupère le yaw actuel du corps (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "get_current_body_yaw"):
            return float(self._robot.get_current_body_yaw())
        if hasattr(self._robot, "get_present_body_yaw"):
            return float(self._robot.get_present_body_yaw())

        return 0.0

    def get_present_antenna_joint_positions(self) -> npt.NDArray[np.float64]:
        """Récupère les positions actuelles des antennes (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "get_present_antenna_joint_positions"):
            positions = self._robot.get_present_antenna_joint_positions()
            if isinstance(positions, list | tuple) and len(positions) >= 2:
                return np.array(
                    [float(positions[0]), float(positions[1])],
                    dtype=np.float64,
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
                logger.warning("Erreur get_present_head_joint_positions: %s", e)

        return None

    def get_present_passive_joint_positions(self) -> dict[str, float] | None:
        """Récupère les positions des joints passifs (conforme SDK)."""
        # Les joints passifs ne sont généralement pas contrôlables
        return None

    def get_motor_control_mode(self) -> Any:
        """Récupère le mode de contrôle moteur (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "get_motor_control_mode"):
            return self._robot.get_motor_control_mode()

        # Créer un mock MotorControlMode simple
        class SimpleMotorControlMode:
            value = "enabled"

        return SimpleMotorControlMode()

    def set_motor_control_mode(self, mode: Any) -> None:
        """Définit le mode de contrôle moteur (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "set_motor_control_mode"):
            self._robot.set_motor_control_mode(mode)
        elif hasattr(self._robot, "robot") and self._robot.robot is not None:
            # Utiliser les méthodes SDK si disponibles
            mode_str = mode.value if hasattr(mode, "value") else str(mode)
            if mode_str == "enabled" and hasattr(self._robot.robot, "enable_motors"):
                self._robot.robot.enable_motors()
            elif mode_str == "disabled" and hasattr(
                self._robot.robot,
                "disable_motors",
            ):
                self._robot.robot.disable_motors()
            elif mode_str == "gravity_compensation" and hasattr(
                self._robot.robot,
                "enable_gravity_compensation",
            ):
                self._robot.robot.enable_gravity_compensation()
        else:
            logger.debug("Mode simulation: moteurs en mode %s", mode)

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
                if hasattr(self._robot, "goto_target"):
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

    def set_target_head_pose(self, pose: npt.NDArray[np.float64]) -> None:
        """Définit la pose cible de la tête (conforme SDK)."""
        self.connect_if_needed()
        self.target_head_pose = pose.copy() if isinstance(pose, np.ndarray) else pose
        self.ik_required = True  # Conforme SDK

        if hasattr(self._robot, "set_target_head_pose"):
            self._robot.set_target_head_pose(pose)
        elif hasattr(self._robot, "set_target"):
            self._robot.set_target(head=pose)

    def set_target_body_yaw(self, body_yaw: float) -> None:
        """Définit le yaw cible du corps (conforme SDK)."""
        self.connect_if_needed()
        self.target_body_yaw = body_yaw
        self.ik_required = True  # Conforme SDK

        if hasattr(self._robot, "set_target_body_yaw"):
            self._robot.set_target_body_yaw(body_yaw)
        elif hasattr(self._robot, "set_target"):
            self._robot.set_target(body_yaw=body_yaw)

    def set_target_head_joint_positions(
        self,
        positions: npt.NDArray[np.float64] | None,
    ) -> None:
        """Définit les positions des joints de la tête (conforme SDK)."""
        self.connect_if_needed()
        if positions is not None:
            self.target_head_joint_positions = (
                positions.copy() if isinstance(positions, np.ndarray) else positions
            )
        self.ik_required = False  # Conforme SDK (joint space = pas d'IK)

        if hasattr(self._robot, "set_target_head_joint_positions"):
            self._robot.set_target_head_joint_positions(positions)
        elif hasattr(self._robot, "set_joint_pos"):
            # Fallback: définir chaque joint individuellement
            if positions is not None and len(positions) >= 7:
                joint_names = [
                    "yaw_body",
                    "stewart_1",
                    "stewart_2",
                    "stewart_3",
                    "stewart_4",
                    "stewart_5",
                    "stewart_6",
                ]
                for i, joint_name in enumerate(joint_names):
                    if i < len(positions):
                        self._robot.set_joint_pos(joint_name, float(positions[i]))

    def set_target_antenna_joint_positions(
        self,
        positions: npt.NDArray[np.float64] | list[float],
    ) -> None:
        """Définit les positions des antennes (conforme SDK)."""
        self.connect_if_needed()
        self.target_antenna_joint_positions = (
            positions.copy() if isinstance(positions, np.ndarray) else positions
        )

        if hasattr(self._robot, "set_target_antenna_joint_positions"):
            self._robot.set_target_antenna_joint_positions(positions)
        elif hasattr(self._robot, "set_target"):
            self._robot.set_target(antennas=positions)

    def set_target(
        self,
        head: npt.NDArray[np.float64] | None = None,
        antennas: npt.NDArray[np.float64] | None = None,
        body_yaw: float | None = None,
    ) -> None:
        """Définit une cible (conforme SDK - utilise les méthodes individuelles)."""
        self.connect_if_needed()

        # Utiliser les méthodes individuelles (conforme SDK)
        if head is not None:
            self.set_target_head_pose(head)
        if body_yaw is not None:
            self.set_target_body_yaw(body_yaw)
        if antennas is not None:
            self.set_target_antenna_joint_positions(antennas)

        # Fallback sur set_target si méthodes individuelles non disponibles
        if hasattr(self._robot, "set_target") and not (
            hasattr(self._robot, "set_target_head_pose")
            or hasattr(self._robot, "set_target_body_yaw")
            or hasattr(self._robot, "set_target_antenna_joint_positions")
        ):
            self._robot.set_target(
                head=head,
                antennas=antennas,
                body_yaw=body_yaw or 0.0,
            )

    async def goto_joint_positions(
        self,
        head_joint_positions: list[float] | None = None,
        antennas_joint_positions: list[float] | None = None,
        duration: float = 0.5,
        method: str = "minjerk",
    ) -> None:
        """Va vers des positions de joints avec interpolation (conforme SDK - async).

        Args:
            head_joint_positions: Positions joints tête [yaw, stewart_1-6] (7 éléments)
            antennas_joint_positions: Positions antennes [right, left] (2 éléments)
            duration: Durée du mouvement en secondes
            method: Méthode d'interpolation ("linear", "minjerk", "ease", "cartoon")

        """
        if duration <= 0.0:
            msg = "Duration must be positive and non-zero. Use set_target() for immediate position setting."
            raise ValueError(
                msg,
            )

        self.connect_if_needed()

        # Obtenir positions actuelles
        start_head = self.get_present_head_joint_positions()
        start_antennas = self.get_present_antenna_joint_positions()

        if start_head is None:
            start_head = np.array([0.0] * 7, dtype=np.float64)
        if start_antennas is None:
            start_antennas = np.array([0.0, 0.0], dtype=np.float64)

        # Positions cibles
        target_head = (
            np.array(head_joint_positions, dtype=np.float64)
            if head_joint_positions is not None
            else start_head.copy()
        )
        target_antennas = (
            np.array(antennas_joint_positions, dtype=np.float64)
            if antennas_joint_positions is not None
            else start_antennas.copy()
        )

        # Interpolation (conforme SDK - utiliser time_trajectory si disponible)
        if hasattr(self._robot, "goto_joint_positions"):
            import asyncio

            if hasattr(self._robot.goto_joint_positions, "__await__"):
                await self._robot.goto_joint_positions(
                    head_joint_positions=head_joint_positions,
                    antennas_joint_positions=antennas_joint_positions,
                    duration=duration,
                    method=method,
                )
            else:
                await asyncio.to_thread(
                    self._robot.goto_joint_positions,
                    head_joint_positions,
                    antennas_joint_positions,
                    duration,
                    method,
                )
        else:
            # Fallback: interpolation conforme SDK (utiliser time_trajectory)
            import asyncio
            import time

            try:
                from reachy_mini.utils.interpolation import (  # type: ignore[import-untyped]
                    time_trajectory,
                )
            except ImportError:
                # Si time_trajectory non disponible, interpolation linéaire simple
                def time_trajectory(t: float, method: str = "linear") -> float:
                    return t

            t0 = time.time()
            while time.time() - t0 < duration:
                t = time.time() - t0
                interp_time = time_trajectory(t / duration, method=method)

                head_interp = start_head + (target_head - start_head) * interp_time
                antennas_interp = (
                    start_antennas + (target_antennas - start_antennas) * interp_time
                )

                self.set_target_head_joint_positions(head_interp)
                self.set_target_antenna_joint_positions(antennas_interp)

                await asyncio.sleep(0.01)

    async def play_move(self, move: object) -> None:
        """Joue un mouvement enregistré (conforme SDK officiel - async).

        Args:
            move: Objet Move du SDK reachy_mini.motion.move

        """
        self.connect_if_needed()

        if hasattr(self._robot, "play_move"):
            # Si play_move est async dans le SDK
            if hasattr(self._robot.play_move, "__await__"):
                await self._robot.play_move(
                    move,
                    play_frequency=100.0,
                    initial_goto_duration=0.0,
                )
            else:
                # Si sync, exécuter dans thread
                import asyncio

                await asyncio.to_thread(
                    self._robot.play_move,
                    move,
                    100.0,
                    0.0,
                )
        elif hasattr(self._robot, "async_play_move"):
            # Fallback sur async_play_move si play_move n'existe pas
            import asyncio

            await asyncio.to_thread(
                self._robot.async_play_move,
                move,
                100.0,
                0.0,
            )
        else:
            logger.warning("play_move non disponible sur ce backend")

    def get_urdf(self) -> str:
        """Récupère la représentation URDF du robot (conforme SDK)."""
        from pathlib import Path

        # Chercher URDF dans les emplacements standards
        urdf_paths = [
            Path(__file__).parent.parent.parent.parent
            / "sim"
            / "models"
            / "robot.urdf",
            Path(__file__).parent.parent.parent.parent.parent
            / "reachy_mini"
            / "src"
            / "reachy_mini"
            / "descriptions"
            / "reachy_mini"
            / "urdf"
            / "robot.urdf",
        ]

        for urdf_path in urdf_paths:
            if urdf_path.exists():
                with open(urdf_path) as f:
                    return f.read()

        logger.warning("URDF non trouvé, retourne URDF vide")
        return '<?xml version="1.0"?><robot name="reachy_mini"></robot>'

    def play_sound(self, sound_file: str) -> None:
        """Joue un fichier son (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "play_sound"):
            self._robot.play_sound(sound_file)
        elif hasattr(self._robot, "robot") and hasattr(self._robot.robot, "play_sound"):
            self._robot.robot.play_sound(sound_file)
        else:
            logger.debug("play_sound non disponible, fichier: %s", sound_file)

    def set_automatic_body_yaw(self, body_yaw: float) -> None:
        """Définit le yaw automatique du corps (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "set_automatic_body_yaw"):
            self._robot.set_automatic_body_yaw(body_yaw)
        else:
            logger.debug("set_automatic_body_yaw non disponible, yaw: %s", body_yaw)

    def set_target_head_joint_current(self, current: npt.NDArray[np.float64]) -> None:
        """Définit le courant des joints de la tête (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "set_target_head_joint_current"):
            self._robot.set_target_head_joint_current(current)
        else:
            logger.debug("set_target_head_joint_current non disponible (simulation)")

    def update_target_head_joints_from_ik(
        self,
        pose: npt.NDArray[np.float64] | None = None,
        body_yaw: float | None = None,
    ) -> None:
        """Met à jour les joints tête depuis IK (conforme SDK)."""
        self.connect_if_needed()

        if pose is None:
            pose = (
                self.target_head_pose
                if self.target_head_pose is not None
                else np.eye(4, dtype=np.float64)
            )
        if body_yaw is None:
            body_yaw = self.target_body_yaw if self.target_body_yaw is not None else 0.0

        if hasattr(self._robot, "update_target_head_joints_from_ik"):
            self._robot.update_target_head_joints_from_ik(pose, body_yaw)
        # Fallback: utiliser IK si disponible via robot (conforme SDK)
        elif hasattr(self._robot, "robot") and hasattr(
            self._robot.robot,
            "head_kinematics",
        ):
            joints = self._robot.robot.head_kinematics.ik(pose, body_yaw=body_yaw)
            if joints is None or np.any(np.isnan(joints)):
                msg = "WARNING: Collision detected or head pose not achievable!"
                raise ValueError(
                    msg,
                )
            # Conforme SDK: mettre à jour directement target_head_joint_positions
            self.target_head_joint_positions = joints
            self.ik_required = False  # Plus besoin d'IK après mise à jour directe

    def update_head_kinematics_model(
        self,
        head_joint_positions: npt.NDArray[np.float64] | None = None,
        antennas_joint_positions: npt.NDArray[np.float64] | None = None,
    ) -> None:
        """Met à jour le modèle cinématique de la tête (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "update_head_kinematics_model"):
            self._robot.update_head_kinematics_model(
                head_joint_positions=head_joint_positions,
                antennas_joint_positions=antennas_joint_positions,
            )
        else:
            # Fallback: mettre à jour via set_target si disponible
            if head_joint_positions is not None:
                self.set_target_head_joint_positions(head_joint_positions)
            if antennas_joint_positions is not None:
                self.set_target_antenna_joint_positions(antennas_joint_positions)

    def close(self) -> None:
        """Ferme le backend (conforme SDK - lifecycle)."""
        try:
            if self._robot and hasattr(self._robot, "disconnect"):
                self._robot.disconnect()
            self._connected = False
        except Exception as e:
            logger.warning("Erreur lors de la fermeture: %s", e)

    def get_status(self) -> RobotStatus | dict[str, Any]:
        """Récupère le statut du backend (conforme SDK)."""
        self.connect_if_needed()

        if hasattr(self._robot, "get_status"):
            status = self._robot.get_status()
            # Convertir en RobotStatus si nécessaire
            if isinstance(status, dict):
                return status  # type: ignore[return-value]
            return status

        # Retourner statut simple
        return {
            "connected": self._connected,
            "backend_type": type(self._robot).__name__,
        }  # type: ignore[return-value]

    # Méthodes Zenoh/Recording (non-critiques pour simulation, stubs pour conformité)
    def set_joint_positions_publisher(self, publisher: Any) -> None:
        """Définit le publisher Zenoh pour positions joints (conforme SDK - stub)."""
        logger.debug("set_joint_positions_publisher: non implémenté (simulation)")

    def set_pose_publisher(self, publisher: Any) -> None:
        """Définit le publisher Zenoh pour poses (conforme SDK - stub)."""
        logger.debug("set_pose_publisher: non implémenté (simulation)")

    def set_recording_publisher(self, publisher: Any) -> None:
        """Définit le publisher Zenoh pour enregistrement (conforme SDK - stub)."""
        logger.debug("set_recording_publisher: non implémenté (simulation)")

    def append_record(self, record: dict[str, Any]) -> None:
        """Ajoute un enregistrement (conforme SDK - stub)."""
        logger.debug("append_record: non implémenté (simulation)")

    def start_recording(self) -> None:
        """Démarre l'enregistrement (conforme SDK - stub)."""
        logger.debug("start_recording: non implémenté (simulation)")

    def stop_recording(self) -> None:
        """Arrête l'enregistrement (conforme SDK - stub)."""
        logger.debug("stop_recording: non implémenté (simulation)")

    # Lifecycle methods (non nécessaires pour adaptateur, stubs pour conformité)
    def run(self) -> None:
        """Exécute le backend (conforme SDK - stub pour adaptateur)."""
        logger.debug("run: non nécessaire pour adaptateur")

    def wrapped_run(self) -> None:
        """Exécute le backend avec gestion d'erreur (conforme SDK - stub)."""
        logger.debug("wrapped_run: non nécessaire pour adaptateur")


def get_backend_adapter() -> BackendAdapter:
    """Dependency FastAPI pour obtenir un BackendAdapter."""
    return BackendAdapter()


def ws_get_backend_adapter(websocket: WebSocket | None = None) -> BackendAdapter:
    """Dependency FastAPI WebSocket pour obtenir un BackendAdapter.

    Args:
        websocket: Connexion WebSocket (optionnel, non utilisé mais requis par FastAPI)

    Returns:
        Instance de BackendAdapter

    """
    # Note: websocket paramètre requis pour FastAPI dependency system
    # mais non utilisé car BackendAdapter ne dépend pas du WebSocket
    return BackendAdapter()
