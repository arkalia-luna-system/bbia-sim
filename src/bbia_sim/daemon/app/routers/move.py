"""Movement-related API routes conformes SDK officiel.

This exposes:
- goto (conforme SDK)
- play (wake_up, goto_sleep)
- stop running moves
- set_target and streaming set_target
"""

import asyncio
import logging
from collections.abc import Coroutine
from enum import Enum
from typing import Any
from uuid import UUID, uuid4

import numpy as np
from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from ....robot_factory import RobotFactory
from ...models import AnyPose, FullBodyTarget, MoveUUID, XYZRPYPose

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/move")

# État global pour les tâches de mouvement
move_tasks: dict[UUID, asyncio.Task[None]] = {}
move_listeners: list[WebSocket] = []


class InterpolationMode(str, Enum):
    """Mode d'interpolation pour les mouvements."""

    LINEAR = "linear"
    MINJERK = "minjerk"
    EASE = "ease"
    CARTOON = "cartoon"


class GotoModelRequest(BaseModel):
    """Modèle de requête pour l'endpoint goto (conforme SDK)."""

    head_pose: AnyPose | None = None
    antennas: tuple[float, float] | None = None
    duration: float
    interpolation: InterpolationMode = InterpolationMode.MINJERK

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "head_pose": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                    },
                    "antennas": [0.0, 0.0],
                    "duration": 2.0,
                    "interpolation": "minjerk",
                },
                {
                    "antennas": [0.0, 0.0],
                    "duration": 1.0,
                    "interpolation": "linear",
                },
            ],
        }
    }


def get_backend_dependency():
    """Dependency pour obtenir le backend robot."""
    backend = RobotFactory.create_backend("mujoco")
    if backend is None:
        backend = RobotFactory.create_backend("reachy_mini")
    if backend is None:
        raise HTTPException(status_code=503, detail="Aucun backend robot disponible")
    return backend


def create_move_task(coro: Coroutine[Any, Any, None]) -> MoveUUID:
    """Crée une nouvelle tâche de mouvement avec UUID (conforme SDK)."""
    uuid = uuid4()

    async def notify_listeners(message: str, details: str = "") -> None:
        """Notifie les listeners WebSocket."""
        disconnected = []
        for ws in move_listeners:
            try:
                await ws.send_json(
                    {
                        "type": message,
                        "uuid": str(uuid),
                        "details": details,
                    }
                )
            except (RuntimeError, WebSocketDisconnect):
                disconnected.append(ws)

        # Nettoyer les connexions fermées
        for ws in disconnected:
            if ws in move_listeners:
                move_listeners.remove(ws)

    async def wrap_coro() -> None:
        """Wrapper pour la coroutine avec notifications."""
        try:
            await notify_listeners("move_started")
            await coro
            await notify_listeners("move_completed")
        except Exception as e:
            await notify_listeners("move_failed", details=str(e))
            logger.error(f"Erreur dans la tâche de mouvement {uuid}: {e}")
        except asyncio.CancelledError:
            await notify_listeners("move_cancelled")
        finally:
            move_tasks.pop(uuid, None)

    task = asyncio.create_task(wrap_coro())
    move_tasks[uuid] = task

    return MoveUUID(uuid=str(uuid))


async def stop_move_task(uuid: UUID) -> dict[str, str]:
    """Arrête une tâche de mouvement en cours."""
    if uuid not in move_tasks:
        raise KeyError(f"Running move with UUID {uuid} not found")

    task = move_tasks.pop(uuid, None)
    assert task is not None

    if task:
        if task.cancel():
            try:
                await task
            except asyncio.CancelledError:
                pass

    return {"message": f"Stopped move with UUID: {uuid}"}


@router.get("/running")
async def get_running_moves() -> list[MoveUUID]:
    """Récupère la liste des mouvements en cours."""
    return [MoveUUID(uuid=str(uuid)) for uuid in move_tasks.keys()]


@router.post("/goto")
async def goto(goto_req: GotoModelRequest) -> MoveUUID:
    """Demande un mouvement vers une cible spécifique (conforme SDK)."""

    async def goto_coro() -> None:
        """Coroutine pour exécuter le mouvement."""
        robot = get_backend_dependency()
        try:
            robot.connect()

            # Convertir AnyPose en matrice 4x4 si présent
            head_pose = None
            if goto_req.head_pose:
                if hasattr(goto_req.head_pose, "to_pose_array"):
                    head_pose = goto_req.head_pose.to_pose_array()
                elif isinstance(goto_req.head_pose, dict):
                    pose_data = goto_req.head_pose
                    if "m" in pose_data:
                        # Matrix4x4Pose
                        from ...models import Matrix4x4Pose

                        head_pose = Matrix4x4Pose(
                            m=tuple(pose_data["m"])
                        ).to_pose_array()
                    else:
                        # XYZRPYPose
                        head_pose = XYZRPYPose(
                            x=pose_data.get("x", 0.0),
                            y=pose_data.get("y", 0.0),
                            z=pose_data.get("z", 0.0),
                            roll=pose_data.get("roll", 0.0),
                            pitch=pose_data.get("pitch", 0.0),
                            yaw=pose_data.get("yaw", 0.0),
                        ).to_pose_array()

            # Convertir antennas en numpy array
            antennas = None
            if goto_req.antennas:
                antennas = np.array(goto_req.antennas)

            # Utiliser goto_target avec interpolation
            if hasattr(robot, "goto_target"):
                interpolation_map = {
                    "linear": "linear",
                    "minjerk": "minjerk",
                    "ease": "ease_in_out",
                    "cartoon": "cartoon",
                }
                method = interpolation_map.get(goto_req.interpolation.value, "minjerk")
                # goto_target peut être sync ou async
                if hasattr(robot.goto_target, "__await__"):
                    await robot.goto_target(
                        head=head_pose,
                        antennas=antennas,
                        duration=goto_req.duration,
                        method=method,
                    )
                else:
                    robot.goto_target(
                        head=head_pose,
                        antennas=antennas,
                        duration=goto_req.duration,
                        method=method,
                    )
            else:
                logger.warning("goto_target non disponible")
        finally:
            robot.disconnect()

    return create_move_task(goto_coro())


@router.post("/play/wake_up")
async def play_wake_up() -> MoveUUID:
    """Demande au robot de se réveiller (conforme SDK)."""

    async def wake_up_coro() -> None:
        robot = get_backend_dependency()
        try:
            robot.connect()
            if hasattr(robot, "wake_up"):
                if hasattr(robot.wake_up, "__await__"):
                    await robot.wake_up()
                else:
                    robot.wake_up()
        finally:
            robot.disconnect()

    return create_move_task(wake_up_coro())


@router.post("/play/goto_sleep")
async def play_goto_sleep() -> MoveUUID:
    """Demande au robot de se mettre en veille (conforme SDK)."""

    async def goto_sleep_coro() -> None:
        robot = get_backend_dependency()
        try:
            robot.connect()
            if hasattr(robot, "goto_sleep"):
                if hasattr(robot.goto_sleep, "__await__"):
                    await robot.goto_sleep()
                else:
                    robot.goto_sleep()
        finally:
            robot.disconnect()

    return create_move_task(goto_sleep_coro())


@router.post("/stop")
async def stop_move(uuid: MoveUUID) -> dict[str, str]:
    """Arrête une tâche de mouvement en cours."""
    try:
        move_uuid = UUID(uuid.uuid)
        return await stop_move_task(move_uuid)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=f"Invalid UUID format: {uuid.uuid}") from e
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e


@router.websocket("/ws/updates")
async def ws_move_updates(websocket: WebSocket) -> None:
    """WebSocket pour streamer les mises à jour de mouvements."""
    await websocket.accept()
    try:
        move_listeners.append(websocket)
        while True:
            _ = await websocket.receive_text()
    except WebSocketDisconnect:
        if websocket in move_listeners:
            move_listeners.remove(websocket)


@router.post("/set_target")
async def set_target(target: FullBodyTarget) -> dict[str, str]:
    """Définit un target directement (sans mouvement)."""
    robot = get_backend_dependency()
    try:
        robot.connect()

        # Convertir AnyPose si présent
        head_pose = None
        if target.target_head_pose:
            if hasattr(target.target_head_pose, "to_pose_array"):
                head_pose = target.target_head_pose.to_pose_array()
            elif isinstance(target.target_head_pose, dict):
                pose_data = target.target_head_pose
                if "m" in pose_data:
                    from ...models import Matrix4x4Pose

                    head_pose = Matrix4x4Pose(m=tuple(pose_data["m"])).to_pose_array()
                else:
                    head_pose = XYZRPYPose(
                        x=pose_data.get("x", 0.0),
                        y=pose_data.get("y", 0.0),
                        z=pose_data.get("z", 0.0),
                        roll=pose_data.get("roll", 0.0),
                        pitch=pose_data.get("pitch", 0.0),
                        yaw=pose_data.get("yaw", 0.0),
                    ).to_pose_array()

        # Convertir antennas
        antennas = None
        if target.target_antennas:
            antennas = np.array(target.target_antennas)

        if hasattr(robot, "set_target"):
            robot.set_target(head=head_pose, antennas=antennas)

    finally:
        robot.disconnect()

    return {"status": "ok"}


@router.websocket("/ws/set_target")
async def ws_set_target(websocket: WebSocket) -> None:
    """WebSocket pour streamer les appels set_target."""
    import json

    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            try:
                target_data = json.loads(data)
                target = FullBodyTarget.model_validate(target_data)
                await set_target(target)
                await websocket.send_json({"status": "ok"})
            except Exception as e:
                await websocket.send_json({"status": "error", "detail": str(e)})
    except WebSocketDisconnect:
        pass
