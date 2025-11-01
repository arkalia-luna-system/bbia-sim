"""Movement-related API routes conformes SDK officiel.

This exposes:
- goto (conforme SDK)
- play (wake_up, goto_sleep)
- stop running moves
- set_target and streaming set_target
- recorded moves datasets (conforme SDK)
"""

import asyncio
import logging
from collections.abc import Coroutine
from enum import Enum
from typing import Any
from uuid import UUID, uuid4

import numpy as np
from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from huggingface_hub.errors import RepositoryNotFoundError
from pydantic import BaseModel

from ...models import AnyPose, FullBodyTarget, MoveUUID
from ..backend_adapter import (
    BackendAdapter,
    get_backend_adapter,
    ws_get_backend_adapter,
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/move")

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


def get_backend_dependency() -> BackendAdapter:
    """Dependency pour obtenir le backend robot (via adaptateur conforme SDK)."""
    return get_backend_adapter()


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

    return MoveUUID(uuid=uuid)


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
    return [MoveUUID(uuid=uuid) for uuid in move_tasks.keys()]


@router.post("/goto")
async def goto(
    goto_req: GotoModelRequest,
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> MoveUUID:
    """Demande un mouvement vers une cible spécifique (conforme SDK)."""
    # Conforme SDK officiel: ne pas passer interpolation/method à goto_target
    # Le backend utilisera InterpolationTechnique.MIN_JERK par défaut
    return create_move_task(
        backend.goto_target(
            head=goto_req.head_pose.to_pose_array() if goto_req.head_pose else None,
            antennas=np.array(goto_req.antennas) if goto_req.antennas else None,
            duration=goto_req.duration,
        )
    )


@router.post("/play/wake_up")
async def play_wake_up(
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> MoveUUID:
    """Demande au robot de se réveiller (conforme SDK)."""
    return create_move_task(backend.wake_up())


@router.post("/play/goto_sleep")
async def play_goto_sleep(
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> MoveUUID:
    """Demande au robot de se mettre en veille (conforme SDK)."""
    return create_move_task(backend.goto_sleep())


@router.get("/recorded-move-datasets/list/{dataset_name:path}")
async def list_recorded_move_dataset(dataset_name: str) -> list[str]:
    """Liste les mouvements enregistrés disponibles dans un dataset (conforme SDK)."""
    try:
        from reachy_mini.motion.recorded_move import RecordedMoves

        moves = RecordedMoves(dataset_name)
        moves_list = moves.list_moves()
        return moves_list if isinstance(moves_list, list) else list(moves_list)
    except ImportError:
        logger.warning("reachy_mini.motion.recorded_move non disponible")
        raise HTTPException(
            status_code=501,
            detail="RecordedMoves non disponible - SDK officiel requis",
        ) from None
    except RepositoryNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e


@router.post("/play/recorded-move-dataset/{dataset_name:path}/{move_name}")
async def play_recorded_move_dataset(
    dataset_name: str,
    move_name: str,
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> MoveUUID:
    """Demande au robot de jouer un mouvement enregistré depuis un dataset (conforme SDK officiel)."""
    try:
        from reachy_mini.motion.recorded_move import RecordedMoves

        recorded_moves = RecordedMoves(dataset_name)
        move = recorded_moves.get(move_name)

        # Conforme SDK officiel : play_move est async, créer coroutine
        async def play_recorded_move_coro() -> None:
            """Coroutine pour jouer un mouvement enregistré (conforme SDK)."""
            await backend.play_move(move)

        return create_move_task(play_recorded_move_coro())
    except ImportError:
        logger.warning("reachy_mini.motion.recorded_move non disponible")
        raise HTTPException(
            status_code=501,
            detail="RecordedMoves non disponible - SDK officiel requis",
        ) from None
    except RepositoryNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e


@router.post("/stop")
async def stop_move(uuid: MoveUUID) -> dict[str, str]:
    """Arrête une tâche de mouvement en cours."""
    return await stop_move_task(uuid.uuid)


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
async def set_target(
    target: FullBodyTarget,
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> dict[str, str]:
    """Définit un target directement (sans mouvement) - conforme SDK."""
    # Conforme SDK officiel: utiliser to_pose_array() directement
    backend.set_target(
        head=(
            target.target_head_pose.to_pose_array() if target.target_head_pose else None
        ),
        antennas=np.array(target.target_antennas) if target.target_antennas else None,
    )
    return {"status": "ok"}


@router.websocket("/ws/set_target")
async def ws_set_target(
    websocket: WebSocket,
    backend: BackendAdapter = Depends(ws_get_backend_adapter),
) -> None:
    """WebSocket pour streamer les appels set_target - conforme SDK."""
    import json

    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            try:
                target = FullBodyTarget.model_validate_json(data)
                # Convertir AnyPose en array numpy
                head_pose_array = None
                if target.target_head_pose:
                    if hasattr(target.target_head_pose, "to_pose_array"):
                        head_pose_array = target.target_head_pose.to_pose_array()
                    elif isinstance(target.target_head_pose, dict | BaseModel):
                        # Fallback: essayer de créer depuis les champs
                        from ...models import Matrix4x4Pose, XYZRPYPose

                        pose_dict = (
                            target.target_head_pose
                            if isinstance(target.target_head_pose, dict)
                            else target.target_head_pose.model_dump()
                        )
                        if "m" in pose_dict:
                            pose_obj: AnyPose = Matrix4x4Pose.model_validate(pose_dict)
                        else:
                            pose_obj = XYZRPYPose.model_validate(pose_dict)
                        head_pose_array = pose_obj.to_pose_array()
                backend.set_target(
                    head=head_pose_array,
                    antennas=(
                        np.array(target.target_antennas)
                        if target.target_antennas
                        else None
                    ),
                )
                await websocket.send_text(json.dumps({"status": "ok"}))
            except Exception as e:
                await websocket.send_text(
                    json.dumps({"status": "error", "detail": str(e)})
                )
    except WebSocketDisconnect:
        pass
