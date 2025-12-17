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
from typing import Annotated, Any
from uuid import UUID, uuid4

import numpy as np
from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from huggingface_hub.errors import RepositoryNotFoundError
from pydantic import BaseModel

try:
    from reachy_mini.motion.recorded_move import RecordedMoves
except ImportError:
    RecordedMoves = None

import contextlib

from fastapi import Query

from bbia_sim.daemon.app.backend_adapter import (
    BackendAdapter,
    get_backend_adapter,
    ws_get_backend_adapter,
)
from bbia_sim.daemon.models import (
    AnyPose,
    BatchMovementRequest,
    FullBodyTarget,
    MoveUUID,
)
from bbia_sim.movement_batch_processor import get_batch_processor

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/move")

# État global pour les tâches de mouvement
move_tasks: dict[UUID, asyncio.Task[None]] = {}
# OPTIMISATION RAM: Limiter nombre de listeners WebSocket (max 20)
move_listeners: list[WebSocket] = []
_MAX_MOVE_LISTENERS = 20


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
        },
    }


def get_backend_dependency() -> BackendAdapter:
    """Dependency pour obtenir le backend robot (via adaptateur conforme SDK)."""
    return get_backend_adapter()


def get_backend_adapter_for_move(
    backend: str | None = Query(
        None,
        description="Type de backend à utiliser ('mujoco', 'reachy_mini', etc.). Si None, utilise le backend par défaut.",
    ),
) -> BackendAdapter:
    """Dependency pour obtenir un BackendAdapter pour les mouvements avec routing multi-backends.

    Args:
        backend: Type de backend à utiliser (optionnel, depuis query param)

    Returns:
        Instance de BackendAdapter configurée pour le backend spécifié
    """
    return get_backend_adapter(backend=backend)


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
                    },
                )
            except (RuntimeError, WebSocketDisconnect, Exception):
                # OPTIMISATION RAM: Capturer toutes les exceptions pour nettoyage robuste
                disconnected.append(ws)

        # OPTIMISATION RAM: Nettoyer les connexions fermées
        for ws in disconnected:
            if ws in move_listeners:
                move_listeners.remove(ws)

    async def wrap_coro() -> None:
        """Wrapper pour la coroutine avec notifications (conforme SDK)."""
        try:
            await notify_listeners("move_started")
            await coro
            await notify_listeners("move_completed")
        except Exception as e:
            await notify_listeners("move_failed", details=str(e))
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
        msg = f"Running move with UUID {uuid} not found"
        raise KeyError(msg)

    task = move_tasks.pop(uuid, None)
    if task is None:
        msg = f"Task for UUID {uuid} was None after pop operation"
        raise RuntimeError(msg)

    if task and task.cancel():
        with contextlib.suppress(asyncio.CancelledError):
            await task

    return {"message": f"Stopped move with UUID: {uuid}"}


@router.get("/running")
async def get_running_moves() -> list[MoveUUID]:
    """Récupère la liste des mouvements en cours."""
    return [MoveUUID(uuid=uuid) for uuid in move_tasks]


@router.post("/goto")
async def goto(
    goto_req: GotoModelRequest,
    backend: Annotated[BackendAdapter, Depends(get_backend_adapter_for_move)],
) -> MoveUUID:
    """Demande un mouvement vers une cible spécifique (conforme SDK)."""
    # Conforme SDK officiel: ne pas passer interpolation/method à goto_target
    # Le backend utilisera InterpolationTechnique.MIN_JERK par défaut
    return create_move_task(
        backend.goto_target(
            head=goto_req.head_pose.to_pose_array() if goto_req.head_pose else None,
            antennas=np.array(goto_req.antennas) if goto_req.antennas else None,
            duration=goto_req.duration,
        ),
    )


@router.post("/play/wake_up")
async def play_wake_up(
    backend: Annotated[BackendAdapter, Depends(get_backend_adapter_for_move)],
) -> MoveUUID:
    """Demande au robot de se réveiller (conforme SDK)."""
    return create_move_task(backend.wake_up())


@router.post("/play/goto_sleep")
async def play_goto_sleep(
    backend: Annotated[BackendAdapter, Depends(get_backend_adapter_for_move)],
) -> MoveUUID:
    """Demande au robot de se mettre en veille (conforme SDK)."""
    return create_move_task(backend.goto_sleep())


@router.get("/recorded-move-datasets/discover")
async def discover_recorded_move_datasets() -> list[str]:
    """Liste les datasets recorded moves disponibles sur HF Hub.

    Amélioration implémentée: utilise l'API Hugging Face Hub si disponible,
    avec fallback vers liste hardcodée pour robustesse.
    Mentionnée dans docs/audit/DECISION_FINAL_AMELIORATIONS.md (🟡 1).

    Returns:
        Liste de noms de datasets disponibles (format: "org/repo-name")

    """
    # Datasets connus hardcodés (toujours inclus pour compatibilité)
    known_datasets = [
        "pollen-robotics/reachy-mini-dances-library",
        "pollen-robotics/reachy-mini-emotions-library",
    ]

    # AMÉLIORATION: Récupérer depuis HF Hub API si disponible
    try:
        from huggingface_hub import HfApi

        api = HfApi()
        # Rechercher datasets avec préfixe "reachy-mini" ou "reachy_mini"
        hf_datasets = api.list_datasets(
            search="reachy-mini",
            limit=50,
        )

        # Filtrer et ajouter les datasets trouvés (éviter doublons)
        hf_dataset_ids: set[str] = set()
        for ds in hf_datasets:
            if ds.id and "reachy" in ds.id.lower():
                hf_dataset_ids.add(ds.id)

        # Merger avec liste hardcodée (éviter doublons)
        all_datasets = list(set(known_datasets) | hf_dataset_ids)
        logger.info(
            f"✅ Découverte {len(hf_dataset_ids)} datasets HF Hub "
            f"(total: {len(all_datasets)})",
        )
        return sorted(all_datasets)  # Trier pour affichage cohérent

    except ImportError:
        logger.debug(
            "huggingface_hub non disponible - utilisation liste hardcodée uniquement",
        )
    except Exception as e:
        logger.warning(
            f"⚠️ Erreur récupération datasets HF Hub (fallback hardcodé): {e}",
        )

    # Fallback: retourner liste hardcodée seulement
    return known_datasets


@router.get("/recorded-move-datasets/list/{dataset_name:path}")
async def list_recorded_move_dataset(dataset_name: str) -> list[str]:
    """Liste les mouvements enregistrés disponibles dans un dataset (conforme SDK)."""
    if RecordedMoves is None:
        raise HTTPException(
            status_code=501,
            detail="RecordedMoves non disponible - SDK officiel requis",
        )
    try:
        moves = RecordedMoves(dataset_name)
    except RepositoryNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e

    result = moves.list_moves()
    # S'assurer que le résultat est une liste de strings
    # Type: ignore car list_moves() retourne Any mais nous convertissons en list[str]
    move_list: list[str] = [str(move) for move in result] if result else []
    return move_list


@router.post("/play/recorded-move-dataset/{dataset_name:path}/{move_name}")
async def play_recorded_move_dataset(
    dataset_name: str,
    move_name: str,
    backend: Annotated[BackendAdapter, Depends(get_backend_adapter_for_move)],
) -> MoveUUID:
    """Demande au robot de jouer un mouvement enregistré depuis un dataset (conforme SDK officiel)."""
    if RecordedMoves is None:
        raise HTTPException(
            status_code=501,
            detail="RecordedMoves non disponible - SDK officiel requis",
        )
    try:
        recorded_moves = RecordedMoves(dataset_name)
    except RepositoryNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    try:
        move = recorded_moves.get(move_name)
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    return create_move_task(backend.play_move(move))


@router.post("/stop")
async def stop_move(uuid: MoveUUID) -> dict[str, str]:
    """Arrête une tâche de mouvement en cours (conforme SDK)."""
    try:
        return await stop_move_task(uuid.uuid)
    except KeyError as e:
        # Convertir KeyError en HTTPException 404 (conforme SDK)
        raise HTTPException(status_code=404, detail=str(e)) from e


@router.websocket("/ws/updates")
async def ws_move_updates(websocket: WebSocket) -> None:
    """WebSocket pour streamer les mises à jour de mouvements."""
    # OPTIMISATION RAM: Limiter nombre de connexions simultanées
    if len(move_listeners) >= _MAX_MOVE_LISTENERS:
        logger.warning(
            "Limite de listeners atteinte (%d), rejet de nouvelle connexion",
            _MAX_MOVE_LISTENERS,
        )
        await websocket.close(code=1008, reason="Too many connections")
        return

    await websocket.accept()
    try:
        move_listeners.append(websocket)
        while True:
            _ = await websocket.receive_text()
    except (WebSocketDisconnect, Exception):
        # OPTIMISATION RAM: Nettoyer dans finally pour garantir suppression
        pass
    finally:
        # OPTIMISATION RAM: Nettoyage garanti même en cas d'exception
        if websocket in move_listeners:
            move_listeners.remove(websocket)


@router.post("/set_target")
async def set_target(
    target: FullBodyTarget,
    backend: Annotated[BackendAdapter, Depends(get_backend_adapter_for_move)],
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
async def ws_set_target(websocket: WebSocket) -> None:
    """WebSocket pour streamer les appels set_target - conforme SDK."""
    import json

    await websocket.accept()
    # Créer backend adapter directement (pas de Depends pour WebSockets)
    backend = ws_get_backend_adapter(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            try:
                target = FullBodyTarget.model_validate_json(data)
                # Conforme SDK officiel: utiliser set_target directement
                await set_target(target, backend)
            except Exception as e:
                await websocket.send_text(
                    json.dumps({"status": "error", "detail": str(e)}),
                )
    except WebSocketDisconnect:
        pass


@router.post("/batch")
async def batch_movements(
    batch_req: BatchMovementRequest,
    backend: Annotated[BackendAdapter, Depends(get_backend_adapter_for_move)],
) -> dict[str, Any]:
    """Exécute plusieurs mouvements en batch (optimisation performance).

    Args:
        batch_req: Requête contenant la liste de mouvements
        backend: Backend adapter pour exécuter les mouvements

    Returns:
        Dictionnaire avec statut et UUIDs des mouvements créés
    """
    batch_processor = get_batch_processor()
    movement_uuids: list[str] = []

    for movement_data in batch_req.movements:
        # Capturer les variables dans la closure pour éviter warnings
        move_type = movement_data.get("type", "goto")
        m_data = movement_data.copy()  # Copie pour éviter référence mutée

        async def create_movement_func(
            m_type: str = move_type, m_d: dict[str, Any] = m_data
        ) -> None:
            """Crée une fonction async pour exécuter le mouvement."""
            if m_type == "goto":
                head_pose_data = m_d.get("head_pose")
                antennas_data = m_d.get("antennas")
                duration = m_d.get("duration", 1.0)

                head_pose = AnyPose(**head_pose_data) if head_pose_data else None
                antennas = tuple(antennas_data) if antennas_data else None

                await backend.goto_target(
                    head=head_pose.to_pose_array() if head_pose else None,
                    antennas=np.array(antennas) if antennas else None,
                    duration=duration,
                )
            elif m_type == "wake_up":
                await backend.wake_up()
            elif m_type == "goto_sleep":
                await backend.goto_sleep()
            elif m_type == "set_target":
                target_data = m_d.get("target", {})
                target = FullBodyTarget(**target_data)
                backend.set_target(
                    head=(
                        target.target_head_pose.to_pose_array()
                        if target.target_head_pose
                        else None
                    ),
                    antennas=(
                        np.array(target.target_antennas)
                        if target.target_antennas
                        else None
                    ),
                )
            else:
                logger.warning("Type de mouvement inconnu: %s", m_type)

        result = await batch_processor.add_movement(create_movement_func)
        movement_uuids.append(result["movement_id"])

    return {
        "status": "queued",
        "movement_count": len(movement_uuids),
        "movement_ids": movement_uuids,
        "batch_size": batch_processor.get_queue_size(),
    }
