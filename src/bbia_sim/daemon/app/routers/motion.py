"""Router pour les endpoints de mouvement du robot."""

import logging
from datetime import datetime
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

logger = logging.getLogger(__name__)

router = APIRouter()


class Pose(BaseModel):
    """Modèle pour une position."""

    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


class JointPosition(BaseModel):
    """Modèle pour une position d'articulation."""

    joint_name: str
    position: float


class MotionCommand(BaseModel):
    """Modèle pour une commande de mouvement."""

    command: str
    parameters: dict[str, Any] = {}


@router.post("/goto_pose")
async def goto_pose(pose: Pose) -> dict[str, Any]:
    """Déplace le robot vers une position spécifique.

    Args:
        pose: Position cible

    Returns:
        Statut du mouvement
    """
    logger.info(f"Mouvement vers la position : {pose.dict()}")

    # Simulation du mouvement
    estimated_time = 2.5

    return {
        "status": "moving",
        "target_pose": pose.dict(),
        "estimated_time": estimated_time,
        "timestamp": datetime.now().isoformat(),
    }


@router.post("/home")
async def go_home() -> dict[str, Any]:
    """Retour à la position d'origine.

    Returns:
        Statut du retour à la position d'origine
    """
    logger.info("Retour à la position d'origine")

    return {
        "status": "returning_home",
        "estimated_time": 3.0,
        "timestamp": datetime.now().isoformat(),
    }


@router.post("/joints")
async def set_joint_positions(positions: list[JointPosition]) -> dict[str, Any]:
    """Définit les positions des articulations.

    Args:
        positions: Liste des positions d'articulations

    Returns:
        Statut de la commande
    """
    logger.info(
        f"Définition des positions d'articulations : {len(positions)} articulations"
    )

    # Validation des articulations
    valid_joints = [
        "right_shoulder_pitch",
        "right_elbow_pitch",
        "right_wrist_pitch",
        "left_shoulder_pitch",
        "left_elbow_pitch",
        "left_wrist_pitch",
        "head_yaw",
        "head_pitch",
    ]

    for pos in positions:
        if pos.joint_name not in valid_joints:
            raise HTTPException(
                status_code=400, detail=f"Articulation invalide : {pos.joint_name}"
            )

    return {
        "status": "moving",
        "joints": [pos.dict() for pos in positions],
        "estimated_time": 1.5,
        "timestamp": datetime.now().isoformat(),
    }


@router.post("/gripper/{side}")
async def control_gripper(side: str, action: str) -> dict[str, Any]:
    """Contrôle une pince.

    Args:
        side: Côté de la pince (left/right)
        action: Action à effectuer (open/close/grip)

    Returns:
        Statut de la commande
    """
    if side not in ["left", "right"]:
        raise HTTPException(
            status_code=400, detail="Côté invalide. Utilisez 'left' ou 'right'"
        )

    if action not in ["open", "close", "grip"]:
        raise HTTPException(
            status_code=400,
            detail="Action invalide. Utilisez 'open', 'close' ou 'grip'",
        )

    logger.info(f"Contrôle de la pince {side} : {action}")

    return {
        "status": "executing",
        "gripper": side,
        "action": action,
        "estimated_time": 0.5,
        "timestamp": datetime.now().isoformat(),
    }


@router.post("/head")
async def control_head(yaw: float = 0.0, pitch: float = 0.0) -> dict[str, Any]:
    """Contrôle la tête du robot.

    Args:
        yaw: Rotation horizontale (-π/2 à π/2)
        pitch: Rotation verticale (-0.5 à 0.5)

    Returns:
        Statut de la commande
    """
    # Validation des limites
    if not -1.57 <= yaw <= 1.57:
        raise HTTPException(
            status_code=400, detail="Yaw doit être entre -1.57 et 1.57 radians"
        )

    if not -0.5 <= pitch <= 0.5:
        raise HTTPException(
            status_code=400, detail="Pitch doit être entre -0.5 et 0.5 radians"
        )

    logger.info(f"Contrôle de la tête : yaw={yaw}, pitch={pitch}")

    return {
        "status": "moving",
        "target": {"yaw": yaw, "pitch": pitch},
        "estimated_time": 1.0,
        "timestamp": datetime.now().isoformat(),
    }


@router.post("/stop")
async def stop_motion() -> dict[str, Any]:
    """Arrête tous les mouvements.

    Returns:
        Statut de l'arrêt
    """
    logger.info("Arrêt de tous les mouvements")

    return {
        "status": "stopped",
        "message": "Tous les mouvements ont été arrêtés",
        "timestamp": datetime.now().isoformat(),
    }


@router.get("/status")
async def get_motion_status() -> dict[str, Any]:
    """Récupère le statut des mouvements.

    Returns:
        Statut des mouvements
    """
    logger.info("Récupération du statut des mouvements")

    return {
        "status": "idle",
        "current_action": None,
        "queue": [],
        "timestamp": datetime.now().isoformat(),
    }


@router.post("/custom")
async def execute_custom_command(command: MotionCommand) -> dict[str, Any]:
    """Exécute une commande de mouvement personnalisée.

    Args:
        command: Commande personnalisée

    Returns:
        Statut de l'exécution
    """
    logger.info(f"Exécution de la commande personnalisée : {command.command}")

    # Simulation de l'exécution
    return {
        "status": "executing",
        "command": command.command,
        "parameters": command.parameters,
        "estimated_time": 2.0,
        "timestamp": datetime.now().isoformat(),
    }
