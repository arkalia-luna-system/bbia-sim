"""Router pour les endpoints de mouvement du robot."""

import logging
from datetime import datetime
from typing import Any

from fastapi import APIRouter, HTTPException

from ....sim.joints import clamp_joint_angle, validate_joint_name
from ...models import HeadControl, JointPosition, MotionCommand, Pose
from ...simulation_service import simulation_service

logger = logging.getLogger(__name__)

router = APIRouter()


# Les modèles sont maintenant importés depuis models.py


@router.post("/goto_pose")
async def goto_pose(pose: Pose) -> dict[str, Any]:
    """Déplace le robot vers une position spécifique.

    Args:
        pose: Position cible

    Returns:
        Statut du mouvement
    """
    logger.info(f"Mouvement vers la position : {pose.model_dump()}")

    # Simulation du mouvement
    estimated_time = 2.5

    return {
        "status": "moving",
        "target_pose": pose.model_dump(),
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

    # Validation des noms de joints avec notre système centralisé
    for pos in positions:
        if not validate_joint_name(pos.joint_name):
            raise HTTPException(
                status_code=422, detail=f"Joint '{pos.joint_name}' non valide"
            )

        # Clamp des angles dans les limites
        clamped_angle = clamp_joint_angle(pos.joint_name, pos.position)
        if clamped_angle != pos.position:
            logger.warning(
                f"Angle {pos.position:.3f} clampé à {clamped_angle:.3f} "
                f"pour joint {pos.joint_name}"
            )
            pos.position = clamped_angle

    # Application des positions dans la simulation
    success_count = 0
    for pos in positions:
        if simulation_service.set_joint_position(pos.joint_name, pos.position):
            success_count += 1

    return {
        "status": "moving" if success_count > 0 else "failed",
        "joints": [pos.model_dump() for pos in positions],
        "success_count": success_count,
        "total_count": len(positions),
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
async def control_head(head_control: HeadControl) -> dict[str, Any]:
    """Contrôle la tête du robot.

    Args:
        head_control: Contrôle de la tête avec validation

    Returns:
        Statut de la commande
    """
    logger.info(
        f"Contrôle de la tête : yaw={head_control.yaw}, pitch={head_control.pitch}"
    )

    # Application des positions dans la simulation
    success_yaw = simulation_service.set_joint_position("neck_yaw", head_control.yaw)
    success_pitch = simulation_service.set_joint_position(
        "head_pitch", head_control.pitch
    )

    return {
        "status": "moving" if (success_yaw or success_pitch) else "failed",
        "target": {"yaw": head_control.yaw, "pitch": head_control.pitch},
        "success": {"yaw": success_yaw, "pitch": success_pitch},
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
