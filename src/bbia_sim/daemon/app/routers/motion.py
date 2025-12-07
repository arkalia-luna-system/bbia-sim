"""Router pour les endpoints de mouvement du robot."""

import logging
from datetime import datetime
from enum import Enum
from typing import Annotated, Any

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel

from bbia_sim.daemon.models import HeadControl, JointPosition, MotionCommand, Pose
from bbia_sim.daemon.simulation_service import simulation_service
from bbia_sim.sim.joints import clamp_joint_angle, validate_joint_name

logger = logging.getLogger(__name__)

router = APIRouter()


# Les modèles sont maintenant importés depuis models.py


class InterpolationMode(str, Enum):
    """Mode d'interpolation pour les mouvements."""

    LINEAR = "linear"
    MINJERK = "minjerk"
    EASE = "ease"
    CARTOON = "cartoon"


@router.post("/goto_pose")
async def goto_pose(
    pose: Pose,
    duration: Annotated[
        float, Query(gt=0, description="Durée du mouvement en secondes")
    ] = 2.5,
    interpolation: Annotated[
        InterpolationMode, Query(description="Mode d'interpolation")
    ] = InterpolationMode.MINJERK,
) -> dict[str, Any]:
    """Déplace le robot vers une position spécifique avec interpolation.

    Args:
        pose: Position cible
        duration: Durée du mouvement en secondes
        interpolation: Mode d'interpolation (linear, minjerk, ease, cartoon)

    Returns:
        Statut du mouvement

    """
    logger.info(
        f"Mouvement vers la position : {pose.model_dump()}, "
        f"duration={duration}, interpolation={interpolation.value}",
    )

    try:
        from bbia_sim.robot_factory import RobotFactory

        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()

            # Convertir Pose en matrice 4x4 pour goto_target
            import numpy as np
            from scipy.spatial.transform import Rotation as R

            rotation = R.from_euler("xyz", [pose.roll, pose.pitch, pose.yaw])
            pose_matrix = np.eye(4)
            pose_matrix[:3, 3] = [pose.x, pose.y, pose.z]
            pose_matrix[:3, :3] = rotation.as_matrix()

            # Utiliser goto_target avec interpolation si disponible
            if hasattr(robot, "goto_target"):
                # Mapping interpolation mode
                interpolation_map = {
                    "linear": "linear",
                    "minjerk": "minjerk",
                    "ease": "ease_in_out",  # ou "ease" selon SDK
                    "cartoon": "cartoon",
                }
                method = interpolation_map.get(interpolation.value, "minjerk")
                robot.goto_target(head=pose_matrix, duration=duration, method=method)
            else:
                # Fallback: utiliser set_joint_pos ou goto_pose
                logger.warning("goto_target non disponible, utilisation fallback")

            robot.disconnect()

        return {
            "status": "moving",
            "target_pose": pose.model_dump(),
            "duration": duration,
            "interpolation": interpolation.value,
            "estimated_time": duration,
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.exception("Erreur lors du mouvement")
        # Retourner quand même une réponse
        return {
            "status": "error",
            "target_pose": pose.model_dump(),
            "duration": duration,
            "interpolation": interpolation.value,
            "error": str(e),
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
        f"Définition des positions d'articulations : {len(positions)} articulations",
    )

    # Validation des noms de joints avec notre système centralisé
    for pos in positions:
        if not validate_joint_name(pos.joint_name):
            raise HTTPException(
                status_code=422,
                detail=f"Joint '{pos.joint_name}' non valide",
            )

        # Clamp des angles dans les limites
        clamped_angle = clamp_joint_angle(pos.joint_name, pos.position)
        if clamped_angle != pos.position:
            logger.warning(
                f"Angle {pos.position:.3f} clampé à {clamped_angle:.3f} "
                f"pour joint {pos.joint_name}",
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
            status_code=400,
            detail="Côté invalide. Utilisez 'left' ou 'right'",
        )

    if action not in ["open", "close", "grip"]:
        raise HTTPException(
            status_code=400,
            detail="Action invalide. Utilisez 'open', 'close' ou 'grip'",
        )

    logger.info("Contrôle de la pince %s : %s", side, action)

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
        f"Contrôle de la tête : yaw={head_control.yaw}, pitch={head_control.pitch}",
    )

    # Application des positions dans la simulation
    success_yaw = simulation_service.set_joint_position("neck_yaw", head_control.yaw)
    success_pitch = simulation_service.set_joint_position(
        "head_pitch",
        head_control.pitch,
    )

    return {
        "status": "moving" if (success_yaw or success_pitch) else "failed",
        "target": {"yaw": head_control.yaw, "pitch": head_control.pitch},
        "success": {"yaw": success_yaw, "pitch": success_pitch},
        "estimated_time": 1.0,
        "timestamp": datetime.now().isoformat(),
    }


@router.post("/wake_up")
async def wake_up() -> dict[str, Any]:
    """Réveille le robot - séquence de réveil complète.

    Returns:
        Statut du réveil

    """
    logger.info("Réveil du robot")
    try:
        from bbia_sim.robot_factory import RobotFactory

        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()
            if hasattr(robot, "wake_up"):
                robot.wake_up()
            else:
                # Fallback: utiliser comportement wake_up
                from bbia_sim.bbia_behavior import BBIABehaviorManager

                behavior_manager = BBIABehaviorManager(robot_api=robot)
                behavior_manager.execute_behavior("wake_up")
            robot.disconnect()

        return {
            "status": "waking_up",
            "message": "Robot en cours de réveil",
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.exception("Erreur lors du réveil")
        return {
            "status": "error",
            "message": f"Erreur: {e!s}",
            "timestamp": datetime.now().isoformat(),
        }


@router.post("/goto_sleep")
async def goto_sleep() -> dict[str, Any]:
    """Met le robot en veille - séquence de mise en veille.

    Returns:
        Statut de la mise en veille

    """
    logger.info("Mise en veille du robot")
    try:
        from bbia_sim.robot_factory import RobotFactory

        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()
            if hasattr(robot, "goto_sleep"):
                robot.goto_sleep()
            else:
                # Fallback: utiliser comportement goto_sleep si disponible
                from bbia_sim.bbia_behavior import BBIABehaviorManager

                behavior_manager = BBIABehaviorManager(robot_api=robot)
                # Si comportement goto_sleep existe
                if "goto_sleep" in behavior_manager.behaviors:
                    behavior_manager.execute_behavior("goto_sleep")
            robot.disconnect()

        return {
            "status": "going_to_sleep",
            "message": "Robot en cours de mise en veille",
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.exception("Erreur lors de la mise en veille")
        return {
            "status": "error",
            "message": f"Erreur: {e!s}",
            "timestamp": datetime.now().isoformat(),
        }


class EmotionRequest(BaseModel):
    """Modèle pour une requête d'émotion."""

    emotion: str
    intensity: float = 0.5


@router.post("/emotion")
async def set_emotion(emotion_request: EmotionRequest) -> dict[str, Any]:
    """Définit une émotion avec intensité.

    Args:
        emotion_request: Requête contenant l'émotion et l'intensité

    Returns:
        Confirmation de définition de l'émotion
    """
    emotion = emotion_request.emotion
    intensity = emotion_request.intensity

    # Valider l'intensité
    if not 0.0 <= intensity <= 1.0:
        return {
            "status": "error",
            "message": f"Intensité invalide: {intensity} (doit être entre 0.0 et 1.0)",
            "timestamp": datetime.now().isoformat(),
        }
    logger.info("Définition émotion: %s avec intensité %s", emotion, intensity)

    try:
        from bbia_sim.robot_factory import RobotFactory

        robot = RobotFactory.create_backend("mujoco")
        if robot:
            robot.connect()
            if hasattr(robot, "set_emotion"):
                success = robot.set_emotion(emotion, intensity)
                if success:
                    robot.disconnect()
                    return {
                        "status": "success",
                        "emotion": emotion,
                        "intensity": intensity,
                        "message": f"Émotion '{emotion}' définie avec intensité {intensity}",
                        "timestamp": datetime.now().isoformat(),
                    }
            robot.disconnect()

        return {
            "status": "error",
            "message": "Impossible de définir l'émotion",
            "timestamp": datetime.now().isoformat(),
        }
    except Exception as e:
        logger.exception("Erreur lors de la définition de l'émotion")
        return {
            "status": "error",
            "message": f"Erreur: {e!s}",
            "timestamp": datetime.now().isoformat(),
        }


@router.post("/stop")
async def stop_motion() -> dict[str, Any]:
    """Arrête tous les mouvements (arrêt d'urgence si disponible).

    Utilise emergency_stop() si le robot le supporte, sinon arrêt standard.

    Returns:
        Statut de l'arrêt

    """
    logger.info("Arrêt de tous les mouvements")

    # Essayer d'utiliser emergency_stop() si disponible
    try:
        from bbia_sim.robot_factory import RobotFactory

        # Essayer d'obtenir le robot actif
        robot = RobotFactory.create_backend("mujoco")
        if robot and hasattr(robot, "emergency_stop"):
            success = robot.emergency_stop()
            if success:
                logger.critical("🛑 Arrêt d'urgence activé via emergency_stop()")
                return {
                    "status": "emergency_stopped",
                    "message": "Arrêt d'urgence activé - Tous les mouvements arrêtés",
                    "timestamp": datetime.now().isoformat(),
                }
    except Exception as e:
        logger.debug("Emergency stop non disponible, fallback standard: %s", e)

    # Fallback: arrêt standard (asynchrone, ignorer si déjà arrêté)
    try:
        # Appel direct asynchrone (stop_simulation est déjà async)
        await simulation_service.stop_simulation()
    except Exception as e:
        logger.debug(
            "Erreur lors de l'arrêt de la simulation (peut être déjà arrêtée): %s",
            e,
        )

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
    logger.info("Exécution de la commande personnalisée : %s", command.command)

    # Simulation de l'exécution
    return {
        "status": "executing",
        "command": command.command,
        "parameters": command.parameters,
        "estimated_time": 2.0,
        "timestamp": datetime.now().isoformat(),
    }
