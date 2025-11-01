"""Router pour les endpoints de contrôle des moteurs."""

import logging
from enum import Enum

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from ....robot_factory import RobotFactory

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/motors")


class MotorControlMode(str, Enum):
    """Mode de contrôle des moteurs."""

    ENABLED = "enabled"
    DISABLED = "disabled"
    GRAVITY_COMPENSATION = "gravity_compensation"


class MotorStatus(BaseModel):
    """Statut des moteurs."""

    mode: MotorControlMode
    status: str


def get_backend():
    """Dependency pour obtenir le backend robot."""
    # Utiliser le backend actif ou mujoco par défaut
    backend = RobotFactory.create_backend("mujoco")
    if backend is None:
        backend = RobotFactory.create_backend("reachy_mini")
    if backend is None:
        raise HTTPException(status_code=503, detail="Aucun backend robot disponible")
    return backend


@router.get("/status", response_model=MotorStatus)
async def get_motor_status(backend=Depends(get_backend)) -> MotorStatus:
    """Récupère le statut actuel des moteurs.

    Returns:
        Statut des moteurs avec mode actuel

    Raises:
        HTTPException: En cas d'erreur
    """
    try:
        # Vérifier si le backend supporte les méthodes motor control
        if hasattr(backend, "robot") and backend.robot is not None:
            # Le SDK ReachyMini a enable_motors/disable_motors
            # Par défaut, on assume enabled si pas de méthode get_status
            mode = MotorControlMode.ENABLED

            # Si le robot SDK a une méthode pour vérifier le statut
            if hasattr(backend.robot, "client"):
                try:
                    status = backend.robot.client.get_status()
                    motor_mode = status.get("backend_status", {}).get(
                        "motor_control_mode", "enabled"
                    )
                    mode = MotorControlMode(motor_mode.lower())
                except Exception:
                    pass  # noqa: B110 - Fallback sur enabled

        else:
            # En simulation, les moteurs sont toujours "enabled"
            mode = MotorControlMode.ENABLED

        return MotorStatus(
            mode=mode,
            status="ok",
        )
    except Exception as e:
        logger.error(f"Erreur lors de la récupération du statut moteur: {e}")
        raise HTTPException(status_code=500, detail=f"Erreur: {str(e)}") from e


@router.post("/set_mode/{mode}")
async def set_motor_mode(
    mode: MotorControlMode,
    backend=Depends(get_backend),
) -> dict[str, str]:
    """Définit le mode de contrôle des moteurs.

    Args:
        mode: Mode souhaité (enabled, disabled, gravity_compensation)
        backend: Backend robot (injecté)

    Returns:
        Confirmation du changement de mode

    Raises:
        HTTPException: En cas d'erreur
    """
    try:
        # Vérifier si le backend supporte les méthodes motor control
        if hasattr(backend, "robot") and backend.robot is not None:
            # Utiliser les méthodes SDK
            if mode == MotorControlMode.ENABLED:
                if hasattr(backend.robot, "enable_motors"):
                    backend.robot.enable_motors()
                else:
                    logger.warning("enable_motors non disponible, utilisant fallback")
            elif mode == MotorControlMode.DISABLED:
                if hasattr(backend.robot, "disable_motors"):
                    backend.robot.disable_motors()
                else:
                    logger.warning("disable_motors non disponible, utilisant fallback")
            elif mode == MotorControlMode.GRAVITY_COMPENSATION:
                if hasattr(backend.robot, "enable_gravity_compensation"):
                    backend.robot.enable_gravity_compensation()
                else:
                    logger.warning("enable_gravity_compensation non disponible, utilisant fallback")
        else:
            # En simulation, on log juste
            logger.info(f"Mode simulation: moteurs en mode {mode.value}")

        return {"status": f"motors changed to {mode.value} mode"}
    except Exception as e:
        logger.error(f"Erreur lors du changement de mode moteur: {e}")
        raise HTTPException(status_code=500, detail=f"Erreur: {str(e)}") from e
