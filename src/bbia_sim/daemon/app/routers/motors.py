"""Router pour les endpoints de contrôle des moteurs (conforme SDK officiel)."""

from enum import Enum

from fastapi import APIRouter, Depends
from pydantic import BaseModel

from ..backend_adapter import BackendAdapter, get_backend_adapter

router = APIRouter(prefix="/api/motors")

# Utiliser MotorControlMode conforme SDK officiel
# Compatible avec reachy_mini.daemon.backend.abstract.MotorControlMode


class MotorControlMode(str, Enum):
    """Mode de contrôle des moteurs (conforme SDK officiel)."""

    Enabled = "enabled"
    Disabled = "disabled"
    GravityCompensation = "gravity_compensation"


class MotorStatus(BaseModel):
    """Statut des moteurs (conforme SDK officiel)."""

    mode: MotorControlMode


@router.get("/status")
async def get_motor_status(
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> MotorStatus:
    """Récupère le statut actuel des moteurs (conforme SDK)."""
    return MotorStatus(mode=backend.get_motor_control_mode())


@router.post("/set_mode/{mode}")
async def set_motor_mode(
    mode: MotorControlMode,
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> dict[str, str]:
    """Définit le mode de contrôle des moteurs (conforme SDK)."""
    backend.set_motor_control_mode(mode)
    return {"status": f"motors changed to {mode} mode"}
