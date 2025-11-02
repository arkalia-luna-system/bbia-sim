"""Router pour les endpoints de contrôle des moteurs (conforme SDK officiel)."""

from enum import Enum

from fastapi import APIRouter, Depends
from pydantic import BaseModel

from ..backend_adapter import BackendAdapter, get_backend_adapter

router = APIRouter(prefix="/motors")

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
    mode_obj = backend.get_motor_control_mode()
    # Convertir en string (support objet avec .value ou string directe)
    mode_str = mode_obj.value if hasattr(mode_obj, "value") else str(mode_obj)

    # S'assurer que le mode est dans l'enum MotorControlMode
    if mode_str not in ["enabled", "disabled", "gravity_compensation"]:
        mode_str = "enabled"  # Valeur par défaut

    return MotorStatus(mode=MotorControlMode(mode_str))


@router.post("/set_mode/{mode}")
async def set_motor_mode(
    mode: MotorControlMode,
    backend: BackendAdapter = Depends(get_backend_adapter),
) -> dict[str, str]:
    """Définit le mode de contrôle des moteurs (conforme SDK)."""
    backend.set_motor_control_mode(mode)
    return {"status": f"motors changed to {mode} mode"}
