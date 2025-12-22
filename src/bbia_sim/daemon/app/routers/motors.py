"""Router pour les endpoints de contrôle des moteurs (conforme SDK officiel)."""

import logging
from enum import Enum
from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from bbia_sim.daemon.app.backend_adapter import BackendAdapter, get_backend_adapter

logger = logging.getLogger(__name__)

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
    backend: Annotated[BackendAdapter, Depends(get_backend_adapter)],
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
    backend: Annotated[BackendAdapter, Depends(get_backend_adapter)],
) -> dict[str, str]:
    """Définit le mode de contrôle des moteurs (conforme SDK)."""
    backend.set_motor_control_mode(mode)
    return {"status": f"motors changed to {mode} mode"}


def get_motor_control_mode(joint_name: str) -> MotorControlMode:  # noqa: ARG001
    """
    Récupère le mode de contrôle moteur pour un joint spécifique.

    Args:
        joint_name: Nom du joint (actuellement ignoré, retourne le mode global)

    Returns:
        Mode de contrôle moteur
    """
    # Note: get_backend_adapter nécessite un contexte FastAPI, donc on crée un adaptateur directement
    backend = BackendAdapter()
    mode_obj = backend.get_motor_control_mode()
    # Convertir en string (support objet avec .value ou string directe)
    mode_str = mode_obj.value if hasattr(mode_obj, "value") else str(mode_obj)

    # S'assurer que le mode est dans l'enum MotorControlMode
    if mode_str not in ["enabled", "disabled", "gravity_compensation"]:
        mode_str = "enabled"  # Valeur par défaut

    return MotorControlMode(mode_str)


class MotorDiagnosticResult(BaseModel):
    """Résultat du diagnostic des moteurs."""

    available_joints: list[str]
    missing_joints: list[str]
    error_details: dict[str, str]
    recommendations: list[str]


@router.get("/diagnostic", response_model=MotorDiagnosticResult)
async def diagnose_motors(
    backend: Annotated[BackendAdapter, Depends(get_backend_adapter)],
) -> MotorDiagnosticResult:
    """Diagnostique l'état des moteurs et détecte les moteurs manquants.

    Returns:
        Résultat du diagnostic avec liste des joints disponibles et manquants
    """
    try:
        # Obtenir les joints disponibles
        available_joints = backend.get_available_joints()

        # Joints attendus pour Reachy Mini
        expected_joints = [
            "yaw_body",
            "stewart_1",
            "stewart_2",
            "stewart_3",
            "stewart_4",
            "stewart_5",
            "stewart_6",
            "left_antenna",
            "right_antenna",
        ]

        # Identifier les joints manquants
        missing_joints = [j for j in expected_joints if j not in available_joints]

        # Détecter les erreurs spécifiques
        error_details: dict[str, str] = {}
        recommendations: list[str] = []

        if missing_joints:
            for joint in missing_joints:
                # Mapping joint → Motor ID
                joint_to_id = {
                    "yaw_body": 10,
                    "stewart_1": 11,
                    "stewart_2": 12,
                    "stewart_3": 13,  # ← Le moteur problématique dans ton cas
                    "stewart_4": 14,
                    "stewart_5": 15,
                    "stewart_6": 16,
                    "left_antenna": 17,
                    "right_antenna": 18,
                }
                motor_id = joint_to_id.get(joint, "?")
                error_details[joint] = (
                    f"Motor ID {motor_id} non détecté. "
                    "Possible cause: mauvais baudrate (57,600 au lieu de 1,000,000) "
                    "ou paramètres d'usine (ID=1 au lieu de {motor_id})"
                )

            recommendations.append(
                "Lancer le scan automatique: python examples/reachy_mini/scan_motors_baudrate.py"
            )
            recommendations.append(
                "OU utiliser le script officiel: reachy-mini-reflash-motors"
            )
            recommendations.append(
                "Vérifier que le daemon est arrêté: sudo systemctl stop reachy-mini-daemon"
            )

        # Vérifier les erreurs de connexion
        try:
            backend.connect_if_needed()
        except Exception as e:
            error_details["connection"] = f"Erreur de connexion: {str(e)}"
            recommendations.append("Vérifier la connexion au robot")
            recommendations.append(
                "Redémarrer le daemon: sudo systemctl restart reachy-mini-daemon"
            )

        return MotorDiagnosticResult(
            available_joints=available_joints,
            missing_joints=missing_joints,
            error_details=error_details,
            recommendations=recommendations,
        )

    except Exception as e:
        logger.exception("Erreur lors du diagnostic des moteurs: %s", e)
        raise HTTPException(
            status_code=500,
            detail=f"Erreur lors du diagnostic: {str(e)}",
        ) from e
