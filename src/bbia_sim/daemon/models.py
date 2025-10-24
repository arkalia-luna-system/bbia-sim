"""Modèles Pydantic pour la validation des données API."""

from typing import Any

from pydantic import BaseModel, Field, field_validator


class JointPosition(BaseModel):
    """Modèle pour une position d'articulation avec validation."""

    joint_name: str = Field(..., min_length=1, max_length=50)
    position: float = Field(..., ge=-3.14, le=3.14)  # Limite physique réaliste

    @field_validator("joint_name")
    @classmethod
    def validate_joint_name(cls, v):
        """Valide le nom de l'articulation."""
        from ..sim.joints import validate_joint_name

        if not validate_joint_name(v):
            raise ValueError(f"Articulation '{v}' non autorisée")
        return v


class Pose(BaseModel):
    """Modèle pour une position avec validation."""

    x: float = Field(..., ge=-1.0, le=1.0)  # Limites réalistes en mètres
    y: float = Field(..., ge=-1.0, le=1.0)
    z: float = Field(..., ge=0.0, le=2.0)
    roll: float = Field(0.0, ge=-3.14, le=3.14)
    pitch: float = Field(0.0, ge=-3.14, le=3.14)
    yaw: float = Field(0.0, ge=-3.14, le=3.14)


class HeadControl(BaseModel):
    """Modèle pour le contrôle de la tête avec validation."""

    yaw: float = Field(..., ge=-1.57, le=1.57)  # Limites physiques réalistes
    pitch: float = Field(..., ge=-0.5, le=0.5)


class GripperControl(BaseModel):
    """Modèle pour le contrôle des pinces."""

    side: str = Field(..., pattern="^(left|right)$")
    action: str = Field(..., pattern="^(open|close|grip)$")


class MotionCommand(BaseModel):
    """Modèle pour une commande de mouvement personnalisée."""

    command: str = Field(..., min_length=1, max_length=100)
    parameters: dict[str, Any] = Field(default_factory=dict, max_length=10)

    @field_validator("parameters")
    @classmethod
    def validate_parameters(cls, v):
        """Valide les paramètres de la commande."""
        if len(v) > 10:
            raise ValueError("Trop de paramètres (max 10)")
        return v


class TelemetryMessage(BaseModel):
    """Modèle pour les messages de télémétrie WebSocket."""

    type: str = Field(..., pattern="^(ping|pong|status|telemetry)$")
    data: dict[str, Any] = Field(default_factory=dict, max_length=50)

    @field_validator("data")
    @classmethod
    def validate_data(cls, v):
        """Valide les données de télémétrie."""
        if len(v) > 50:
            raise ValueError("Trop de données (max 50 champs)")
        return v
