"""Modèles Pydantic pour la validation des données API."""

from datetime import datetime
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


# Modèles conformes SDK officiel
class XYZRPYPose(BaseModel):
    """Représente une pose 3D avec position (x, y, z) et orientation (roll, pitch, yaw)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @classmethod
    def from_pose_array(cls, arr: Any) -> "XYZRPYPose":
        """Crée une pose XYZRPYPose depuis un array numpy 4x4."""
        import numpy as np
        from scipy.spatial.transform import Rotation as R

        if isinstance(arr, np.ndarray):
            assert arr.shape == (4, 4), "Array must be of shape (4, 4)"
            x, y, z = arr[0, 3], arr[1, 3], arr[2, 3]
            roll, pitch, yaw = R.from_matrix(arr[:3, :3]).as_euler("xyz")
            return cls(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
        return cls()

    def to_pose_array(self) -> Any:
        """Convertit en matrice 4x4 numpy."""
        import numpy as np
        from scipy.spatial.transform import Rotation as R

        rotation = R.from_euler("xyz", [self.roll, self.pitch, self.yaw])
        pose_matrix = np.eye(4)
        pose_matrix[:3, 3] = [self.x, self.y, self.z]
        pose_matrix[:3, :3] = rotation.as_matrix()
        return pose_matrix


class Matrix4x4Pose(BaseModel):
    """Représente une pose 3D par sa matrice de transformation 4x4."""

    m: tuple[
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
    ]

    @classmethod
    def from_pose_array(cls, arr: Any) -> "Matrix4x4Pose":
        """Crée une pose Matrix4x4Pose depuis un array numpy 4x4."""
        import numpy as np

        if isinstance(arr, np.ndarray):
            assert arr.shape == (4, 4), "Array must be of shape (4, 4)"
            m = tuple(arr.flatten().tolist())
            return cls(m=m)
        return cls(
            m=(
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            )
        )

    def to_pose_array(self) -> Any:
        """Convertit en matrice 4x4 numpy."""
        import numpy as np

        return np.array(self.m).reshape((4, 4))


AnyPose = XYZRPYPose | Matrix4x4Pose


def as_any_pose(pose: Any, use_matrix: bool) -> AnyPose:
    """Convertit un array numpy en AnyPose (conforme SDK)."""
    import numpy as np

    if isinstance(pose, np.ndarray):
        if use_matrix:
            return Matrix4x4Pose.from_pose_array(pose)
        else:
            return XYZRPYPose.from_pose_array(pose)
    # Fallback
    if use_matrix:
        return Matrix4x4Pose(
            m=(
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            )
        )
    return XYZRPYPose()


class FullBodyTarget(BaseModel):
    """Représente le corps complet incluant pose tête et joints antennes (conforme SDK)."""

    target_head_pose: AnyPose | None = None
    target_antennas: tuple[float, float] | None = None
    timestamp: datetime | None = None  # Conforme SDK


class MoveUUID(BaseModel):
    """Identifiant unique pour une tâche de mouvement (conforme SDK)."""

    uuid: str  # UUID en string pour compatibilité JSON (SDK utilise UUID, mais JSON nécessite str)


class FullState(BaseModel):
    """Représente l'état complet du robot incluant toutes les positions d'articulations et poses (conforme SDK)."""

    control_mode: str | None = (
        None  # MotorControlMode.value (SDK utilise MotorControlMode mais on accepte str aussi)
    )
    head_pose: AnyPose | None = None
    head_joints: list[float] | None = None
    body_yaw: float | None = None
    antennas_position: list[float] | None = None
    timestamp: datetime | None = None
    passive_joints: list[float] | None = None
    # Champs target optionnels (ajoutés pour conformité complète)
    target_head_pose: AnyPose | None = None
    target_head_joints: list[float] | None = None
    target_body_yaw: float | None = None
    target_antennas_position: list[float] | None = None
