"""Modèles Pydantic pour l'API BBIA-SIM (conforme SDK officiel)."""

from typing import Any
from uuid import UUID

import numpy as np
from pydantic import BaseModel, Field

try:
    from reachy_mini.utils.interpolation import HeadPose  # type: ignore[import-untyped]
except ImportError:
    HeadPose = Any  # type: ignore[misc,assignment]


class AnyPose(BaseModel):
    """Pose flexible (matrice 4x4 ou HeadPose ou dict)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def to_pose_array(self) -> np.ndarray:
        """Convertit en tableau numpy 4x4."""
        # Créer matrice de transformation depuis xyz + rpy
        import math

        cos_r, sin_r = math.cos(self.roll), math.sin(self.roll)
        cos_p, sin_p = math.cos(self.pitch), math.sin(self.pitch)
        cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)

        # Matrice de rotation ZYX (yaw, pitch, roll)
        R = np.array(
            [
                [
                    cos_y * cos_p,
                    cos_y * sin_p * sin_r - sin_y * cos_r,
                    cos_y * sin_p * cos_r + sin_y * sin_r,
                ],
                [
                    sin_y * cos_p,
                    sin_y * sin_p * sin_r + cos_y * cos_r,
                    sin_y * sin_p * cos_r - cos_y * sin_r,
                ],
                [-sin_p, cos_p * sin_r, cos_p * cos_r],
            ],
            dtype=np.float64,
        )

        # Matrice 4x4 complète
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = R
        pose[:3, 3] = [self.x, self.y, self.z]

        return pose


def as_any_pose(data: Any) -> AnyPose:
    """Convertit diverses représentations en AnyPose."""
    if isinstance(data, dict):
        return AnyPose(**data)
    if isinstance(data, AnyPose):
        return data
    if isinstance(data, np.ndarray):
        # Extraire xyz depuis colonne 3
        x, y, z = data[:3, 3]
        # Extraire rpy depuis matrice de rotation (simplifié)
        # TODO: Implémenter extraction RPY complète si nécessaire
        return AnyPose(x=x, y=y, z=z, roll=0.0, pitch=0.0, yaw=0.0)
    return AnyPose()


class FullBodyTarget(BaseModel):
    """Cible complète du corps (conforme SDK)."""

    target_head_pose: AnyPose | None = None
    target_antennas: tuple[float, float] | None = None
    target_body_yaw: float | None = None


class FullState(BaseModel):
    """État complet du robot (conforme SDK)."""

    present_head_pose: Any = None
    present_body_yaw: float | None = None
    present_antenna_joint_positions: tuple[float, float] | None = None
    target_head_pose: Any | None = None
    target_body_yaw: float | None = None
    target_antenna_joint_positions: tuple[float, float] | None = None


class MoveUUID(BaseModel):
    """UUID d'un mouvement (conforme SDK)."""

    uuid: UUID = Field(default_factory=lambda: __import__("uuid").uuid4())


class BatchMovementRequest(BaseModel):
    """Requête pour exécuter plusieurs mouvements en batch."""

    movements: list[dict[str, Any]] = Field(
        ...,
        description="Liste de mouvements à exécuter en batch",
        min_items=1,
        max_items=10,
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "movements": [
                        {
                            "type": "goto",
                            "head_pose": {"x": 0.0, "y": 0.0, "z": 0.2},
                            "duration": 1.0,
                        },
                        {
                            "type": "goto",
                            "head_pose": {"x": 0.1, "y": 0.0, "z": 0.2},
                            "duration": 1.0,
                        },
                    ],
                },
            ],
        },
    }
