"""Mapping centralisé des joints et leurs limites pour Reachy Mini."""

from dataclasses import dataclass


@dataclass
class JointSpec:
    """Spécification d'une articulation."""

    name: str
    min_angle: float  # en radians
    max_angle: float  # en radians
    max_velocity: float  # en rad/s
    gear_ratio: int = 100  # ratio du moteur


# Mapping centralisé des articulations Reachy Mini
REACHY_JOINTS: dict[str, JointSpec] = {
    "neck_yaw": JointSpec(
        name="neck_yaw",
        min_angle=-1.57,  # -90°
        max_angle=1.57,  # +90°
        max_velocity=2.0,
        gear_ratio=100,
    ),
    "right_shoulder_pitch": JointSpec(
        name="right_shoulder_pitch",
        min_angle=-1.57,  # -90°
        max_angle=1.57,  # +90°
        max_velocity=2.0,
        gear_ratio=100,
    ),
    "right_elbow_pitch": JointSpec(
        name="right_elbow_pitch",
        min_angle=-1.57,  # -90°
        max_angle=1.57,  # +90°
        max_velocity=2.0,
        gear_ratio=100,
    ),
    "right_gripper_joint": JointSpec(
        name="right_gripper_joint",
        min_angle=-0.5,  # -28.6°
        max_angle=0.5,  # +28.6°
        max_velocity=1.0,
        gear_ratio=50,
    ),
    "left_shoulder_pitch": JointSpec(
        name="left_shoulder_pitch",
        min_angle=-1.57,  # -90°
        max_angle=1.57,  # +90°
        max_velocity=2.0,
        gear_ratio=100,
    ),
    "left_elbow_pitch": JointSpec(
        name="left_elbow_pitch",
        min_angle=-1.57,  # -90°
        max_angle=1.57,  # +90°
        max_velocity=2.0,
        gear_ratio=100,
    ),
    "left_gripper_joint": JointSpec(
        name="left_gripper_joint",
        min_angle=-0.5,  # -28.6°
        max_angle=0.5,  # +28.6°
        max_velocity=1.0,
        gear_ratio=50,
    ),
}


def get_joint_spec(joint_name: str) -> JointSpec:
    """Récupère la spécification d'une articulation."""
    if joint_name not in REACHY_JOINTS:
        raise ValueError(f"Articulation inconnue: {joint_name}")
    return REACHY_JOINTS[joint_name]


def get_available_joints() -> list[str]:
    """Retourne la liste des articulations disponibles."""
    return list(REACHY_JOINTS.keys())


def validate_joint_position(joint_name: str, position: float) -> bool:
    """Valide qu'une position d'articulation est dans les limites."""
    spec = get_joint_spec(joint_name)
    return spec.min_angle <= position <= spec.max_angle


def validate_joint_velocity(joint_name: str, velocity: float) -> bool:
    """Valide qu'une vitesse d'articulation est dans les limites."""
    spec = get_joint_spec(joint_name)
    return abs(velocity) <= spec.max_velocity


def get_joint_range(joint_name: str) -> tuple[float, float]:
    """Retourne la plage de mouvement d'une articulation."""
    spec = get_joint_spec(joint_name)
    return (spec.min_angle, spec.max_angle)
