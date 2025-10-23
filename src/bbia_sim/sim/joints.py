"""
Validation et configuration des joints BBIA-SIM.

Ce module centralise la validation des noms de joints
et leurs limites pour assurer la cohérence entre l'API
et le simulateur MuJoCo.
"""

from typing import Any

# Mapping des joints valides avec leurs limites (min, max) en radians
VALID_JOINTS: dict[str, tuple[float, float]] = {
    # Cou
    "neck_yaw": (-1.57, 1.57),  # ±90°
    "neck_pitch": (-0.79, 0.79),  # ±45°
    "neck_roll": (-0.52, 0.52),  # ±30°
    # Épaule droite
    "right_shoulder_pitch": (-1.57, 1.57),  # ±90°
    "right_shoulder_roll": (-1.57, 1.57),  # ±90°
    "right_shoulder_yaw": (-1.57, 1.57),  # ±90°
    # Coude droit
    "right_elbow_pitch": (-1.57, 1.57),  # ±90°
    "right_elbow_roll": (-1.57, 1.57),  # ±90°
    # Poignet droit
    "right_wrist_pitch": (-1.57, 1.57),  # ±90°
    "right_wrist_roll": (-1.57, 1.57),  # ±90°
    "right_wrist_yaw": (-1.57, 1.57),  # ±90°
    # Épaule gauche
    "left_shoulder_pitch": (-1.57, 1.57),  # ±90°
    "left_shoulder_roll": (-1.57, 1.57),  # ±90°
    "left_shoulder_yaw": (-1.57, 1.57),  # ±90°
    # Coude gauche
    "left_elbow_pitch": (-1.57, 1.57),  # ±90°
    "left_elbow_roll": (-1.57, 1.57),  # ±90°
    # Poignet gauche
    "left_wrist_pitch": (-1.57, 1.57),  # ±90°
    "left_wrist_roll": (-1.57, 1.57),  # ±90°
    "left_wrist_yaw": (-1.57, 1.57),  # ±90°
    # Gripper (pince)
    "right_gripper": (0.0, 0.04),  # 0-4cm
    "left_gripper": (0.0, 0.04),  # 0-4cm
}

# Liste des noms de joints valides (pour validation rapide)
VALID_JOINT_NAMES: list[str] = list(VALID_JOINTS.keys())

# Joints principaux pour les démos
MAIN_JOINTS: list[str] = [
    "neck_yaw",
    "neck_pitch",
    "right_shoulder_pitch",
    "right_elbow_pitch",
    "left_shoulder_pitch",
    "left_elbow_pitch",
]


def validate_joint_name(joint_name: str) -> bool:
    """
    Valide qu'un nom de joint existe.

    Args:
        joint_name: Nom du joint à valider

    Returns:
        True si le joint est valide
    """
    return joint_name in VALID_JOINT_NAMES


def get_joint_limits(joint_name: str) -> tuple[float, float]:
    """
    Récupère les limites d'un joint.

    Args:
        joint_name: Nom du joint

    Returns:
        Tuple (min_limit, max_limit) en radians

    Raises:
        ValueError: Si le joint n'existe pas
    """
    if not validate_joint_name(joint_name):
        raise ValueError(f"Joint '{joint_name}' non valide")

    return VALID_JOINTS[joint_name]


def clamp_joint_angle(joint_name: str, angle: float) -> float:
    """
    Clamp un angle dans les limites d'un joint.

    Args:
        joint_name: Nom du joint
        angle: Angle à clamper

    Returns:
        Angle clamper dans les limites

    Raises:
        ValueError: Si le joint n'existe pas
    """
    min_limit, max_limit = get_joint_limits(joint_name)
    return max(min_limit, min(max_limit, angle))


def get_joint_info(joint_name: str) -> dict[str, Any]:
    """
    Récupère toutes les informations d'un joint.

    Args:
        joint_name: Nom du joint

    Returns:
        Dict avec les informations du joint

    Raises:
        ValueError: Si le joint n'existe pas
    """
    if not validate_joint_name(joint_name):
        raise ValueError(f"Joint '{joint_name}' non valide")

    min_limit, max_limit = get_joint_limits(joint_name)

    return {
        "name": joint_name,
        "min_limit": min_limit,
        "max_limit": max_limit,
        "range": max_limit - min_limit,
        "is_main": joint_name in MAIN_JOINTS,
    }
