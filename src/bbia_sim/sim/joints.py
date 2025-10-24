"""Validation et configuration des joints BBIA-SIM.

Ce module centralise la validation des noms de joints
et leurs limites pour assurer la cohérence entre l'API
et le simulateur MuJoCo.
"""

from typing import Any

# Mapping des joints valides avec leurs limites (min, max) en radians
VALID_JOINTS: dict[str, tuple[float, float]] = {
    # Corps principal
    "yaw_body": (-3.14, 3.14),  # ±180°
    # Articulations Stewart (plateforme parallèle)
    "stewart_1": (-3.14, 3.14),  # ±180°
    "stewart_2": (-3.14, 3.14),  # ±180°
    "stewart_3": (-3.14, 3.14),  # ±180°
    "stewart_4": (-3.14, 3.14),  # ±180°
    "stewart_5": (-3.14, 3.14),  # ±180°
    "stewart_6": (-3.14, 3.14),  # ±180°
    # Articulations passives
    "passive_1": (-3.14, 3.14),  # ±180°
    "passive_2": (-3.14, 3.14),  # ±180°
    "passive_3": (-3.14, 3.14),  # ±180°
    "passive_4": (-3.14, 3.14),  # ±180°
    "passive_5": (-3.14, 3.14),  # ±180°
    "passive_6": (-3.14, 3.14),  # ±180°
    "passive_7": (-3.14, 3.14),  # ±180°
    # Antennes
    "right_antenna": (-3.14, 3.14),  # ±180°
    "left_antenna": (-3.14, 3.14),  # ±180°
}

# Liste des noms de joints valides (pour validation rapide)
VALID_JOINT_NAMES: list[str] = list(VALID_JOINTS.keys())

# Joints principaux pour les démos
MAIN_JOINTS: list[str] = [
    "yaw_body",
    "stewart_1",
    "stewart_2",
    "stewart_3",
    "right_antenna",
    "left_antenna",
]


def validate_joint_name(joint_name: str) -> bool:
    """Valide qu'un nom de joint existe.

    Args:
        joint_name: Nom du joint à valider

    Returns:
        True si le joint est valide

    """
    return joint_name in VALID_JOINT_NAMES


def get_joint_limits(joint_name: str) -> tuple[float, float]:
    """Récupère les limites d'un joint.

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
    """Clamp un angle dans les limites d'un joint.

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
    """Récupère toutes les informations d'un joint.

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
