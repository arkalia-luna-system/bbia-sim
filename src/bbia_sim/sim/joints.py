"""Validation et configuration des joints BBIA-SIM.

Ce module centralise la validation des noms de joints
et leurs limites pour assurer la cohérence entre l'API
et le simulateur MuJoCo.
"""

from typing import Any

# Mapping des joints valides avec leurs limites (min, max) en radians
# ⚠️ NOTE EXPERT: Ces limites sont génériques pour la simulation MuJoCo directe.
# Pour les limites EXACTES du SDK Reachy Mini officiel, voir:
# - mapping_reachy.py (source de vérité)
# - reachy_mini_backend.py (implémentation SDK)
# Les limites exactes sont extraites de reachy_mini_REAL_OFFICIAL.xml
VALID_JOINTS: dict[str, tuple[float, float]] = {
    # Corps principal - Limite exacte: (-2.792526803190975, 2.792526803190879) rad
    "yaw_body": (
        -2.79,
        2.79,
    ),  # Approximation pour sim (limite exacte dans mapping_reachy.py)
    # Articulations Stewart (plateforme parallèle) - Limites exactes du XML officiel
    # ⚠️ IMPORTANT: Les joints stewart ne doivent PAS être contrôlés individuellement
    # Utiliser goto_target() ou set_target_head_pose() avec IK au lieu de set_joint_pos()
    # Limites exactes du XML:
    # stewart_1: (-0.8377580409572196, 1.3962634015955222)
    # stewart_2: (-1.396263401595614, 1.2217304763958803)
    # stewart_3: (-0.8377580409572173, 1.3962634015955244)
    # stewart_4: (-1.3962634015953894, 0.8377580409573525)
    # stewart_5: (-1.2217304763962082, 1.396263401595286)
    # stewart_6: (-1.3962634015954123, 0.8377580409573296)
    "stewart_1": (-0.84, 1.40),  # Approximation (limite exacte: -0.838, 1.396)
    "stewart_2": (-1.40, 1.22),  # Approximation (limite exacte: -1.396, 1.222)
    "stewart_3": (-0.84, 1.40),  # Approximation (limite exacte: -0.838, 1.396)
    "stewart_4": (-1.40, 0.84),  # Approximation (limite exacte: -1.396, 0.838)
    "stewart_5": (-1.22, 1.40),  # Approximation (limite exacte: -1.222, 1.396)
    "stewart_6": (-1.40, 0.84),  # Approximation (limite exacte: -1.396, 0.838)
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
