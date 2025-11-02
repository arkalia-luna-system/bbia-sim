#!/usr/bin/env python3
"""
mapping_reachy.py - Source de vérité pour les joints Reachy Mini
Mapping unique des noms/limites joints pour éviter les divergences sim ↔ réel
"""

from dataclasses import dataclass


@dataclass
class JointInfo:
    """Information complète d'un joint Reachy."""

    name: str
    min_limit: float  # rad
    max_limit: float  # rad
    safe_amplitude: float  # rad (≤ 0.3)
    description: str = ""
    is_active: bool = True


class ReachyMapping:
    """Mapping centralisé des joints Reachy Mini (SDK officiel)."""

    # Joints principaux (mobiles) - Noms réels du modèle MuJoCo officiel
    JOINTS: dict[str, JointInfo] = {
        # Corps (1 joint principal) - Limites réelles du modèle MuJoCo officiel
        "yaw_body": JointInfo(
            name="yaw_body",
            min_limit=-2.792526803190975,  # Exact du XML officiel
            max_limit=2.792526803190879,  # Exact du XML officiel
            safe_amplitude=0.3,
            description="Rotation du corps principal",
        ),
        # Tête (6 joints Stewart platform) - Limites réelles du modèle MuJoCo officiel
        "stewart_1": JointInfo(
            name="stewart_1",
            min_limit=-0.8377580409572196,  # Exact du XML officiel
            max_limit=1.3962634015955222,  # Exact du XML officiel
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint tête 1 (⚠️ Nécessite IK via goto_target/set_target_head_pose)",
        ),
        "stewart_2": JointInfo(
            name="stewart_2",
            min_limit=-1.396263401595614,  # Exact du XML officiel
            max_limit=1.2217304763958803,  # Exact du XML officiel
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint tête 2 (⚠️ Nécessite IK via goto_target/set_target_head_pose)",
        ),
        "stewart_3": JointInfo(
            name="stewart_3",
            min_limit=-0.8377580409572173,  # Exact du XML officiel
            max_limit=1.3962634015955244,  # Exact du XML officiel
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint tête 3 (⚠️ Nécessite IK via goto_target/set_target_head_pose)",
        ),
        "stewart_4": JointInfo(
            name="stewart_4",
            min_limit=-1.3962634015953894,  # Exact du XML officiel
            max_limit=0.8377580409573525,  # Exact du XML officiel
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint tête 4 (⚠️ Nécessite IK via goto_target/set_target_head_pose)",
        ),
        "stewart_5": JointInfo(
            name="stewart_5",
            min_limit=-1.2217304763962082,  # Exact du XML officiel
            max_limit=1.396263401595286,  # Exact du XML officiel
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint tête 5 (⚠️ Nécessite IK via goto_target/set_target_head_pose)",
        ),
        "stewart_6": JointInfo(
            name="stewart_6",
            min_limit=-1.3962634015954123,  # Exact du XML officiel
            max_limit=0.8377580409573296,  # Exact du XML officiel
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint tête 6 (⚠️ Nécessite IK via goto_target/set_target_head_pose)",
        ),
    }

    # Joints interdits (passifs/bloqués)
    # Note: Antennes maintenant animables avec limites sûres (-0.3 à 0.3 rad)
    # Elles peuvent être retirées de FORBIDDEN_JOINTS si on veut les animer
    FORBIDDEN_JOINTS: set[str] = {
        # "left_antenna",   # Optionnel: décommenter pour bloquer par défaut
        # "right_antenna",  # Optionnel: décommenter pour bloquer par défaut
        "passive_1",
        "passive_2",
        "passive_3",
        "passive_4",
        "passive_5",
        "passive_6",
        "passive_7",
    }

    # Joints recommandés pour les démos (contrôle direct sécurisé)
    # ⚠️ IMPORTANT EXPERT ROBOTIQUE: Les joints stewart (stewart_1-6) NE PEUVENT PAS être contrôlés individuellement
    # car la plateforme Stewart utilise la cinématique inverse (IK). Chaque joint stewart influence
    # plusieurs degrés de liberté simultanément (roll, pitch, yaw, position X/Y/Z).
    #
    # Méthodes correctes pour contrôler la tête (SDK officiel):
    # 1. goto_target(head=pose_4x4, ...) - ⭐ Recommandé avec interpolation "minjerk"
    # 2. set_target_head_pose(pose_4x4) - Contrôle direct via IK
    # 3. look_at_world(x, y, z) - Calcul IK automatique vers point 3D
    # 4. create_head_pose(pitch, yaw, roll) puis set_target_head_pose() - Interface simple
    #
    # RECOMMENDED_JOINTS ne liste que yaw_body car c'est le seul joint mobile pouvant être contrôlé directement.
    RECOMMENDED_JOINTS: set[str] = {
        "yaw_body"
    }  # stewart_1-6 nécessitent goto_target() / IK

    # Limite de sécurité globale
    GLOBAL_SAFETY_LIMIT: float = 0.3  # rad

    @classmethod
    def get_joint_info(cls, joint_name: str) -> JointInfo:
        """Récupère les informations d'un joint."""
        if joint_name in cls.FORBIDDEN_JOINTS:
            raise ValueError(f"Joint interdit: {joint_name}")

        if joint_name not in cls.JOINTS:
            raise ValueError(f"Joint inconnu: {joint_name}")

        return cls.JOINTS[joint_name]

    @classmethod
    def validate_position(cls, joint_name: str, position: float) -> tuple[bool, float]:
        """
        Valide et clamp une position de joint.

        ⚠️ IMPORTANT (Sécurité Expert):
        - Applique d'abord les limites hardware (min_limit, max_limit)
        - Puis applique la limite de sécurité (safe_amplitude) si plus restrictive
        - Pour les joints stewart: utiliser goto_target() avec IK plutôt que contrôle direct

        Returns:
            (is_valid, clamped_position)
        """
        if joint_name in cls.FORBIDDEN_JOINTS:
            return False, 0.0

        if joint_name not in cls.JOINTS:
            return False, 0.0

        joint_info = cls.JOINTS[joint_name]

        # Étape 1: Clamp dans les limites hardware du joint (SDK officiel)
        clamped_pos = max(joint_info.min_limit, min(joint_info.max_limit, position))

        # Étape 2: Limite de sécurité logicielle (plus restrictive)
        # CORRECTION EXPERTE: Appliquer la limite de sécurité seulement si elle est
        # plus restrictive que les limites hardware (aligné avec reachy_mini_backend.py)
        safe_min = max(-joint_info.safe_amplitude, joint_info.min_limit)
        safe_max = min(joint_info.safe_amplitude, joint_info.max_limit)

        # Ne clamp que si la limite de sécurité est réellement plus restrictive
        if safe_min > joint_info.min_limit or safe_max < joint_info.max_limit:
            clamped_pos = max(safe_min, min(safe_max, clamped_pos))

        return True, clamped_pos

    @classmethod
    def get_all_joints(cls) -> set[str]:
        """Retourne tous les joints disponibles."""
        return set(cls.JOINTS.keys())

    @classmethod
    def get_recommended_joints(cls) -> set[str]:
        """Retourne les joints recommandés pour les démos."""
        return cls.RECOMMENDED_JOINTS.copy()

    @classmethod
    def get_forbidden_joints(cls) -> set[str]:
        """Retourne les joints interdits."""
        return cls.FORBIDDEN_JOINTS.copy()

    @classmethod
    def is_joint_safe(cls, joint_name: str) -> bool:
        """Vérifie si un joint est sûr à utiliser."""
        return joint_name in cls.JOINTS and joint_name not in cls.FORBIDDEN_JOINTS


# Export des constantes pour compatibilité
JOINTS = ReachyMapping.JOINTS
FORBIDDEN_JOINTS = ReachyMapping.FORBIDDEN_JOINTS
RECOMMENDED_JOINTS = ReachyMapping.RECOMMENDED_JOINTS
GLOBAL_SAFETY_LIMIT = ReachyMapping.GLOBAL_SAFETY_LIMIT


def validate_joint_position(joint_name: str, position: float) -> tuple[bool, float]:
    """Fonction utilitaire pour valider une position de joint."""
    return ReachyMapping.validate_position(joint_name, position)


def get_joint_info(joint_name: str) -> JointInfo:
    """Fonction utilitaire pour récupérer les infos d'un joint."""
    return ReachyMapping.get_joint_info(joint_name)


if __name__ == "__main__":
    """Test du mapping."""
    print("🔍 Test du mapping Reachy Mini")
    print("=" * 50)

    # Test joints valides
    print("✅ Joints disponibles:")
    for joint_name in ReachyMapping.get_all_joints():
        joint_info = ReachyMapping.get_joint_info(joint_name)
        print(
            f"  - {joint_name}: [{joint_info.min_limit:.2f}, {joint_info.max_limit:.2f}] rad"
        )

    print(f"\n🚫 Joints interdits: {len(ReachyMapping.get_forbidden_joints())}")
    for joint in ReachyMapping.get_forbidden_joints():
        print(f"  - {joint}")

    print(f"\n🎯 Joints recommandés: {ReachyMapping.get_recommended_joints()}")

    # Test validation
    print("\n🧪 Test validation:")
    test_cases = [
        ("yaw_body", 0.2),  # OK
        ("yaw_body", 0.5),  # Clampé à 0.3
        ("yaw_body", -0.8),  # Clampé à -0.3
        ("left_antenna", 0.1),  # Animable avec limites (-0.3 à 0.3 rad)
        ("unknown_joint", 0.1),  # Inconnu
    ]

    for joint, pos in test_cases:
        try:
            is_valid, clamped = ReachyMapping.validate_position(joint, pos)
            print(f"  {joint}({pos}) → {is_valid}, {clamped:.2f}")
        except ValueError as e:
            print(f"  {joint}({pos}) → ERREUR: {e}")

    print("\n✅ Mapping Reachy Mini validé !")
