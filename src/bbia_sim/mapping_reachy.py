#!/usr/bin/env python3
"""
mapping_reachy.py - Source de vérité pour les joints Reachy Mini
Mapping unique des noms/limites joints pour éviter les divergences sim ↔ réel
"""

from typing import Dict, Tuple, Set
from dataclasses import dataclass


@dataclass
class JointInfo:
    """Information complète d'un joint Reachy."""
    name: str
    min_limit: float  # rad
    max_limit: float  # rad
    safe_amplitude: float  # rad (≤ 0.3)
    description: str
    is_active: bool = True


class ReachyMapping:
    """Mapping centralisé des joints Reachy Mini."""
    
    # Joints principaux (mobiles)
    JOINTS: Dict[str, JointInfo] = {
        "yaw_body": JointInfo(
            name="yaw_body",
            min_limit=-0.5,
            max_limit=0.5,
            safe_amplitude=0.3,
            description="Rotation du corps principal"
        ),
        "stewart_1": JointInfo(
            name="stewart_1",
            min_limit=-0.3,
            max_limit=0.3,
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint 1"
        ),
        "stewart_2": JointInfo(
            name="stewart_2",
            min_limit=-0.3,
            max_limit=0.3,
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint 2"
        ),
        "stewart_3": JointInfo(
            name="stewart_3",
            min_limit=-0.3,
            max_limit=0.3,
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint 3"
        ),
        "stewart_4": JointInfo(
            name="stewart_4",
            min_limit=-0.3,
            max_limit=0.3,
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint 4"
        ),
        "stewart_5": JointInfo(
            name="stewart_5",
            min_limit=-0.3,
            max_limit=0.3,
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint 5"
        ),
        "stewart_6": JointInfo(
            name="stewart_6",
            min_limit=-0.3,
            max_limit=0.3,
            safe_amplitude=0.2,
            description="Plateforme Stewart - joint 6"
        ),
    }
    
    # Joints interdits (passifs/bloqués)
    FORBIDDEN_JOINTS: Set[str] = {
        "left_antenna",
        "right_antenna",
        "passive_1",
        "passive_2", 
        "passive_3",
        "passive_4",
        "passive_5",
        "passive_6",
        "passive_7"
    }
    
    # Joints recommandés pour les démos
    RECOMMENDED_JOINTS: Set[str] = {"yaw_body"}
    
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
    def validate_position(cls, joint_name: str, position: float) -> Tuple[bool, float]:
        """
        Valide et clamp une position de joint.
        
        Returns:
            (is_valid, clamped_position)
        """
        if joint_name in cls.FORBIDDEN_JOINTS:
            return False, 0.0
        
        if joint_name not in cls.JOINTS:
            return False, 0.0
        
        joint_info = cls.JOINTS[joint_name]
        
        # Clamp dans les limites du joint
        clamped_pos = max(joint_info.min_limit, min(joint_info.max_limit, position))
        
        # Clamp dans la limite de sécurité
        clamped_pos = max(-joint_info.safe_amplitude, 
                         min(joint_info.safe_amplitude, clamped_pos))
        
        return True, clamped_pos
    
    @classmethod
    def get_all_joints(cls) -> Set[str]:
        """Retourne tous les joints disponibles."""
        return set(cls.JOINTS.keys())
    
    @classmethod
    def get_recommended_joints(cls) -> Set[str]:
        """Retourne les joints recommandés pour les démos."""
        return cls.RECOMMENDED_JOINTS.copy()
    
    @classmethod
    def get_forbidden_joints(cls) -> Set[str]:
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


def validate_joint_position(joint_name: str, position: float) -> Tuple[bool, float]:
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
        print(f"  - {joint_name}: [{joint_info.min_limit:.2f}, {joint_info.max_limit:.2f}] rad")
    
    print(f"\n🚫 Joints interdits: {len(ReachyMapping.get_forbidden_joints())}")
    for joint in ReachyMapping.get_forbidden_joints():
        print(f"  - {joint}")
    
    print(f"\n🎯 Joints recommandés: {ReachyMapping.get_recommended_joints()}")
    
    # Test validation
    print(f"\n🧪 Test validation:")
    test_cases = [
        ("yaw_body", 0.2),    # OK
        ("yaw_body", 0.5),    # Clampé à 0.3
        ("yaw_body", -0.8),   # Clampé à -0.3
        ("left_antenna", 0.1), # Interdit
        ("unknown_joint", 0.1), # Inconnu
    ]
    
    for joint, pos in test_cases:
        try:
            is_valid, clamped = ReachyMapping.validate_position(joint, pos)
            print(f"  {joint}({pos}) → {is_valid}, {clamped:.2f}")
        except ValueError as e:
            print(f"  {joint}({pos}) → ERREUR: {e}")
    
    print(f"\n✅ Mapping Reachy Mini validé !")
