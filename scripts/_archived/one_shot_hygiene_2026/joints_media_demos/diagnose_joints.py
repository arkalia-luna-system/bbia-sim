#!/usr/bin/env python3
"""Script de diagnostic des joints Reachy Mini
Identifie les joints sûrs vs problématiques pour éviter les erreurs
"""

import sys
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import mujoco


def analyze_joints():
    """Analyse tous les joints et les classe par sécurité."""
    model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    model = mujoco.MjModel.from_xml_path(model_path)

    print("🔍 DIAGNOSTIC DES JOINTS REACHY MINI")
    print("=" * 50)

    safe_joints = []
    problematic_joints = []
    blocked_joints = []

    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_range = model.jnt_range[i]

        if joint_range[0] == joint_range[1]:
            blocked_joints.append((name, joint_range))
        else:
            # Calculer la plage de mouvement
            range_size = joint_range[1] - joint_range[0]

            # Classification par sécurité
            if name == "yaw_body":
                safe_joints.append((name, joint_range, "TRÈS SÛR - Rotation corps"))
            elif name.startswith("stewart_"):
                # Les joints Stewart sont plus complexes
                if range_size > 2.0:  # Grande plage = potentiellement problématique
                    problematic_joints.append(
                        (name, joint_range, "PROBLÉMATIQUE - Grande plage"),
                    )
                else:
                    safe_joints.append((name, joint_range, "SÛR - Plage modérée"))
            else:
                safe_joints.append((name, joint_range, "SÛR - Joint standard"))

    print("📊 RÉSULTATS DU DIAGNOSTIC:")
    print(f"   ✅ Joints sûrs: {len(safe_joints)}")
    print(f"   ⚠️  Joints problématiques: {len(problematic_joints)}")
    print(f"   ❌ Joints bloqués: {len(blocked_joints)}")

    print(f"\n✅ JOINTS SÛRS ({len(safe_joints)}):")
    for name, joint_range, reason in safe_joints:
        range_size = joint_range[1] - joint_range[0]
        print(
            f"   • {name:12} | [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] | {range_size:5.3f} rad | {reason}",
        )

    if problematic_joints:
        print(f"\n⚠️  JOINTS PROBLÉMATIQUES ({len(problematic_joints)}):")
        for name, joint_range, reason in problematic_joints:
            range_size = joint_range[1] - joint_range[0]
            print(
                f"   • {name:12} | [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] | {range_size:5.3f} rad | {reason}",
            )

    print(f"\n❌ JOINTS BLOQUÉS ({len(blocked_joints)}):")
    for name, joint_range in blocked_joints:
        print(
            f"   • {name:12} | [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] | BLOQUÉ",
        )

    print("\n🎯 RECOMMANDATIONS:")
    print("   • Pour les démos: Utilisez SEULEMENT les joints sûrs")
    print("   • Joint le plus sûr: yaw_body (rotation du corps)")
    print("   • Évitez les joints problématiques en production")
    print("   • Les joints bloqués ne peuvent pas bouger")

    print("\n🚀 COMMANDES RECOMMANDÉES:")
    print("   mjpython examples/demo_robot_correct.py          # Démo principale")
    print("   mjpython examples/test_safe_joints.py           # Test joints sûrs")
    print("   mjpython examples/test_all_joints.py            # Test tous (sécurisé)")

    return safe_joints, problematic_joints, blocked_joints


if __name__ == "__main__":
    analyze_joints()
