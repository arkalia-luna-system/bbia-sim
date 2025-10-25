#!/usr/bin/env python3
"""
Script de vérification des joints Reachy Mini
Utilise ce script pour vérifier quels joints peuvent bouger avant de créer des animations.
"""

import sys
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import mujoco as mj


def check_joints():
    """Vérifie les limites de tous les joints du modèle Reachy Mini."""

    model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"

    print("🔍 Vérification des joints Reachy Mini...")
    print(f"📁 Modèle: {model_path}")
    print("=" * 60)

    try:
        model = mj.MjModel.from_xml_path(model_path)

        mobile_joints = []
        blocked_joints = []

        print(f"📊 Total des joints: {model.njnt}")
        print()

        for i in range(model.njnt):
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, i)
            joint_range = model.jnt_range[i]

            if joint_range[0] != joint_range[1]:
                mobile_joints.append((name, joint_range))
                print(
                    f"✅ {name:15} | [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad | MOBILE"
                )
            else:
                blocked_joints.append((name, joint_range))
                print(
                    f"❌ {name:15} | [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad | BLOQUÉ"
                )

        print()
        print("=" * 60)
        print("📈 Résumé:")
        print(f"   ✅ Joints mobiles: {len(mobile_joints)}")
        print(f"   ❌ Joints bloqués: {len(blocked_joints)}")

        print()
        print("🎯 RECOMMANDATIONS:")
        print("   • Utilisez 'yaw_body' pour les animations principales")
        print("   • Les antennes (left_antenna, right_antenna) sont BLOQUÉES")
        print("   • Les joints passifs (passive_1-7) sont BLOQUÉES")
        print("   • La plateforme Stewart (stewart_1-6) est mobile mais complexe")

        print()
        print("🚀 COMMANDES RECOMMANDÉES:")
        print("   mjpython examples/demo_robot_correct.py")
        print("   mjpython examples/test_all_joints.py")

        return mobile_joints, blocked_joints

    except Exception as e:
        print(f"❌ Erreur lors de la vérification: {e}")
        return [], []


def suggest_animation_params(joint_name, mobile_joints):
    """Suggère des paramètres d'animation sûrs pour un joint."""

    for name, joint_range in mobile_joints:
        if name == joint_name:
            min_range, max_range = joint_range
            safe_amplitude = (max_range - min_range) * 0.2  # 20% de la plage

            print(f"🎮 Paramètres suggérés pour '{joint_name}':")
            print(f"   • Limites: [{min_range:.3f}, {max_range:.3f}] rad")
            print(f"   • Amplitude sûre: {safe_amplitude:.3f} rad")
            print("   • Fréquence recommandée: 0.1 Hz (SÉCURISÉ)")
            print(
                f"   • Commande: mjpython examples/demo_viewer_bbia_simple.py --joint {joint_name} --amplitude {safe_amplitude:.3f}"
            )
            return

    print(f"❌ Joint '{joint_name}' non trouvé ou bloqué")


if __name__ == "__main__":
    mobile_joints, blocked_joints = check_joints()

    if len(sys.argv) > 1:
        joint_name = sys.argv[1]
        suggest_animation_params(joint_name, mobile_joints)
