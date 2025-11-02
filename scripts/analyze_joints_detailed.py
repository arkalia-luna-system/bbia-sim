#!/usr/bin/env python3
"""
Script d'analyse des joints MuJoCo pour BBIA Reachy Mini
Extrait et classe tous les joints du modèle officiel
"""

import sys
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent / "src"))

import mujoco


def analyze_joints(xml_path: str):
    """Analyse les joints du modèle MuJoCo et les classe."""
    print(f"🔍 Analyse du modèle: {xml_path}")

    # Charger le modèle
    model = mujoco.MjModel.from_xml_path(xml_path)

    print(f"✅ Modèle chargé: {model.njnt} joints détectés")

    # Classification des joints
    safe_joints = []
    risky_joints = []
    forbidden_joints = []

    print("\n📋 TABLEAU COMPLET DES JOINTS:")
    print("=" * 80)
    print(
        f"{'Nom':<15} {'Type':<8} {'Range (rad)':<20} {'Range (°)':<15} {'Statut':<12}"
    )
    print("=" * 80)

    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_range = model.jnt_range[i]

        # Déterminer le type de joint
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
            joint_type = "hinge"
        elif model.jnt_type[i] == mujoco.mjtJoint.mjJNT_BALL:
            joint_type = "ball"
        elif model.jnt_type[i] == mujoco.mjtJoint.mjJNT_SLIDE:
            joint_type = "slide"
        else:
            joint_type = "other"

        # Conversion en degrés
        range_deg = (
            f"[{joint_range[0]*180/3.14159:.1f}°, {joint_range[1]*180/3.14159:.1f}°]"
        )
        range_rad = f"[{joint_range[0]:.3f}, {joint_range[1]:.3f}]"

        # Classification
        if joint_range[0] == joint_range[1]:  # Joint bloqué
            status = "❌ FORBIDDEN"
            forbidden_joints.append((name, joint_range, "BLOQUÉ"))
        elif name == "yaw_body":  # Joint très sûr
            status = "✅ SAFE"
            safe_joints.append((name, joint_range, "TRÈS SÛR"))
        elif "stewart" in name:  # Joints Stewart - risqués
            range_size = joint_range[1] - joint_range[0]
            if range_size > 2.0:
                status = "⚠️ RISKY"
                risky_joints.append((name, joint_range, "PROBLÉMATIQUE"))
            else:
                status = "✅ SAFE"
                safe_joints.append((name, joint_range, "SÛR"))
        else:
            status = "✅ SAFE"
            safe_joints.append((name, joint_range, "SÛR"))

        print(
            f"{name:<15} {joint_type:<8} {range_rad:<20} {range_deg:<15} {status:<12}"
        )

    print("=" * 80)

    # Résumé
    print("\n📊 RÉSUMÉ DE LA CLASSIFICATION:")
    print(f"✅ JOINTS SÛRS: {len(safe_joints)}")
    for name, _joint_range, desc in safe_joints:
        print(f"   • {name:<15}: {desc}")

    print(f"\n⚠️ JOINTS RISQUÉS: {len(risky_joints)}")
    for name, _joint_range, desc in risky_joints:
        print(f"   • {name:<15}: {desc}")

    print(f"\n❌ JOINTS INTERDITS: {len(forbidden_joints)}")
    for name, _joint_range, desc in forbidden_joints:
        print(f"   • {name:<15}: {desc}")

    return {
        "safe": safe_joints,
        "risky": risky_joints,
        "forbidden": forbidden_joints,
        "total": model.njnt,
    }


if __name__ == "__main__":
    xml_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    results = analyze_joints(xml_path)
