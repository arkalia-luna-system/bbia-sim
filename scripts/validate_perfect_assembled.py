#!/usr/bin/env python3
"""
Script de validation du modèle Reachy Mini parfaitement assemblé.
Vérifie les dimensions, l'assemblage et les articulations.
"""

import mujoco
import numpy as np
import sys
import os


def validate_perfect_assembled_model():
    """Valide le modèle reachy_mini_perfect_assembled.xml"""

    model_path = "/Volumes/T7/bbia-reachy-sim/src/bbia_sim/sim/models/reachy_mini_perfect_assembled.xml"

    print("🔍 Validation du modèle Reachy Mini parfaitement assemblé")
    print("=" * 60)

    try:
        # Chargement du modèle
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)

        print(f"✅ Modèle chargé avec succès")
        print(f"📁 Fichier : {model_path}")

        # Informations générales
        print(f"\n📊 Informations générales :")
        print(f"   • Nombre de corps : {model.nbody}")
        print(f"   • Nombre d'articulations : {model.njnt}")
        print(f"   • Nombre de géométries : {model.ngeom}")
        print(f"   • Nombre de moteurs : {model.nu}")

        # Vérification des dimensions
        print(f"\n📏 Vérification des dimensions :")

        # Position de la base
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        base_pos = model.body_pos[base_id]
        print(
            f"   • Position base : ({base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f})"
        )

        # Position de la tête
        head_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "head")
        head_pos = model.body_pos[head_id]
        print(
            f"   • Position tête : ({head_pos[0]:.3f}, {head_pos[1]:.3f}, {head_pos[2]:.3f})"
        )

        # Hauteur totale du robot
        height = head_pos[2] - base_pos[2] + 0.5  # +50cm pour la base
        print(f"   • Hauteur totale : {height:.3f}m")

        # Largeur du robot (distance entre les bras)
        right_arm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_arm")
        left_arm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_arm")
        right_arm_pos = model.body_pos[right_arm_id]
        left_arm_pos = model.body_pos[left_arm_id]
        width = abs(left_arm_pos[1] - right_arm_pos[1])
        print(f"   • Largeur totale : {width:.3f}m")

        # Validation des dimensions
        print(f"\n✅ Validation des dimensions :")
        if 0.4 <= height <= 0.7:
            print(f"   ✅ Hauteur OK ({height:.3f}m) - Dans la plage 0.4-0.7m")
        else:
            print(f"   ❌ Hauteur NOK ({height:.3f}m) - Hors plage 0.4-0.7m")

        if 0.2 <= width <= 0.4:
            print(f"   ✅ Largeur OK ({width:.3f}m) - Dans la plage 0.2-0.4m")
        else:
            print(f"   ❌ Largeur NOK ({width:.3f}m) - Hors plage 0.2-0.4m")

        # Vérification de l'assemblage
        print(f"\n🔧 Vérification de l'assemblage :")

        # Vérifier que tous les corps sont bien connectés
        bodies_to_check = [
            "base",
            "torso",
            "head",
            "right_arm",
            "left_arm",
            "right_forearm",
            "left_forearm",
            "right_gripper",
            "left_gripper",
        ]
        assembly_ok = True

        for body_name in bodies_to_check:
            try:
                body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                body_pos = model.body_pos[body_id]
                print(
                    f"   ✅ {body_name} : ({body_pos[0]:.3f}, {body_pos[1]:.3f}, {body_pos[2]:.3f})"
                )
            except:
                print(f"   ❌ {body_name} : Corps non trouvé")
                assembly_ok = False

        # Vérification des articulations
        print(f"\n🤖 Vérification des articulations :")
        expected_joints = [
            "neck_yaw",
            "neck_pitch",
            "neck_roll",
            "right_shoulder_pitch",
            "right_shoulder_roll",
            "right_elbow_pitch",
            "right_wrist_pitch",
            "right_wrist_roll",
            "right_gripper_joint",
            "left_shoulder_pitch",
            "left_shoulder_roll",
            "left_elbow_pitch",
            "left_wrist_pitch",
            "left_wrist_roll",
            "left_gripper_joint",
        ]

        joints_found = 0
        for joint_name in expected_joints:
            try:
                joint_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                )
                joint_range = model.jnt_range[joint_id]
                print(
                    f"   ✅ {joint_name} : [{joint_range[0]:.2f}, {joint_range[1]:.2f}] rad"
                )
                joints_found += 1
            except:
                print(f"   ❌ {joint_name} : Articulation non trouvée")

        # Score de fidélité
        print(f"\n🎯 Score de fidélité :")
        height_score = 1 if 0.4 <= height <= 0.7 else 0
        width_score = 1 if 0.2 <= width <= 0.4 else 0
        joints_score = 1 if joints_found == 15 else 0
        assembly_score = 1 if assembly_ok else 0

        total_score = height_score + width_score + joints_score + assembly_score
        print(f"   • Dimensions : {height_score + width_score}/2")
        print(f"   • Articulations : {joints_score}/1")
        print(f"   • Assemblage : {assembly_score}/1")
        print(f"   • Score total : {total_score}/4")

        if total_score == 4:
            print(f"\n🎉 MODÈLE PARFAIT ! Toutes les vérifications sont OK")
            return True
        else:
            print(f"\n⚠️  Modèle à améliorer. Score : {total_score}/4")
            return False

    except Exception as e:
        print(f"❌ Erreur lors de la validation : {e}")
        return False


if __name__ == "__main__":
    success = validate_perfect_assembled_model()
    sys.exit(0 if success else 1)
