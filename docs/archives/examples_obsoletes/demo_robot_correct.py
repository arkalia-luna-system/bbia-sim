#!/usr/bin/env python3
"""
Démo 3D BBIA - Robot Reachy Mini avec les BONS joints
Version qui utilise les joints qui peuvent vraiment bouger
"""

import sys
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import math
import time

import mujoco
import mujoco.viewer


def main():
    print("🤖 Démo 3D BBIA - Robot Reachy Mini avec les BONS joints")

    # Charger le modèle
    model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print(f"✅ Modèle chargé: {model.njnt} joints")

    # Lister les joints avec leurs limites
    print("📋 Joints disponibles avec limites:")
    movable_joints = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_range = model.jnt_range[i]
        if joint_range[0] != joint_range[1]:  # Joint mobile
            movable_joints.append((name, joint_range))
            print(
                f"  ✅ {name:15s}: [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad"
            )
        else:
            print(f"  ❌ {name:15s}: BLOQUÉ")

    print(f"\n🎯 Joints mobiles: {len(movable_joints)}")

    # Utiliser yaw_body (rotation du corps) - le plus visible
    joint_name = "yaw_body"
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    joint_range = model.jnt_range[joint_id]

    print(f"🎮 Animation du joint: {joint_name}")
    print(f"📏 Limites: [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad")

    # Calculer une amplitude sûre (20% de la plage)
    safe_range = (joint_range[1] - joint_range[0]) * 0.2
    amplitude = min(safe_range, 0.3)  # Max 0.3 rad pour être sûr

    print(f"🌊 Amplitude sûre: {amplitude:.3f} rad")

    # Lancer le viewer avec animation
    print("🎮 Lancement du viewer MuJoCo...")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < 15:
            t = time.time() - start_time

            # Animation sinusoïdale dans les limites sûres
            angle = amplitude * math.sin(
                2 * math.pi * 0.5 * t
            )  # 0.5 Hz pour être visible

            # Appliquer l'animation
            data.qpos[joint_id] = angle

            # Step de simulation
            mujoco.mj_step(model, data)

            # Synchroniser avec le viewer
            viewer.sync()

            # Affichage périodique
            if int(t * 2) % 2 == 0:  # Toutes les secondes
                print(f"t={t:5.1f}s | {joint_name}={angle:6.3f} rad")

    print("✅ Animation terminée - Le robot a-t-il tourné correctement ?")


if __name__ == "__main__":
    main()
