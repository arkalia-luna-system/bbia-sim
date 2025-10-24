#!/usr/bin/env python3
"""
Test rapide - Robot Reachy Mini en 3D
Version ultra-simple pour vérifier que le robot bouge
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
    print("🤖 Test rapide - Robot Reachy Mini en 3D")

    # Charger le modèle
    model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print(f"✅ Modèle chargé: {model.njnt} joints")

    # Lister les joints disponibles
    joints = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in range(model.njnt)
    ]
    print(f"📋 Joints: {', '.join(joints[:5])}...")

    # Trouver l'ID de left_antenna
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_antenna")
    print(f"🎯 Animation du joint: left_antenna (ID: {joint_id})")

    # Lancer le viewer avec animation
    print("🎮 Lancement du viewer MuJoCo...")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < 10:
            t = time.time() - start_time

            # Animation sinusoïdale simple
            angle = 0.5 * math.sin(2 * math.pi * 1.0 * t)
            data.qpos[joint_id] = angle

            # Step de simulation
            mujoco.mj_step(model, data)

            # Synchroniser avec le viewer
            viewer.sync()

            # Affichage périodique
            if int(t * 10) % 10 == 0:  # Toutes les secondes
                print(f"t={t:5.1f}s | left_antenna={angle:6.3f} rad")

    print("✅ Test terminé - Le robot a-t-il bougé ?")


if __name__ == "__main__":
    main()
