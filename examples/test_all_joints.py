#!/usr/bin/env python3
"""
Démo 3D BBIA - Test de TOUS les joints mobiles
Version qui teste chaque joint mobile un par un
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
    print("🤖 Test de TOUS les joints mobiles du Reachy Mini")

    # Charger le modèle
    model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Trouver tous les joints mobiles
    movable_joints = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_range = model.jnt_range[i]
        if joint_range[0] != joint_range[1]:  # Joint mobile
            movable_joints.append((name, i, joint_range))

    print(f"✅ {len(movable_joints)} joints mobiles trouvés")

    # Tester chaque joint mobile
    for joint_name, joint_id, joint_range in movable_joints:
        print(f"\n🎯 Test du joint: {joint_name}")
        print(f"📏 Limites: [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad")

        # Amplitude sûre (10% de la plage)
        safe_range = (joint_range[1] - joint_range[0]) * 0.1
        amplitude = min(safe_range, 0.2)  # Max 0.2 rad

        print(f"🌊 Amplitude: {amplitude:.3f} rad")
        print("🎮 Lancement du viewer...")

        with mujoco.viewer.launch_passive(model, data) as viewer:
            start_time = time.time()

            while viewer.is_running() and (time.time() - start_time) < 5:
                t = time.time() - start_time

                # Animation sinusoïdale
                angle = amplitude * math.sin(2 * math.pi * 1.0 * t)

                # Appliquer l'animation
                data.qpos[joint_id] = angle

                # Step de simulation
                mujoco.mj_step(model, data)

                # Synchroniser avec le viewer
                viewer.sync()

                # Affichage
                if int(t * 2) % 2 == 0:
                    print(f"  t={t:3.1f}s | {joint_name}={angle:6.3f} rad")

        print(f"✅ Test de {joint_name} terminé")

        # Attendre un peu avant le prochain joint
        time.sleep(1)

    print("\n🎉 Tests terminés - Quels joints ont bien bougé ?")


if __name__ == "__main__":
    main()
