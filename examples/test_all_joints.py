#!/usr/bin/env python3
"""
Démo 3D BBIA - Test de TOUS les joints mobiles (VERSION SÉCURISÉE)
Version qui teste chaque joint mobile un par un avec sécurité renforcée
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
    print("🤖 Test de TOUS les joints mobiles du Reachy Mini (VERSION SÉCURISÉE)")
    print("⚠️  Version avec sécurité renforcée pour éviter les problèmes")

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

    # Tester chaque joint mobile avec sécurité
    for joint_name, joint_id, joint_range in movable_joints:
        print(f"\n🎯 Test du joint: {joint_name}")
        print(f"📏 Limites: [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad")

        # Amplitude TRÈS sûre (3% de la plage, max 0.1 rad)
        safe_range = (joint_range[1] - joint_range[0]) * 0.03
        amplitude = min(safe_range, 0.1)  # Max 0.1 rad pour éviter les problèmes

        print(f"🌊 Amplitude: {amplitude:.3f} rad (TRÈS SÛRE)")
        print("🎮 Lancement du viewer...")

        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                start_time = time.time()
                step_count = 0

                # Durée réduite pour éviter les problèmes
                duration = 2 if joint_name == "yaw_body" else 1.5

                while viewer.is_running() and (time.time() - start_time) < duration:
                    t = time.time() - start_time

                    # Animation sinusoïdale très lente (0.3 Hz)
                    angle = amplitude * math.sin(2 * math.pi * 0.3 * t)

                    # Appliquer l'animation
                    data.qpos[joint_id] = angle

                    # Step de simulation
                    mujoco.mj_step(model, data)

                    # Synchroniser avec le viewer
                    viewer.sync()

                    step_count += 1

                    # Affichage toutes les 1 seconde
                    if step_count % 100 == 0:
                        print(f"  t={t:3.1f}s | {joint_name}={angle:6.3f} rad")

            print(f"✅ Test de {joint_name} terminé avec succès")

        except Exception as e:
            print(f"❌ Erreur avec {joint_name}: {e}")
            print("⚠️  Ce joint peut être problématique, ignoré")

        # Attendre avant le prochain joint
        time.sleep(1)

    print("\n🎉 Tests terminés - Version sécurisée")
    print("💡 Pour des tests encore plus sûrs: mjpython examples/test_safe_joints.py")


if __name__ == "__main__":
    main()
