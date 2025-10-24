#!/usr/bin/env python3
"""
Démo 3D BBIA - Test SÉCURISÉ des joints mobiles
Version qui teste seulement les joints sûrs (yaw_body + quelques stewart)
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
    print("🤖 Test SÉCURISÉ des joints mobiles du Reachy Mini")
    print("⚠️  Seuls les joints sûrs sont testés pour éviter les problèmes")

    # Charger le modèle
    model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Joints sûrs à tester (ordre de sécurité)
    safe_joints = [
        ("yaw_body", "Rotation du corps - LE PLUS SÛR"),
        ("stewart_1", "Stewart 1 - Test prudent"),
        ("stewart_3", "Stewart 3 - Test prudent"),
    ]

    print(f"✅ {len(safe_joints)} joints sûrs seront testés")

    # Tester chaque joint sûr
    for joint_name, description in safe_joints:
        print(f"\n🎯 Test du joint: {joint_name}")
        print(f"📋 Description: {description}")

        # Trouver l'ID du joint
        joint_id = None
        joint_range = None
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name == joint_name:
                joint_id = i
                joint_range = model.jnt_range[i]
                break

        if joint_id is None:
            print(f"❌ Joint {joint_name} non trouvé, ignoré")
            continue

        print(f"📏 Limites: [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad")

        # Amplitude TRÈS sûre (5% de la plage, max 0.1 rad)
        safe_range = (joint_range[1] - joint_range[0]) * 0.05
        amplitude = min(safe_range, 0.1)  # Max 0.1 rad pour être sûr

        print(f"🌊 Amplitude: {amplitude:.3f} rad (TRÈS SÛRE)")
        print("🎮 Lancement du viewer...")

        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                start_time = time.time()
                step_count = 0

                while (
                    viewer.is_running() and (time.time() - start_time) < 3
                ):  # 3s seulement
                    t = time.time() - start_time

                    # Animation sinusoïdale très lente
                    angle = amplitude * math.sin(2 * math.pi * 0.5 * t)  # 0.5 Hz

                    # Appliquer l'animation
                    data.qpos[joint_id] = angle

                    # Step de simulation
                    mujoco.mj_step(model, data)

                    # Synchroniser avec le viewer
                    viewer.sync()

                    step_count += 1

                    # Affichage toutes les 2 secondes
                    if step_count % 200 == 0:
                        print(f"  t={t:3.1f}s | {joint_name}={angle:6.3f} rad")

            print(f"✅ Test de {joint_name} terminé avec succès")

        except Exception as e:
            print(f"❌ Erreur avec {joint_name}: {e}")
            print("⚠️  Ce joint peut être problématique, ignoré")

        # Attendre avant le prochain joint
        time.sleep(2)

    print("\n🎉 Tests sécurisés terminés")
    print("💡 Pour des tests complets, utilisez: python scripts/check_joints.py")


if __name__ == "__main__":
    main()
