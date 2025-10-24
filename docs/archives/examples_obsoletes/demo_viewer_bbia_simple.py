#!/usr/bin/env python3
"""
Démo 3D BBIA - Visualisation Reachy Mini avec animation SIMPLIFIÉE
Version qui fonctionne parfaitement en mode graphique
"""

import argparse
import math
import sys
import time
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import mujoco
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(
        description="Démo 3D BBIA Reachy Mini - Version Simplifiée"
    )
    parser.add_argument(
        "--xml",
        default="src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml",
        help="Chemin vers le modèle XML MuJoCo",
    )
    parser.add_argument("--joint", default="left_antenna", help="Nom du joint à animer")
    parser.add_argument(
        "--duration", type=int, default=10, help="Durée de l'animation en secondes"
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=1.0,
        help="Fréquence de l'animation sinusoïdale en Hz",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.5,
        help="Amplitude de l'animation en radians",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Mode headless (sans interface graphique)",
    )

    args = parser.parse_args()

    try:
        print("🤖 Initialisation du simulateur BBIA...")
        print(f"📁 Modèle: {args.xml}")

        # Charger le modèle MuJoCo directement
        model = mujoco.MjModel.from_xml_path(args.xml)
        data = mujoco.MjData(model)

        # Vérifier que le joint existe
        joint_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(model.njnt)
        ]
        if args.joint not in joint_names:
            print(f"❌ Joint '{args.joint}' non trouvé!")
            print(f"📋 Joints disponibles: {', '.join(joint_names)}")
            return 1

        print(f"✅ Modèle chargé avec {model.njnt} joints")
        print(
            f"🎮 Lancement {'headless' if args.headless else 'viewer 3D'} pour {args.duration}s..."
        )
        print(f"📋 Joint animé: {args.joint}")
        print(f"🌊 Fréquence: {args.frequency} Hz, Amplitude: {args.amplitude} rad")

        # Trouver l'ID du joint
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, args.joint)

        if args.headless:
            # Mode headless
            start_time = time.time()
            step_count = 0

            while time.time() - start_time < args.duration:
                t = time.time() - start_time
                angle = args.amplitude * math.sin(2 * math.pi * args.frequency * t)

                # Appliquer directement à qpos
                data.qpos[joint_id] = angle
                mujoco.mj_step(model, data)

                step_count += 1
                if step_count % 100 == 0:
                    print(
                        f"Step {step_count:4d} | t={t:5.2f}s | {args.joint}={angle:6.3f} rad"
                    )

                time.sleep(0.01)  # 100 Hz

            print(f"✅ Animation headless terminée ({step_count} steps)")
        else:
            # Mode graphique avec viewer MuJoCo
            print("🎮 Lancement du viewer MuJoCo...")

            with mujoco.viewer.launch_passive(model, data) as viewer:
                start_time = time.time()
                step_count = 0

                while (
                    viewer.is_running() and (time.time() - start_time) < args.duration
                ):
                    t = time.time() - start_time
                    angle = args.amplitude * math.sin(2 * math.pi * args.frequency * t)

                    # Appliquer l'animation
                    data.qpos[joint_id] = angle
                    mujoco.mj_step(model, data)

                    # Synchroniser avec le viewer
                    viewer.sync()

                    step_count += 1

                    # Affichage périodique
                    if step_count % 100 == 0:
                        print(
                            f"Step {step_count:4d} | t={t:5.2f}s | {args.joint}={angle:6.3f} rad"
                        )

                print(f"✅ Animation graphique terminée ({step_count} steps)")

        return 0

    except Exception as e:
        print(f"❌ Erreur: {e}")
        if "mjpython" in str(e) and sys.platform == "darwin":
            print("💡 Sur macOS, essayez: mjpython examples/demo_viewer_bbia_simple.py")
        return 1


if __name__ == "__main__":
    sys.exit(main())
