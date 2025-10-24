#!/usr/bin/env python3
"""
Démo 3D BBIA - Visualisation Reachy Mini avec animation CORRIGÉE
Utilise le simulateur BBIA existant pour animer les joints de manière stable
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


def get_safe_joints(model):
    """Retourne la liste des joints sûrs pour l'animation."""
    safe_joints = []
    problematic_joints = []
    blocked_joints = []

    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_range = model.jnt_range[i]

        if joint_range[0] == joint_range[1]:  # Joint bloqué
            blocked_joints.append(name)
        elif name == "yaw_body":  # Joint très sûr
            safe_joints.append((name, joint_range, "TRÈS SÛR"))
        elif "stewart" in name:  # Joints Stewart - risqués
            range_size = joint_range[1] - joint_range[0]
            if range_size > 2.0:
                problematic_joints.append((name, joint_range, "PROBLÉMATIQUE"))
            else:
                safe_joints.append((name, joint_range, "SÛR"))
        else:
            safe_joints.append((name, joint_range, "SÛR"))

    return safe_joints, problematic_joints, blocked_joints


def main():
    parser = argparse.ArgumentParser(
        description="Démo 3D BBIA Reachy Mini - Version Corrigée"
    )
    parser.add_argument(
        "--xml",
        default="src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml",
        help="Chemin vers le modèle XML MuJoCo",
    )
    parser.add_argument("--joint", default="yaw_body", help="Nom du joint à animer")
    parser.add_argument(
        "--duration", type=int, default=10, help="Durée de l'animation en secondes"
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=0.5,
        help="Fréquence de l'animation sinusoïdale en Hz",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.3,
        help="Amplitude de l'animation en radians (max recommandé: 0.3)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Mode headless (sans interface graphique)",
    )
    parser.add_argument(
        "--list-joints",
        action="store_true",
        help="Lister tous les joints et leurs statuts",
    )

    args = parser.parse_args()

    try:
        # Charger le modèle MuJoCo
        print("🤖 Chargement du modèle MuJoCo...")
        print(f"📁 Modèle: {args.xml}")

        model = mujoco.MjModel.from_xml_path(args.xml)
        data = mujoco.MjData(model)

        print(f"✅ Modèle chargé: {model.njnt} joints détectés")

        # Analyser les joints
        safe_joints, problematic_joints, blocked_joints = get_safe_joints(model)

        if args.list_joints:
            print("\n📋 ANALYSE DES JOINTS:")
            print(f"\n✅ JOINTS SÛRS ({len(safe_joints)}):")
            for name, joint_range, status in safe_joints:
                print(
                    f"  • {name:15s}: [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad - {status}"
                )

            print(f"\n⚠️  JOINTS PROBLÉMATIQUES ({len(problematic_joints)}):")
            for name, joint_range, status in problematic_joints:
                print(
                    f"  • {name:15s}: [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad - {status}"
                )

            print(f"\n❌ JOINTS BLOQUÉS ({len(blocked_joints)}):")
            for name in blocked_joints:
                print(f"  • {name:15s}: BLOQUÉ")

            print("\n🎯 RECOMMANDATION: Utilisez 'yaw_body' pour une animation sûre")
            return 0

        # Vérifier que le joint demandé est valide
        joint_names = [name for name, _, _ in safe_joints]
        if args.joint not in joint_names:
            print(f"❌ Joint '{args.joint}' non trouvé ou non sûr!")
            print(f"📋 Joints sûrs disponibles: {', '.join(joint_names)}")
            print("💡 Utilisez --list-joints pour voir tous les joints")
            return 1

        # Trouver l'ID du joint
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, args.joint)
        joint_range = model.jnt_range[joint_id]

        # Vérifier l'amplitude
        max_safe_amplitude = min((joint_range[1] - joint_range[0]) * 0.2, 0.3)
        if args.amplitude > max_safe_amplitude:
            print(f"⚠️  Amplitude {args.amplitude} rad trop élevée!")
            print(f"💡 Amplitude sûre recommandée: {max_safe_amplitude:.3f} rad")
            args.amplitude = max_safe_amplitude
            print(f"🔧 Amplitude ajustée à: {args.amplitude:.3f} rad")

        print("\n🎮 Configuration de l'animation:")
        print(f"   • Joint: {args.joint}")
        print(f"   • Durée: {args.duration}s")
        print(f"   • Fréquence: {args.frequency} Hz")
        print(f"   • Amplitude: {args.amplitude:.3f} rad")
        print(f"   • Mode: {'headless' if args.headless else 'viewer 3D'}")

        if args.headless:
            # Mode headless avec animation
            print("\n🚀 Démarrage animation headless...")
            start_time = time.time()
            step_count = 0

            while time.time() - start_time < args.duration:
                t = time.time() - start_time
                angle = args.amplitude * math.sin(2 * math.pi * args.frequency * t)

                # Clamp l'angle dans les limites du joint
                angle = max(joint_range[0], min(joint_range[1], angle))
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
            # Mode graphique avec animation intégrée
            print("\n🎮 Lancement du viewer MuJoCo avec animation...")

            try:
                with mujoco.viewer.launch_passive(model, data) as viewer:
                    start_time = time.time()
                    step_count = 0

                    print("✅ Viewer lancé - Animation en cours...")
                    print("💡 Fermez la fenêtre pour arrêter l'animation")

                    while (
                        viewer.is_running()
                        and (time.time() - start_time) < args.duration
                    ):
                        t = time.time() - start_time
                        angle = args.amplitude * math.sin(
                            2 * math.pi * args.frequency * t
                        )

                        # Clamp l'angle dans les limites du joint
                        angle = max(joint_range[0], min(joint_range[1], angle))
                        data.qpos[joint_id] = angle

                        mujoco.mj_step(model, data)
                        viewer.sync()
                        step_count += 1

                        if step_count % 100 == 0:
                            print(
                                f"Step {step_count:4d} | t={t:5.2f}s | {args.joint}={angle:6.3f} rad"
                            )

                    print(f"✅ Animation terminée ({step_count} steps)")

            except Exception as e:
                error_msg = str(e)
                if "mjpython" in error_msg.lower():
                    print(f"❌ Erreur viewer graphique: {error_msg}")
                    print(
                        "💡 Sur macOS, utilisez: mjpython examples/demo_viewer_bbia_corrected.py"
                    )
                    print("💡 Ou essayez le mode headless: --headless")
                else:
                    print(f"❌ Erreur viewer graphique: {e}")
                    print("💡 Essayez le mode headless: --headless")
                return 1

        print("\n🎉 Démonstration terminée avec succès!")
        return 0

    except FileNotFoundError:
        print(f"❌ Fichier modèle introuvable: {args.xml}")
        print("💡 Vérifiez le chemin du fichier XML")
        return 1
    except Exception as e:
        print(f"❌ Erreur: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
