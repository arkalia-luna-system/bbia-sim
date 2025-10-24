#!/usr/bin/env python3
"""
Démo 3D BBIA - Visualisation Reachy Mini avec animation
Utilise le simulateur BBIA existant pour animer les joints
"""

import argparse
import math
import sys
import time
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.sim.simulator import MuJoCoSimulator


def main():
    parser = argparse.ArgumentParser(description="Démo 3D BBIA Reachy Mini")
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
        # Initialisation du simulateur BBIA
        print("🤖 Initialisation du simulateur BBIA...")
        print(f"📁 Modèle: {args.xml}")

        simulator = MuJoCoSimulator(args.xml)

        # Vérification que le joint existe
        available_joints = simulator.get_available_joints()
        if args.joint not in available_joints:
            print(f"❌ Joint '{args.joint}' non trouvé!")
            print(f"📋 Joints disponibles: {', '.join(available_joints)}")
            return 1

        print(f"✅ Simulateur initialisé avec {len(available_joints)} joints")
        print(
            f"🎮 Lancement {'headless' if args.headless else 'viewer 3D'} pour {args.duration}s..."
        )
        print(f"📋 Joint animé: {args.joint}")
        print(f"🌊 Fréquence: {args.frequency} Hz, Amplitude: {args.amplitude} rad")

        if args.headless:
            # Mode headless avec animation
            start_time = time.time()
            step_count = 0

            while time.time() - start_time < args.duration:
                t = time.time() - start_time
                angle = args.amplitude * math.sin(2 * math.pi * args.frequency * t)

                simulator.set_joint_position(args.joint, angle)
                simulator._step_simulation()

                step_count += 1
                if step_count % 100 == 0:
                    current_pos = simulator.get_joint_position(args.joint)
                    print(
                        f"Step {step_count:4d} | t={t:5.2f}s | {args.joint}={current_pos:6.3f} rad"
                    )

                time.sleep(0.01)  # 100 Hz

            print(f"✅ Animation headless terminée ({step_count} steps)")
        else:
            # Mode graphique avec animation intégrée
            try:
                print("🎮 Lancement du viewer MuJoCo avec animation...")

                # Créer le viewer manuellement
                import mujoco.viewer

                simulator.viewer = mujoco.viewer.launch_passive(
                    simulator.model, simulator.data
                )

                # Animation intégrée dans la boucle du viewer
                start_time = time.time()
                step_count = 0

                while (
                    simulator.viewer.is_running()
                    and (time.time() - start_time) < args.duration
                ):
                    t = time.time() - start_time
                    angle = args.amplitude * math.sin(2 * math.pi * args.frequency * t)

                    # Appliquer l'animation
                    simulator.set_joint_position(args.joint, angle)
                    simulator._step_simulation()

                    # Synchroniser avec le viewer
                    simulator.viewer.sync()

                    step_count += 1

                    # Affichage périodique
                    if step_count % 100 == 0:
                        current_pos = simulator.get_joint_position(args.joint)
                        print(
                            f"Step {step_count:4d} | t={t:5.2f}s | {args.joint}={current_pos:6.3f} rad"
                        )

                # Fermer le viewer proprement
                simulator.viewer.close()

                print(f"✅ Animation graphique terminée ({step_count} steps)")

            except Exception as e:
                if "mjpython" in str(e) and sys.platform == "darwin":
                    print(
                        "❌ Sur macOS, le viewer MuJoCo nécessite mjpython au lieu de python."
                    )
                    print("💡 Solutions:")
                    print("  • Utilisez: mjpython examples/demo_viewer_bbia.py")
                    print(
                        "  • Ou utilisez: python examples/demo_viewer_bbia.py --headless"
                    )
                    return 1
                else:
                    print(f"❌ Erreur viewer: {e}")
                    print("🔄 Fallback vers mode headless...")
                    # Fallback vers headless
                    start_time = time.time()
                    step_count = 0

                    while time.time() - start_time < args.duration:
                        t = time.time() - start_time
                        angle = args.amplitude * math.sin(
                            2 * math.pi * args.frequency * t
                        )

                        simulator.set_joint_position(args.joint, angle)
                        simulator._step_simulation()

                        step_count += 1
                        if step_count % 100 == 0:
                            current_pos = simulator.get_joint_position(args.joint)
                            print(
                                f"Step {step_count:4d} | t={t:5.2f}s | {args.joint}={current_pos:6.3f} rad"
                            )

                        time.sleep(0.01)

                    print(f"✅ Animation headless terminée ({step_count} steps)")

        return 0

    except Exception as e:
        print(f"❌ Erreur: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
