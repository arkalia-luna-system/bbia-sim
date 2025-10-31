#!/usr/bin/env python3
"""
Démo Émotion → Pose : BBIA Émotions anime le robot
Vertical slice : Émotion → Articulation → Animation visible
Utilise RobotAPI pour backend unifié
"""

import argparse
import math
import sys
import time
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_emotions import BBIAEmotions
from bbia_sim.robot_factory import RobotFactory
from scripts.replay_viewer import AnimationRecorder


def emotion_to_pose(
    emotion: str, intensity: float, step: int, total_steps: int
) -> float:
    """Convertit une émotion BBIA en position d'articulation."""
    # PERSONNALITÉ BBIA: Mix bébé + humain + chien + IA
    # Mouvements expressifs, variés et beaucoup plus longs qu'avant
    # Animations 2x plus longues avec patterns variés

    t = step / total_steps

    # OPTIMISATION EXPERTE: Amplitudes conformes SDK (max 0.3 rad), conservatrices pour éviter dépassement
    emotion_patterns = {
        "happy": (
            lambda t: 0.12
            * math.sin(2 * math.pi * 0.1 * t)
            * (1 + 0.5 * math.sin(4 * math.pi * t))
        ),  # Joyeux + oscillations (max 0.18 rad)
        "sad": (
            lambda t: -0.12 * math.sin(2 * math.pi * 0.3 * t)
            - 0.04 * math.sin(6 * math.pi * t)
        ),  # Triste + tremblements (max 0.16 rad)
        "angry": (
            lambda t: 0.18 * math.sin(2 * math.pi * 0.8 * t)
            + 0.04 * math.sin(8 * math.pi * t)
        ),  # Agité + rapide (max 0.22 rad < 0.3)
        "surprised": (
            lambda t: 0.15 * math.sin(2 * math.pi * 0.2 * t) * math.cos(3 * math.pi * t)
        ),  # Surprise complexe (max 0.15 rad)
        "neutral": (
            lambda t: 0.08 * math.sin(2 * math.pi * 0.1 * t)
            + 0.03 * math.sin(6 * math.pi * t)
        ),  # Subtile + micro-mouvements (max 0.11 rad)
    }

    base_movement = emotion_patterns.get(emotion, emotion_patterns["neutral"])(t)

    # Appliquer l'intensité
    return base_movement * intensity


def main():
    parser = argparse.ArgumentParser(description="Démo Émotion → Pose BBIA")
    parser.add_argument(
        "--emotion",
        default="happy",
        help="Émotion BBIA (happy, sad, angry, surprised, neutral)",
    )
    parser.add_argument(
        "--intensity", type=float, default=0.8, help="Intensité de l'émotion (0.0-1.0)"
    )
    parser.add_argument("--duration", type=int, default=20, help="Durée en secondes")
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument("--joint", default="yaw_body", help="Joint à animer")
    parser.add_argument("--backend", default="mujoco", help="Backend (mujoco, reachy)")
    parser.add_argument(
        "--record", help="Enregistrer l'animation dans ce fichier (.jsonl)"
    )

    args = parser.parse_args()

    # Validation
    if args.intensity < 0.0 or args.intensity > 1.0:
        print("❌ Intensité doit être entre 0.0 et 1.0")
        return 1

    # 1. Créer le backend RobotAPI
    robot = RobotFactory.create_backend(args.backend)
    if not robot:
        print(f"❌ Impossible de créer le backend {args.backend}")
        return 1

    # 2. Connecter au robot/simulateur
    if not robot.connect():
        print(f"❌ Impossible de se connecter au backend {args.backend}")
        return 1

    print(
        f"✅ Backend {args.backend} connecté : {len(robot.get_available_joints())} joints détectés"
    )

    # 2.1. Lancer le viewer MuJoCo si nécessaire
    if args.backend == "mujoco" and not args.headless:
        print("🖥️ Lancement du viewer MuJoCo...")
        if robot.launch_viewer(passive=True):
            # Configurer la caméra à 180° (face optimal) immédiatement
            if hasattr(robot, "viewer") and robot.viewer is not None:
                robot.viewer.cam.azimuth = 180.0
                robot.viewer.cam.elevation = -15.0
                robot.viewer.cam.distance = 1.2  # Rapproché de 20%
                robot.viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
                robot.viewer.sync()
        else:
            print("⚠️ Viewer MuJoCo non lancé, mais démo continue")

    # 3. Vérifier le joint
    available_joints = robot.get_available_joints()
    if args.joint not in available_joints:
        print(f"❌ Joint '{args.joint}' introuvable")
        print(f"✅ Joints disponibles : {available_joints}")
        robot.disconnect()
        return 1

    # 4. Initialiser BBIA Émotions
    emotions = BBIAEmotions()
    if args.emotion not in emotions.emotions:
        print(f"❌ Émotion '{args.emotion}' non supportée")
        print(f"✅ Émotions disponibles : {list(emotions.emotions.keys())}")
        robot.disconnect()
        return 1

    # 5. Configuration animation
    fps = 10  # 10 Hz pour animation fluide
    total_steps = args.duration * fps

    print("\n🎭 Configuration BBIA Émotion → Pose :")
    print(f"   • Backend : {args.backend}")
    print(f"   • Émotion : {args.emotion}")
    print(f"   • Intensité : {args.intensity}")
    print(f"   • Joint : {args.joint}")
    print(f"   • Durée : {args.duration}s")
    print(f"   • Mode : {'headless' if args.headless else 'graphique'}")

    # 6. Initialiser l'enregistreur si demandé
    recorder = None
    if args.record:
        recorder = AnimationRecorder(args.record)
        recorder.start_recording()
        print(f"🎬 Enregistrement activé: {args.record}")

    # 7. Animation
    print("\n🚀 Démarrage animation émotion → pose...")

    try:
        start_time = time.time()  # noqa: F823
        for step in range(total_steps):
            # Calculer la pose basée sur l'émotion
            angle = emotion_to_pose(args.emotion, args.intensity, step, total_steps)

            # Appliquer la pose via RobotAPI
            robot.set_joint_pos(args.joint, angle)
            robot.step()

            # Synchroniser avec le viewer si MuJoCo
            if args.backend == "mujoco":
                robot.sync_viewer()

            # Enregistrer la frame si demandé
            if recorder:
                recorder.record_frame(args.joint, angle, step)

            # Log périodique
            if step % 20 == 0:
                elapsed = time.time() - start_time
                current_pos = robot.get_joint_pos(args.joint)
                print(
                    f"  Step {step:3d} | t={elapsed:3.1f}s | {args.joint}={current_pos:6.3f} rad"
                )

        print(f"✅ Animation terminée ({total_steps} steps)")

    except Exception as e:
        print(f"❌ Erreur animation : {e}")
        robot.disconnect()
        return 1

    finally:
        # 8. Arrêter l'enregistrement
        if recorder:
            frames_recorded = recorder.stop_recording()
            print(f"💾 {frames_recorded} frames enregistrées")

        # 9. Déconnexion
        robot.disconnect()

    print("\n🎉 Démo émotion → pose terminée avec succès !")
    print(
        f"   • Backend {args.backend} → Émotion '{args.emotion}' → Joint '{args.joint}'"
    )
    print(f"   • Intensité {args.intensity} → Animation fluide")

    # Garder le viewer ouvert quelques secondes pour voir le résultat final
    if args.backend == "mujoco" and not args.headless:
        if hasattr(robot, "viewer") and robot.viewer is not None:
            print("\n⏸️  Viewer ouvert encore 3 secondes...")
            import time

            time.sleep(3)

    return 0


if __name__ == "__main__":
    exit(main())
