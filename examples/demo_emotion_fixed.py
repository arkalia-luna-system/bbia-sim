#!/usr/bin/env python3
"""
Démo Émotion → Pose BBIA avec MuJoCo direct (version stable)
Version qui fonctionne comme demo_robot_correct.py
"""

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import mujoco
import mujoco.viewer

from bbia_sim.bbia_emotions import BBIAEmotions


def emotion_to_pose(
    emotion: str, intensity: float, step: int, total_steps: int
) -> float:
    """Convertit une émotion BBIA en position d'articulation."""
    # Mapping émotion → mouvement
    emotion_patterns = {
        "happy": (
            lambda t: 0.2 * math.sin(2 * math.pi * 0.1 * t)
        ),  # Mouvement joyeux SÉCURISÉ
        "sad": lambda t: -0.2 * math.sin(2 * math.pi * 0.3 * t),  # Mouvement triste
        "angry": lambda t: 0.4 * math.sin(2 * math.pi * 0.8 * t),  # Mouvement agité
        "surprised": lambda t: 0.5 * math.sin(2 * math.pi * 0.2 * t),  # Mouvement lent
        "neutral": lambda t: 0.1 * math.sin(2 * math.pi * 0.1 * t),  # Mouvement subtil
    }

    t = step / total_steps
    base_movement = emotion_patterns.get(emotion, emotion_patterns["neutral"])(t)

    # Appliquer l'intensité
    return base_movement * intensity


def main():
    parser = argparse.ArgumentParser(
        description="Démo Émotion → Pose BBIA (version stable)"
    )
    parser.add_argument(
        "--emotion",
        default="happy",
        help="Émotion BBIA (happy, sad, angry, surprised, neutral)",
    )
    parser.add_argument(
        "--intensity", type=float, default=0.8, help="Intensité de l'émotion (0.0-1.0)"
    )
    parser.add_argument("--duration", type=int, default=10, help="Durée en secondes")
    parser.add_argument("--joint", default="yaw_body", help="Joint à animer")

    args = parser.parse_args()

    # Validation
    if args.intensity < 0.0 or args.intensity > 1.0:
        print("❌ Intensité doit être entre 0.0 et 1.0")
        return 1

    print("🤖 Démo Émotion → Pose BBIA (version stable)")

    # Charger le modèle
    model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print(f"✅ Modèle chargé: {model.njnt} joints")

    # Vérifier le joint
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, args.joint)
    if joint_id == -1:
        print(f"❌ Joint '{args.joint}' introuvable")
        print("✅ Joints disponibles:")
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            joint_range = model.jnt_range[i]
            if joint_range[0] != joint_range[1]:  # Joint mobile
                print(f"  • {name}")
        return 1

    joint_range = model.jnt_range[joint_id]
    print(f"🎮 Joint sélectionné: {args.joint}")
    print(f"📏 Limites: [{joint_range[0]:6.3f}, {joint_range[1]:6.3f}] rad")

    # Initialiser BBIA Émotions
    emotions = BBIAEmotions()
    if args.emotion not in emotions.emotions:
        print(f"❌ Émotion '{args.emotion}' non supportée")
        print(f"✅ Émotions disponibles : {list(emotions.emotions.keys())}")
        return 1

    # Configuration animation
    fps = 10  # 10 Hz pour animation fluide
    total_steps = args.duration * fps

    print("\n🎭 Configuration BBIA Émotion → Pose :")
    print(f"   • Émotion : {args.emotion}")
    print(f"   • Intensité : {args.intensity}")
    print(f"   • Joint : {args.joint}")
    print(f"   • Durée : {args.duration}s")

    # Lancer le viewer avec animation
    print("🎮 Lancement du viewer MuJoCo...")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        step = 0

        while viewer.is_running() and (time.time() - start_time) < args.duration:
            t = time.time() - start_time

            # Calculer la pose basée sur l'émotion
            angle = emotion_to_pose(args.emotion, args.intensity, step, total_steps)

            # Appliquer l'animation
            data.qpos[joint_id] = angle

            # Step de simulation
            mujoco.mj_step(model, data)

            # Synchroniser avec le viewer
            viewer.sync()

            # Affichage périodique
            if step % 20 == 0:
                print(f"  Step {step:3d} | t={t:5.1f}s | {args.joint}={angle:6.3f} rad")

            step += 1

    print(f"✅ Animation terminée ({step} steps)")

    print("\n🎉 Démo émotion → pose terminée avec succès !")
    print(f"   • Émotion '{args.emotion}' → Joint '{args.joint}'")
    print(f"   • Intensité {args.intensity} → Animation fluide")

    return 0


if __name__ == "__main__":
    exit(main())
