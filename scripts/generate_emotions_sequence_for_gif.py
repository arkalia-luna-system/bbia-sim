#!/usr/bin/env python3
"""
Script pour générer séquence d'émotions pour GIF
Enchaîne automatiquement : happy (2s) → neutral (1s) → curious (2s) → excited (2s) → calm (1s)
Total : 8 secondes, prêt pour enregistrement écran
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.bbia_emotions import BBIAEmotions
from bbia_sim.robot_factory import RobotFactory


def emotion_to_pose(emotion: str, intensity: float, step: int, total_steps: int) -> float:
    """Convertit une émotion en position d'articulation."""
    t = step / total_steps

    emotion_patterns = {
        "happy": lambda t: 0.12 * math.sin(2 * math.pi * 0.1 * t) * (1 + 0.5 * math.sin(4 * math.pi * t)),
        "sad": lambda t: -0.12 * math.sin(2 * math.pi * 0.3 * t) - 0.04 * math.sin(6 * math.pi * t),
        "angry": lambda t: 0.18 * math.sin(2 * math.pi * 0.8 * t) + 0.04 * math.sin(8 * math.pi * t),
        "surprised": lambda t: 0.15 * math.sin(2 * math.pi * 0.2 * t) * math.cos(3 * math.pi * t),
        "neutral": lambda t: 0.08 * math.sin(2 * math.pi * 0.1 * t) + 0.03 * math.sin(6 * math.pi * t),
        "curious": lambda t: 0.10 * math.sin(2 * math.pi * 0.15 * t) * (1 + 0.3 * math.sin(3 * math.pi * t)),
        "excited": lambda t: 0.16 * math.sin(2 * math.pi * 0.12 * t) * (1 + 0.8 * math.sin(5 * math.pi * t)),
        "calm": lambda t: 0.06 * math.sin(2 * math.pi * 0.08 * t),
    }

    base_movement = emotion_patterns.get(emotion, emotion_patterns["neutral"])(t)
    return base_movement * intensity


def main():
    # Séquence d'émotions avec durées
    sequence = [
        ("happy", 2.0, 0.8),
        ("neutral", 1.0, 0.5),
        ("curious", 2.0, 0.7),
        ("excited", 2.0, 0.9),
        ("calm", 1.0, 0.6),
    ]

    # Créer le robot
    robot = RobotFactory.create_backend("mujoco")
    if not robot or not robot.connect():
        print("❌ Impossible de se connecter")
        return 1

    print("✅ Robot connecté")
    
    # Lancer viewer
    if robot.launch_viewer(passive=True):
        print("✅ Viewer MuJoCo lancé")
    
    print("\n🎬 DÉBUT SÉQUENCE (8 secondes)")
    print("📹 Préparez votre enregistrement d'écran maintenant !\n")
    time.sleep(2)  # Temps pour démarrer l'enregistrement

    fps = 10
    total_duration = sum(duration for _, duration, _ in sequence)
    
    print(f"⏱️  Durée totale : {total_duration}s\n")

    step_count = 0
    for emotion, duration, intensity in sequence:
        total_steps = int(duration * fps)
        print(f"🎭 {emotion.upper()} ({duration}s, intensité {intensity})")
        
        for step in range(total_steps):
            angle = emotion_to_pose(emotion, intensity, step, total_steps)
            robot.set_joint_pos("yaw_body", angle)
            robot.step()
            robot.sync_viewer()
            step_count += 1
            time.sleep(1.0 / fps)

    print(f"\n✅ Séquence terminée ({total_duration}s)")
    print("⏸️  Arrêtez votre enregistrement d'écran")
    print("💾 Puis convertissez la vidéo en GIF avec ffmpeg")
    
    time.sleep(2)
    robot.disconnect()
    return 0


if __name__ == "__main__":
    exit(main())

