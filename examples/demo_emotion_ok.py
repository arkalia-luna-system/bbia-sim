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


def emotion_to_pose(emotion: str, intensity: float, step: int, total_steps: int) -> float:
    """Convertit une émotion BBIA en position d'articulation."""
    # Mapping émotion → mouvement
    emotion_patterns = {
        "happy": lambda t: 0.3 * math.sin(2 * math.pi * 0.5 * t),  # Mouvement joyeux
        "sad": lambda t: -0.2 * math.sin(2 * math.pi * 0.3 * t),   # Mouvement triste
        "angry": lambda t: 0.4 * math.sin(2 * math.pi * 0.8 * t),  # Mouvement agité
        "surprised": lambda t: 0.5 * math.sin(2 * math.pi * 0.2 * t),  # Mouvement lent
        "neutral": lambda t: 0.1 * math.sin(2 * math.pi * 0.1 * t),  # Mouvement subtil
    }
    
    t = step / total_steps
    base_movement = emotion_patterns.get(emotion, emotion_patterns["neutral"])(t)
    
    # Appliquer l'intensité
    return base_movement * intensity


def main():
    parser = argparse.ArgumentParser(description="Démo Émotion → Pose BBIA")
    parser.add_argument("--emotion", default="happy", help="Émotion BBIA (happy, sad, angry, surprised, neutral)")
    parser.add_argument("--intensity", type=float, default=0.8, help="Intensité de l'émotion (0.0-1.0)")
    parser.add_argument("--duration", type=int, default=10, help="Durée en secondes")
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument("--joint", default="yaw_body", help="Joint à animer")
    parser.add_argument("--backend", default="mujoco", help="Backend (mujoco, reachy)")
    parser.add_argument("--record", help="Enregistrer l'animation dans ce fichier (.jsonl)")
    
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
    
    print(f"✅ Backend {args.backend} connecté : {len(robot.get_available_joints())} joints détectés")
    
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
    
    print(f"\n🎭 Configuration BBIA Émotion → Pose :")
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
    print(f"\n🚀 Démarrage animation émotion → pose...")
    
    try:
        start_time = time.time()
        for step in range(total_steps):
            # Calculer la pose basée sur l'émotion
            angle = emotion_to_pose(args.emotion, args.intensity, step, total_steps)
            
            # Appliquer la pose via RobotAPI
            robot.set_joint_pos(args.joint, angle)
            robot.step()
            
            # Enregistrer la frame si demandé
            if recorder:
                recorder.record_frame(args.joint, angle, step)
            
            # Log périodique
            if step % 20 == 0:
                elapsed = time.time() - start_time
                current_pos = robot.get_joint_pos(args.joint)
                print(f"  Step {step:3d} | t={elapsed:3.1f}s | {args.joint}={current_pos:6.3f} rad")
        
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
    
    print(f"\n🎉 Démo émotion → pose terminée avec succès !")
    print(f"   • Backend {args.backend} → Émotion '{args.emotion}' → Joint '{args.joint}'")
    print(f"   • Intensité {args.intensity} → Animation fluide")
    
    return 0


if __name__ == "__main__":
    exit(main())
