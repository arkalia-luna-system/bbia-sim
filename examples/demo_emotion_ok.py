#!/usr/bin/env python3
"""
Démo Émotion → Pose : BBIA Émotions anime le robot
Vertical slice : Émotion → Articulation → Animation visible
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
from bbia_sim.bbia_emotions import BBIAEmotions


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
    
    args = parser.parse_args()
    
    # Validation
    if args.intensity < 0.0 or args.intensity > 1.0:
        print("❌ Intensité doit être entre 0.0 et 1.0")
        return 1
    
    # 1. Charger le modèle MuJoCo
    model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        print(f"✅ Modèle chargé : {model.njnt} joints détectés")
    except Exception as e:
        print(f"❌ Erreur chargement modèle : {e}")
        return 1
    
    # 2. Trouver le joint
    joint_id = None
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name == args.joint:
            joint_id = i
            break
    
    if joint_id is None:
        print(f"❌ Joint '{args.joint}' introuvable")
        return 1
    
    # 3. Initialiser BBIA Émotions
    emotions = BBIAEmotions()
    if args.emotion not in emotions.emotions:
        print(f"❌ Émotion '{args.emotion}' non supportée")
        print(f"✅ Émotions disponibles : {list(emotions.emotions.keys())}")
        return 1
    
    # 4. Configuration animation
    fps = 10  # 10 Hz pour animation fluide
    total_steps = args.duration * fps
    
    print(f"\n🎭 Configuration BBIA Émotion → Pose :")
    print(f"   • Émotion : {args.emotion}")
    print(f"   • Intensité : {args.intensity}")
    print(f"   • Joint : {args.joint}")
    print(f"   • Durée : {args.duration}s")
    print(f"   • Mode : {'headless' if args.headless else 'graphique'}")
    
    # 5. Animation
    print(f"\n🚀 Démarrage animation émotion → pose...")
    
    if args.headless:
        # Mode headless
        start_time = time.time()
        for step in range(total_steps):
            # Calculer la pose basée sur l'émotion
            angle = emotion_to_pose(args.emotion, args.intensity, step, total_steps)
            
            # Appliquer la pose
            data.qpos[joint_id] = angle
            mujoco.mj_step(model, data)
            
            # Log périodique
            if step % 20 == 0:
                elapsed = time.time() - start_time
                print(f"  Step {step:3d} | t={elapsed:3.1f}s | {args.joint}={angle:6.3f} rad")
        
        print(f"✅ Animation headless terminée ({total_steps} steps)")
    
    else:
        # Mode graphique
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                start_time = time.time()
                step = 0
                
                while viewer.is_running() and step < total_steps:
                    # Calculer la pose basée sur l'émotion
                    angle = emotion_to_pose(args.emotion, args.intensity, step, total_steps)
                    
                    # Appliquer la pose
                    data.qpos[joint_id] = angle
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    
                    step += 1
                    
                    # Log périodique
                    if step % 20 == 0:
                        elapsed = time.time() - start_time
                        print(f"  Step {step:3d} | t={elapsed:3.1f}s | {args.joint}={angle:6.3f} rad")
            
            print(f"✅ Animation graphique terminée ({step} steps)")
            
        except Exception as e:
            print(f"❌ Erreur viewer graphique : {e}")
            print("💡 Essayez le mode headless : --headless")
            return 1
    
    print(f"\n🎉 Démo émotion → pose terminée avec succès !")
    print(f"   • Émotion '{args.emotion}' → Joint '{args.joint}'")
    print(f"   • Intensité {args.intensity} → Animation fluide")
    
    return 0


if __name__ == "__main__":
    exit(main())
