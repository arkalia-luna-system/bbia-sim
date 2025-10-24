#!/usr/bin/env python3
"""
Démo Vision → Suivi : BBIA Vision suit une cible
Vertical slice : Détection → Tracking → Mouvement robotique
"""

import argparse
import math
import random
import sys
import time
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import mujoco
import mujoco.viewer
from bbia_sim.bbia_vision import BBIAVision


class VirtualTarget:
    """Cible virtuelle pour simulation du tracking."""
    
    def __init__(self):
        self.x = 0.0  # Position horizontale (-1 à 1)
        self.y = 0.0  # Position verticale (-1 à 1)
        self.speed = 0.02
        self.direction = random.uniform(0, 2 * math.pi)
    
    def update(self):
        """Met à jour la position de la cible."""
        # Mouvement aléatoire avec changement de direction
        if random.random() < 0.1:  # 10% de chance de changer de direction
            self.direction = random.uniform(0, 2 * math.pi)
        
        # Déplacement
        self.x += self.speed * math.cos(self.direction)
        self.y += self.speed * math.sin(self.direction)
        
        # Limites
        self.x = max(-1.0, min(1.0, self.x))
        self.y = max(-1.0, min(1.0, self.y))


def vision_to_tracking(target_x: float, target_y: float, step: int, total_steps: int) -> float:
    """Convertit la position de la cible en angle de tracking."""
    # Mapping position cible → angle de rotation
    # target_x : -1 (gauche) à 1 (droite) → angle : -0.5 à 0.5 rad
    
    # Tracking horizontal (yaw_body)
    tracking_angle = target_x * 0.5  # Amplitude max 0.5 rad
    
    # Ajouter un peu de mouvement naturel
    natural_movement = 0.1 * math.sin(2 * math.pi * 0.2 * step / total_steps)
    
    return tracking_angle + natural_movement


def main():
    parser = argparse.ArgumentParser(description="Démo Vision → Suivi BBIA")
    parser.add_argument("--duration", type=int, default=15, help="Durée en secondes")
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument("--joint", default="yaw_body", help="Joint à animer")
    parser.add_argument("--target-speed", type=float, default=0.02, help="Vitesse de la cible")
    parser.add_argument("--tracking-gain", type=float, default=0.5, help="Gain de tracking")
    
    args = parser.parse_args()
    
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
    
    # 3. Initialiser BBIA Vision
    vision = BBIAVision()
    print(f"✅ BBIA Vision initialisé")
    print(f"   • Qualité : {vision.vision_quality}")
    print(f"   • Portée : {vision.detection_range}m")
    print(f"   • Tracking : {'actif' if vision.tracking_active else 'inactif'}")
    
    # 4. Créer la cible virtuelle
    target = VirtualTarget()
    target.speed = args.target_speed
    
    print(f"\n👁️ Configuration BBIA Vision → Suivi :")
    print(f"   • Cible virtuelle : position aléatoire")
    print(f"   • Vitesse cible : {args.target_speed}")
    print(f"   • Gain tracking : {args.tracking_gain}")
    print(f"   • Joint : {args.joint}")
    print(f"   • Durée : {args.duration}s")
    print(f"   • Mode : {'headless' if args.headless else 'graphique'}")
    
    # 5. Configuration animation
    fps = 10
    total_steps = args.duration * fps
    
    print(f"\n🚀 Démarrage animation vision → suivi...")
    print(f"   • Cible initiale : x={target.x:.2f}, y={target.y:.2f}")
    
    if args.headless:
        # Mode headless
        start_time = time.time()
        for step in range(total_steps):
            # Mettre à jour la cible
            target.update()
            
            # Calculer l'angle de tracking
            angle = vision_to_tracking(target.x, target.y, step, total_steps)
            angle *= args.tracking_gain  # Appliquer le gain
            
            # Appliquer la pose
            data.qpos[joint_id] = angle
            mujoco.mj_step(model, data)
            
            # Log périodique
            if step % 30 == 0:
                elapsed = time.time() - start_time
                print(f"  Step {step:3d} | t={elapsed:3.1f}s | target=({target.x:5.2f},{target.y:5.2f}) | {args.joint}={angle:6.3f} rad")
        
        print(f"✅ Animation headless terminée ({total_steps} steps)")
    
    else:
        # Mode graphique
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                start_time = time.time()
                step = 0
                
                while viewer.is_running() and step < total_steps:
                    # Mettre à jour la cible
                    target.update()
                    
                    # Calculer l'angle de tracking
                    angle = vision_to_tracking(target.x, target.y, step, total_steps)
                    angle *= args.tracking_gain  # Appliquer le gain
                    
                    # Appliquer la pose
                    data.qpos[joint_id] = angle
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    
                    step += 1
                    
                    # Log périodique
                    if step % 30 == 0:
                        elapsed = time.time() - start_time
                        print(f"  Step {step:3d} | t={elapsed:3.1f}s | target=({target.x:5.2f},{target.y:5.2f}) | {args.joint}={angle:6.3f} rad")
            
            print(f"✅ Animation graphique terminée ({step} steps)")
            
        except Exception as e:
            print(f"❌ Erreur viewer graphique : {e}")
            print("💡 Essayez le mode headless : --headless")
            return 1
    
    print(f"\n🎉 Démo vision → suivi terminée avec succès !")
    print(f"   • Cible virtuelle → Tracking '{args.joint}'")
    print(f"   • Position finale : x={target.x:.2f}, y={target.y:.2f}")
    print(f"   • Angle final : {data.qpos[joint_id]:.3f} rad")
    
    return 0


if __name__ == "__main__":
    exit(main())
