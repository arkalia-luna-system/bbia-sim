#!/usr/bin/env python3
"""
Démo Voix → Action : BBIA Voix contrôle le robot
Vertical slice : Commande vocale → Action robotique
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
from bbia_sim.bbia_voice import dire_texte


def parse_voice_command(command: str) -> dict:
    """Parse une commande vocale et retourne l'action correspondante."""
    command = command.lower().strip()
    
    # Mapping commandes vocales → actions
    voice_actions = {
        "regarde-moi": {"action": "look_at", "target": "user", "duration": 3},
        "tourne à gauche": {"action": "turn_left", "angle": 0.5, "duration": 2},
        "tourne à droite": {"action": "turn_right", "angle": 0.5, "duration": 2},
        "salue": {"action": "greet", "gesture": "wave", "duration": 3},
        "souris": {"action": "smile", "emotion": "happy", "duration": 2},
        "arrête": {"action": "stop", "duration": 1},
        "bonjour": {"action": "greet", "gesture": "nod", "duration": 2},
    }
    
    # Recherche de correspondance
    for key, action in voice_actions.items():
        if key in command:
            return action
    
    # Commande par défaut
    return {"action": "unknown", "duration": 1}


def execute_voice_action(action: dict, model, data, joint_id: int, step: int, total_steps: int) -> float:
    """Exécute une action basée sur la commande vocale."""
    action_type = action["action"]
    t = step / total_steps
    
    if action_type == "look_at":
        # Mouvement de regard
        return 0.3 * math.sin(2 * math.pi * 0.3 * t)
    
    elif action_type == "turn_left":
        # Rotation à gauche
        return -0.4 * (1 - math.cos(math.pi * t))
    
    elif action_type == "turn_right":
        # Rotation à droite
        return 0.4 * (1 - math.cos(math.pi * t))
    
    elif action_type == "greet":
        # Salutation (mouvement de va-et-vient)
        return 0.2 * math.sin(4 * math.pi * t)
    
    elif action_type == "smile":
        # Sourire (mouvement joyeux)
        return 0.25 * math.sin(2 * math.pi * 0.5 * t)
    
    elif action_type == "stop":
        # Arrêt (retour à la position neutre)
        return 0.0
    
    else:
        # Action inconnue (mouvement subtil)
        return 0.1 * math.sin(2 * math.pi * 0.1 * t)


def main():
    parser = argparse.ArgumentParser(description="Démo Voix → Action BBIA")
    parser.add_argument("--command", default="regarde-moi", help="Commande vocale")
    parser.add_argument("--duration", type=int, default=5, help="Durée en secondes")
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument("--joint", default="yaw_body", help="Joint à animer")
    parser.add_argument("--speak", action="store_true", help="Activer la synthèse vocale")
    
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
    
    # 3. Parser la commande vocale
    action = parse_voice_command(args.command)
    
    print(f"\n🎤 Configuration BBIA Voix → Action :")
    print(f"   • Commande : '{args.command}'")
    print(f"   • Action : {action['action']}")
    print(f"   • Joint : {args.joint}")
    print(f"   • Durée : {args.duration}s")
    print(f"   • Mode : {'headless' if args.headless else 'graphique'}")
    print(f"   • Synthèse vocale : {'activée' if args.speak else 'désactivée'}")
    
    # 4. Synthèse vocale (optionnelle)
    if args.speak:
        try:
            response = f"J'ai entendu : {args.command}"
            dire_texte(response)
            print(f"🗣️ BBIA dit : '{response}'")
        except Exception as e:
            print(f"⚠️ Erreur synthèse vocale : {e}")
    
    # 5. Configuration animation
    fps = 10
    total_steps = args.duration * fps
    
    print(f"\n🚀 Démarrage animation voix → action...")
    
    if args.headless:
        # Mode headless
        start_time = time.time()
        for step in range(total_steps):
            # Exécuter l'action basée sur la commande vocale
            angle = execute_voice_action(action, model, data, joint_id, step, total_steps)
            
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
                    # Exécuter l'action basée sur la commande vocale
                    angle = execute_voice_action(action, model, data, joint_id, step, total_steps)
                    
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
    
    print(f"\n🎉 Démo voix → action terminée avec succès !")
    print(f"   • Commande '{args.command}' → Action '{action['action']}'")
    print(f"   • Joint '{args.joint}' → Animation fluide")
    
    return 0


if __name__ == "__main__":
    exit(main())
