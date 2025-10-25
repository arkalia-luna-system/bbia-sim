#!/usr/bin/env python3
"""
Enregistreur de traces golden pour RobotAPI
Génère des traces de référence pour les tests de non-régression
"""

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

# Ajout du chemin src pour les imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory


def main():
    ap = argparse.ArgumentParser(description="Enregistreur de traces golden")
    ap.add_argument("--backend", default="mujoco", help="Backend (mujoco, reachy)")
    ap.add_argument("--duration", type=float, default=5.0, help="Durée en secondes")
    ap.add_argument("--joint", default="yaw_body", help="Joint à animer")
    ap.add_argument("--emotion", default="happy", help="Émotion BBIA")
    ap.add_argument("--seed", type=int, default=42, help="Seed pour reproductibilité")
    ap.add_argument(
        "--out", default="artifacts/golden/tmp.jsonl", help="Fichier de sortie"
    )
    ap.add_argument("--slow", action="store_true", help="Mode lent pour robot réel")
    args = ap.parse_args()

    # Configuration reproductible
    random.seed(args.seed)
    t0 = time.time()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)

    print("🎬 Enregistrement trace golden:")
    print(f"   • Backend: {args.backend}")
    print(f"   • Durée: {args.duration}s")
    print(f"   • Joint: {args.joint}")
    print(f"   • Émotion: {args.emotion}")
    print(f"   • Seed: {args.seed}")
    print(f"   • Sortie: {args.out}")

    # Créer le robot
    robot = RobotFactory.create_backend(args.backend)
    if not robot:
        print(f"❌ Impossible de créer le backend {args.backend}")
        return 1

    # Connecter
    if not robot.connect():
        print(f"❌ Impossible de se connecter au backend {args.backend}")
        return 1

    # Définir l'émotion
    robot.set_emotion(args.emotion, intensity=0.8)

    # Enregistrer la trace
    frames_recorded = 0
    with open(args.out, "w", encoding="utf-8") as f:
        while (t := time.time() - t0) < args.duration:
            # Mouvement adapté selon le backend et le mode
            if args.backend == "reachy" or args.slow:
                # Mode lent pour robot réel (0.1 Hz = 10s par cycle)
                target = 0.2 * math.sin(2 * math.pi * 0.1 * t)
            else:
                # Mode normal pour simulation (0.5 Hz = 2s par cycle)
                target = 0.3 * math.sin(2 * math.pi * 0.5 * t)

            robot.set_joint_pos(args.joint, target)
            robot.step()

            qpos = robot.get_joint_pos(args.joint)
            if qpos is not None:
                frame = {
                    "t": t,
                    "joint": args.joint,
                    "qpos": qpos,
                    "meta": {"seed": args.seed, "backend": args.backend},
                }
                f.write(json.dumps(frame) + "\n")
                frames_recorded += 1

    robot.disconnect()

    print(f"✅ Trace enregistrée: {frames_recorded} frames → {args.out}")
    return 0


if __name__ == "__main__":
    exit(main())
