#!/usr/bin/env python3
"""
Démo Comportement → Scénario : BBIA Comportements exécutent des scénarios
Vertical slice : Comportement → Séquence d'actions → Animation complète
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

from bbia_sim.bbia_behavior import BBIABehaviorManager


class BehaviorScenario:
    """Scénario de comportement avec séquence d'actions."""

    def __init__(self, behavior_name: str):
        self.behavior_name = behavior_name
        self.phase = 0
        self.phase_duration = 0
        self.current_step = 0

        # Définir les phases du scénario
        self.scenarios = {
            "wake_up": [
                {"phase": "stretch", "duration": 3, "movement": "slow_rise"},
                {"phase": "look_around", "duration": 2, "movement": "scan"},
                {"phase": "ready", "duration": 1, "movement": "neutral"},
            ],
            "greeting": [
                {"phase": "approach", "duration": 2, "movement": "forward"},
                {"phase": "wave", "duration": 3, "movement": "wave"},
                {"phase": "smile", "duration": 2, "movement": "happy"},
            ],
            "emotional_response": [
                {"phase": "recognize", "duration": 1, "movement": "focus"},
                {"phase": "react", "duration": 3, "movement": "emotional"},
                {"phase": "settle", "duration": 2, "movement": "calm"},
            ],
        }

        self.current_scenario = self.scenarios.get(
            behavior_name, self.scenarios["wake_up"]
        )
        self.total_phases = len(self.current_scenario)

    def get_current_phase(self):
        """Retourne la phase actuelle du scénario."""
        if self.phase < self.total_phases:
            return self.current_scenario[self.phase]
        return self.current_scenario[-1]  # Dernière phase

    def update_phase(self, step: int, fps: int):
        """Met à jour la phase du scénario."""
        current_phase_info = self.get_current_phase()
        phase_steps = current_phase_info["duration"] * fps

        if self.current_step >= phase_steps:
            self.phase += 1
            self.current_step = 0
            if self.phase < self.total_phases:
                print(f"🔄 Phase suivante : {self.get_current_phase()['phase']}")
        else:
            self.current_step += 1


def behavior_to_movement(scenario: BehaviorScenario, step: int, fps: int) -> float:
    """Convertit un comportement BBIA en mouvement robotique."""
    current_phase = scenario.get_current_phase()
    movement_type = current_phase["movement"]
    phase_progress = scenario.current_step / (current_phase["duration"] * fps)

    # Mapping mouvement → angle
    movements = {
        "slow_rise": lambda t: 0.3 * (1 - math.cos(math.pi * t)),  # Lever lent
        "scan": lambda t: 0.4 * math.sin(4 * math.pi * t),  # Balayage
        "neutral": lambda t: 0.0,  # Position neutre
        "forward": lambda t: 0.2 * math.sin(2 * math.pi * t),  # Avancer
        "wave": lambda t: 0.5 * math.sin(6 * math.pi * t),  # Salutation
        "happy": lambda t: 0.3 * math.sin(2 * math.pi * 0.5 * t),  # Joyeux
        "focus": lambda t: 0.1 * math.sin(2 * math.pi * 0.1 * t),  # Concentration
        "emotional": lambda t: 0.4 * math.sin(2 * math.pi * 0.8 * t),  # Émotionnel
        "calm": lambda t: 0.1 * math.sin(2 * math.pi * 0.2 * t),  # Calme
    }

    base_movement = movements.get(movement_type, movements["neutral"])(phase_progress)

    # Ajouter une transition douce entre les phases
    if scenario.phase < scenario.total_phases - 1:
        next_phase = scenario.current_scenario[scenario.phase + 1]
        next_movement = movements.get(next_phase["movement"], movements["neutral"])
        transition_factor = min(
            1.0, phase_progress * 2
        )  # Transition sur la dernière moitié
        base_movement = (
            base_movement * (1 - transition_factor)
            + next_movement(0) * transition_factor
        )

    return base_movement


def main():
    parser = argparse.ArgumentParser(description="Démo Comportement → Scénario BBIA")
    parser.add_argument(
        "--behavior",
        default="wake_up",
        help="Comportement BBIA (wake_up, greeting, emotional_response)",
    )
    parser.add_argument(
        "--duration", type=int, default=10, help="Durée totale en secondes"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument("--joint", default="yaw_body", help="Joint à animer")
    parser.add_argument(
        "--intensity", type=float, default=1.0, help="Intensité du comportement"
    )

    args = parser.parse_args()

    # Validation
    if args.intensity < 0.0 or args.intensity > 2.0:
        print("❌ Intensité doit être entre 0.0 et 2.0")
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

    # 3. Initialiser BBIA Comportements
    behavior_manager = BBIABehaviorManager()
    print("✅ BBIA Comportements initialisés")
    print(f"   • Comportements disponibles : {list(behavior_manager.behaviors.keys())}")

    # 4. Créer le scénario
    scenario = BehaviorScenario(args.behavior)

    print("\n🎭 Configuration BBIA Comportement → Scénario :")
    print(f"   • Comportement : {args.behavior}")
    print(f"   • Phases : {scenario.total_phases}")
    for i, phase in enumerate(scenario.current_scenario):
        print(f"     {i+1}. {phase['phase']} ({phase['duration']}s)")
    print(f"   • Joint : {args.joint}")
    print(f"   • Durée totale : {args.duration}s")
    print(f"   • Intensité : {args.intensity}")
    print(f"   • Mode : {'headless' if args.headless else 'graphique'}")

    # 5. Configuration animation
    fps = 10
    total_steps = args.duration * fps

    print("\n🚀 Démarrage animation comportement → scénario...")
    print(f"   • Phase initiale : {scenario.get_current_phase()['phase']}")

    if args.headless:
        # Mode headless
        start_time = time.time()
        for step in range(total_steps):
            # Mettre à jour le scénario
            scenario.update_phase(step, fps)

            # Calculer le mouvement basé sur le comportement
            angle = behavior_to_movement(scenario, step, fps)
            angle *= args.intensity  # Appliquer l'intensité

            # Appliquer la pose
            data.qpos[joint_id] = angle
            mujoco.mj_step(model, data)

            # Log périodique
            if step % 20 == 0:
                elapsed = time.time() - start_time
                current_phase = scenario.get_current_phase()
                print(
                    f"  Step {step:3d} | t={elapsed:3.1f}s | phase={current_phase['phase']} | {args.joint}={angle:6.3f} rad"
                )

        print(f"✅ Animation headless terminée ({total_steps} steps)")

    else:
        # Mode graphique
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                start_time = time.time()
                step = 0

                while viewer.is_running() and step < total_steps:
                    # Mettre à jour le scénario
                    scenario.update_phase(step, fps)

                    # Calculer le mouvement basé sur le comportement
                    angle = behavior_to_movement(scenario, step, fps)
                    angle *= args.intensity  # Appliquer l'intensité

                    # Appliquer la pose
                    data.qpos[joint_id] = angle
                    mujoco.mj_step(model, data)
                    viewer.sync()

                    step += 1

                    # Log périodique
                    if step % 20 == 0:
                        elapsed = time.time() - start_time
                        current_phase = scenario.get_current_phase()
                        print(
                            f"  Step {step:3d} | t={elapsed:3.1f}s | phase={current_phase['phase']} | {args.joint}={angle:6.3f} rad"
                        )

            print(f"✅ Animation graphique terminée ({step} steps)")

        except Exception as e:
            print(f"❌ Erreur viewer graphique : {e}")
            print("💡 Essayez le mode headless : --headless")
            return 1

    print("\n🎉 Démo comportement → scénario terminée avec succès !")
    print(f"   • Comportement '{args.behavior}' → Scénario complet")
    print(f"   • Phases exécutées : {scenario.phase}/{scenario.total_phases}")
    print(f"   • Joint '{args.joint}' → Animation fluide")

    return 0


if __name__ == "__main__":
    exit(main())
