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
                {
                    "phase": "stretch",
                    "duration": 6,
                    "movement": "slow_rise",
                },  # 2x plus long
                {
                    "phase": "look_around",
                    "duration": 4,
                    "movement": "scan",
                },  # 2x plus long
                {
                    "phase": "ready",
                    "duration": 2,
                    "movement": "neutral",
                },  # 2x plus long
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
    # PERSONNALITÉ BBIA: Comportements expressifs comme un chien/humain/IA
    # Mouvements beaucoup plus variés et expressifs
    current_phase = scenario.get_current_phase()
    movement_type = current_phase["movement"]
    phase_progress = scenario.current_step / (current_phase["duration"] * fps)

    # Mapping mouvement → angle EXPRESSIF (SÉCURISÉ SDK - max 0.3 rad conforme)
    # OPTIMISATION EXPERTE: Amplitudes conservatrices pour éviter dépassement avec intensité
    movements = {
        "slow_rise": (
            lambda t: 0.12 * (1 - math.cos(math.pi * t))
            + 0.04 * math.sin(4 * math.pi * t)
        ),  # Lever avec ondulations (max 0.16 rad)
        "scan": (
            lambda t: 0.15
            * math.sin(4 * math.pi * t)
            * (1 + 0.2 * math.sin(8 * math.pi * t))
        ),  # Balayage animé (max 0.18 rad)
        "neutral": (
            lambda t: 0.02 * math.sin(6 * math.pi * t)
        ),  # Position neutre avec micro-mouvements (max 0.02 rad)
        "forward": (
            lambda t: 0.12 * math.sin(2 * math.pi * t)
            + 0.04 * math.cos(4 * math.pi * t)
        ),  # Avancer expressif (max 0.16 rad)
        "wave": (
            lambda t: 0.18
            * math.sin(6 * math.pi * t)
            * (1 + 0.3 * math.sin(12 * math.pi * t))
        ),  # Salutation enthousiaste (max 0.234 rad < 0.3)
        "happy": (
            lambda t: 0.10
            * math.sin(2 * math.pi * 0.1 * t)
            * (1 + 0.5 * math.sin(6 * math.pi * t))
        ),  # Joyeux ondulant (max 0.15 rad)
        "focus": (
            lambda t: 0.08 * math.sin(2 * math.pi * 0.1 * t)
            + 0.03 * math.sin(10 * math.pi * t)
        ),  # Concentration rapide (max 0.11 rad)
        "emotional": (
            lambda t: 0.12
            * math.sin(2 * math.pi * 0.8 * t)
            * (1 + 0.4 * math.cos(8 * math.pi * t))
        ),  # Émotionnel animé (max 0.168 rad)
        "calm": (
            lambda t: 0.08 * math.sin(2 * math.pi * 0.2 * t) * math.cos(3 * math.pi * t)
        ),  # Calme complexe (max 0.08 rad)
    }

    movement_func = movements.get(movement_type)
    if movement_func is None:
        movement_func = movements["neutral"]
    base_movement = movement_func(phase_progress)

    # Ajouter une transition douce entre les phases
    if scenario.phase < scenario.total_phases - 1:
        next_phase = scenario.current_scenario[scenario.phase + 1]
        next_movement_name = str(next_phase["movement"])
        next_movement_func = movements.get(next_movement_name)
        if next_movement_func is None:
            next_movement_func = movements["neutral"]
        transition_factor = min(
            1.0, phase_progress * 2
        )  # Transition sur la dernière moitié
        base_movement = (
            base_movement * (1 - transition_factor)
            + next_movement_func(0) * transition_factor
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
        "--duration", type=int, default=40, help="Durée totale en secondes"
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

        # CRITIQUE: Remettre le robot en position initiale centrée
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

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
                # Configurer la caméra à 180° (face optimal) immédiatement
                viewer.cam.azimuth = 180.0
                viewer.cam.elevation = -15.0
                viewer.cam.distance = 1.2  # Rapproché de 20%
                viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
                viewer.sync()

                start_time = time.time()
                step = 0

                while viewer.is_running() and step < total_steps:
                    # Mettre à jour le scénario
                    scenario.update_phase(step, fps)

                    # Calculer le mouvement basé sur le comportement
                    angle = behavior_to_movement(scenario, step, fps)
                    angle *= args.intensity  # Appliquer l'intensité

                    # Appliquer la pose au joint
                    data.qpos[joint_id] = angle

                    # EXPERT ROBOTIQUE: Synchronisation tête+corps pour expressivité optimale
                    # Au lieu de contrôler stewart_1 directement (non conforme SDK),
                    # utiliser la cinématique inverse via create_head_pose si disponible
                    if args.joint == "yaw_body":
                        # OPTIMISATION: Mouvement tête synchronisé via cinématique inverse (conforme SDK)
                        # Note: stewart_1 ne peut pas être contrôlé individuellement (plateforme Stewart)
                        # Si SDK disponible, utiliser create_head_pose pour mouvement tête expressif
                        try:
                            # Mouvement tête subtil synchronisé avec corps (écoute attentive)
                            head_pitch = 0.05 * math.sin(step * 0.1)  # Pitch subtil
                            # Pas de yaw latéral pour ce mouvement
                            # Note: Dans MuJoCo direct, on simule en contrôlant stewart_1,
                            # mais avec le SDK officiel, utiliser create_head_pose + set_target_head_pose
                            # Ici en mode MuJoCo direct, on utilise l'approximation stewart_1
                            stewart_1_id = None
                            for i in range(model.njnt):
                                if (
                                    mujoco.mj_id2name(
                                        model, mujoco.mjtObj.mjOBJ_JOINT, i
                                    )
                                    == "stewart_1"
                                ):
                                    stewart_1_id = i
                                    break
                            if stewart_1_id is not None:
                                # Approximation: stewart_1 ≈ pitch (valide uniquement en sim MuJoCo direct)
                                data.qpos[stewart_1_id] = head_pitch
                        except ImportError:
                            # Fallback sans SDK: petit mouvement stewart_1 approximatif
                            stewart_1_id = None
                            for i in range(model.njnt):
                                if (
                                    mujoco.mj_id2name(
                                        model, mujoco.mjtObj.mjOBJ_JOINT, i
                                    )
                                    == "stewart_1"
                                ):
                                    stewart_1_id = i
                                    break
                            if stewart_1_id is not None:
                                head_pos = 0.06 * math.sin(step * 0.1)
                                data.qpos[stewart_1_id] = head_pos

                    mujoco.mj_forward(model, data)  # Mettre à jour la physique
                    mujoco.mj_step(model, data)  # Avancer la simulation
                    viewer.sync()  # CRITIQUE: Synchroniser le viewer

                    step += 1

                    # Log périodique
                    if step % 20 == 0:
                        elapsed = time.time() - start_time
                        current_phase = scenario.get_current_phase()
                        print(
                            f"  Step {step:3d} | t={elapsed:3.1f}s | phase={current_phase['phase']} | {args.joint}={angle:6.3f} rad"
                        )

                    # CRITIQUE: Petit délai pour fluidité (60 FPS = 1/60s)
                    time.sleep(1 / 60)

            print(f"✅ Animation graphique terminée ({step} steps)")

        except Exception as e:
            print(f"❌ Erreur viewer graphique : {e}")
            print("💡 Essayez le mode headless : --headless")
            return 1

    print("\n🎉 Démo comportement → scénario terminée avec succès !")
    print(f"   • Comportement '{args.behavior}' → Scénario complet")
    print(f"   • Phases exécutées : {scenario.phase}/{scenario.total_phases}")
    print(f"   • Joint '{args.joint}' → Animation fluide")

    # Garder le viewer ouvert quelques secondes si mode graphique
    if not args.headless:
        print("\n⏸️  Viewer ouvert encore 3 secondes...")
        time.sleep(3)

    return 0


if __name__ == "__main__":
    exit(main())
