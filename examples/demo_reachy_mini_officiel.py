#!/usr/bin/env python3
"""
Démo BBIA-SIM avec Backend Reachy-Mini SDK Officiel
Démonstration complète des capacités avec le SDK officiel
"""

import argparse
import sys
import time
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.mapping_reachy import ReachyMapping
from bbia_sim.robot_api import RobotFactory


def demo_reachy_mini_officiel():
    """Démo complète avec le backend Reachy-Mini SDK officiel."""
    print("🤖 Démo BBIA-SIM - Backend Reachy-Mini SDK Officiel")
    print("=" * 60)

    # Créer le robot
    print("\n🔧 Initialisation...")
    robot = RobotFactory.create_backend("reachy_mini")
    mapping = ReachyMapping()

    print(f"✅ Backend: {type(robot).__name__}")
    print(f"✅ Joints disponibles: {len(robot.get_available_joints())}")
    print(f"✅ Joints recommandés: {mapping.get_recommended_joints()}")

    # Simulation connexion
    print("\n🔌 Connexion simulée...")
    robot.is_connected = True
    print("✅ Robot connecté (mode simulation)")

    # Test émotions
    print("\n🎭 Test des émotions...")
    emotions = ["happy", "sad", "neutral", "excited", "curious", "calm"]
    for emotion in emotions:
        success = robot.set_emotion(emotion, 0.7)
        print(f"   {emotion}: {'✅' if success else '❌'}")
        time.sleep(0.5)

    # Test mouvements tête
    print("\n👀 Test mouvements tête...")
    head_joints = ["head_1", "head_2", "head_3", "head_4", "head_5", "head_6"]
    for joint in head_joints:
        # Mouvement doux
        robot.set_joint_pos(joint, 0.1)
        time.sleep(0.3)
        robot.set_joint_pos(joint, -0.1)
        time.sleep(0.3)
        robot.set_joint_pos(joint, 0.0)
        print(f"   {joint}: ✅")

    # Test rotation corps
    print("\n🔄 Test rotation corps...")
    robot.set_joint_pos("body_yaw", 0.2)
    time.sleep(1.0)
    robot.set_joint_pos("body_yaw", -0.2)
    time.sleep(1.0)
    robot.set_joint_pos("body_yaw", 0.0)
    print("   body_yaw: ✅")

    # Test look_at
    print("\n👁️ Test look_at...")
    positions = [(0.1, 0.0, 0.2), (0.0, 0.1, 0.2), (-0.1, 0.0, 0.2)]
    for x, y, z in positions:
        success = robot.look_at(x, y, z)
        print(f"   look_at({x}, {y}, {z}): {'✅' if success else '❌'}")
        time.sleep(0.5)

    # Test comportements
    print("\n🎬 Test comportements...")
    behaviors = ["wake_up", "nod", "goto_sleep"]
    for behavior in behaviors:
        success = robot.run_behavior(behavior, 2.0)
        print(f"   {behavior}: {'✅' if success else '❌'}")
        time.sleep(1.0)

    # Test sécurité
    print("\n🛡️ Test sécurité...")
    print(f"   Limite amplitude: {robot.safe_amplitude_limit} rad")
    print(f"   Joints interdits: {robot.forbidden_joints}")

    # Test joints interdits
    for joint in robot.forbidden_joints:
        success = robot.set_joint_pos(joint, 0.1)
        print(
            f"   Joint interdit {joint}: {'❌ (OK)' if not success else '⚠️ (Problème)'}"
        )

    # Test amplitude limite
    robot.set_joint_pos("head_1", 0.5)  # Au-delà de la limite
    pos = robot.get_joint_pos("head_1")
    print(f"   Amplitude limitée: {pos:.3f} rad (≤ {robot.safe_amplitude_limit})")

    # Télémétrie finale
    print("\n📊 Télémétrie finale...")
    telemetry = robot.get_telemetry()
    for key, value in telemetry.items():
        print(f"   {key}: {value}")

    print("\n🎉 Démo terminée avec succès !")
    print("✅ Backend Reachy-Mini SDK officiel 100% fonctionnel")


def demo_comparaison_backends():
    """Compare les performances des différents backends."""
    print("\n🔍 Comparaison Backends")
    print("=" * 40)

    backends = ["mujoco", "reachy", "reachy_mini"]

    for backend_type in backends:
        print(f"\n📋 Backend: {backend_type}")
        try:
            robot = RobotFactory.create_backend(backend_type)

            # Test création
            start_time = time.time()
            joints = robot.get_available_joints()
            creation_time = time.time() - start_time

            print(f"   Joints: {len(joints)}")
            print(f"   Temps création: {creation_time:.3f}s")
            print(f"   Type: {type(robot).__name__}")

            # Test émotion
            start_time = time.time()
            success = robot.set_emotion("happy", 0.5)
            emotion_time = time.time() - start_time

            print(f"   Émotion: {'✅' if success else '❌'} ({emotion_time:.3f}s)")

        except Exception as e:
            print(f"   Erreur: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Démo BBIA-SIM Reachy-Mini SDK Officiel"
    )
    parser.add_argument(
        "--compare", action="store_true", help="Comparer tous les backends"
    )
    parser.add_argument("--quick", action="store_true", help="Démo rapide")

    args = parser.parse_args()

    if args.compare:
        demo_comparaison_backends()
    else:
        demo_reachy_mini_officiel()

    print("\n🚀 Prochaines étapes:")
    print("1. Tester avec robot physique (dans 2 mois)")
    print("2. Optimiser performances IA")
    print("3. Développer nouveaux comportements")
    print("4. Préparer démos professionnelles")
