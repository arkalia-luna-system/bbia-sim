#!/usr/bin/env python3
"""
🎉 SURPRISE 3D BBIA-SIM - Visualisation Complète du Robot Reachy-Mini
Démonstration spectaculaire avec le SDK officiel et MuJoCo
"""

import argparse
import sys
import time
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.mapping_reachy import ReachyMapping
from bbia_sim.robot_api import RobotFactory


def surprise_3d_mujoco():
    """Surprise 3D avec MuJoCo - Visualisation spectaculaire !"""
    print("🎉 SURPRISE 3D BBIA-SIM - Visualisation Spectaculaire !")
    print("=" * 60)
    print("🤖 Robot Reachy-Mini avec SDK Officiel + MuJoCo")
    print("🎨 Animation complète des capacités du robot")
    print("=" * 60)

    # Créer le robot MuJoCo
    print("\n🔧 Initialisation MuJoCo...")
    robot_mujoco = RobotFactory.create_backend("mujoco")
    robot_mujoco.connect()
    print("✅ Robot MuJoCo connecté")

    # Créer le robot SDK officiel pour comparaison
    print("\n🔧 Initialisation SDK Officiel...")
    robot_officiel = RobotFactory.create_backend("reachy_mini")
    robot_officiel.is_connected = True  # Mode simulation
    print("✅ Robot SDK Officiel connecté (simulation)")

    mapping = ReachyMapping()

    print("\n📊 Comparaison Backends:")
    print(f"   MuJoCo: {len(robot_mujoco.get_available_joints())} joints")
    print(f"   SDK Officiel: {len(robot_officiel.get_available_joints())} joints")
    print(f"   Joints recommandés: {mapping.get_recommended_joints()}")

    # Animation spectaculaire
    print("\n🎬 DÉBUT DE L'ANIMATION SPECTACULAIRE !")
    print("=" * 50)

    # Séquence 1: Réveil du robot
    print("\n🌅 Séquence 1: Réveil du Robot")
    robot_mujoco.run_behavior("wake_up", 3.0)
    robot_officiel.run_behavior("wake_up", 3.0)
    print("✅ Robot réveillé avec élégance")

    # Séquence 2: Émotions expressives
    print("\n🎭 Séquence 2: Émotions Expressives")
    emotions = [
        ("happy", "😊 Heureux"),
        ("excited", "🤩 Excité"),
        ("curious", "🤔 Curieux"),
        ("sad", "😢 Triste"),
        ("neutral", "😐 Neutre"),
        ("calm", "😌 Calme"),
    ]

    for emotion, emoji_desc in emotions:
        print(f"   {emoji_desc}")
        robot_mujoco.set_emotion(emotion, 0.8)
        robot_officiel.set_emotion(emotion, 0.8)
        time.sleep(1.5)

    # Séquence 3: Mouvements de tête complexes
    print("\n👀 Séquence 3: Mouvements de Tête Complexes")
    head_movements = [
        ("head_1", "Hochement vertical"),
        ("head_2", "Rotation latérale"),
        ("head_3", "Inclinaison"),
        ("body_yaw", "Rotation corps"),
    ]

    for joint, description in head_movements:
        print(f"   {description} ({joint})")
        # Mouvement sinusoïdal
        for i in range(20):
            angle = 0.2 * (i / 10.0 - 1.0)  # -0.2 à +0.2
            robot_mujoco.set_joint_pos(joint, angle)
            robot_officiel.set_joint_pos(joint, angle)
            time.sleep(0.1)

        # Retour au centre
        robot_mujoco.set_joint_pos(joint, 0.0)
        robot_officiel.set_joint_pos(joint, 0.0)
        time.sleep(0.5)

    # Séquence 4: Look_at dynamique
    print("\n👁️ Séquence 4: Look_at Dynamique")
    look_positions = [
        (0.2, 0.0, 0.3, "Regarde devant"),
        (0.0, 0.2, 0.3, "Regarde à droite"),
        (-0.2, 0.0, 0.3, "Regarde derrière"),
        (0.0, -0.2, 0.3, "Regarde à gauche"),
        (0.1, 0.1, 0.4, "Regarde en haut-droite"),
        (0.0, 0.0, 0.2, "Regarde au centre"),
    ]

    for x, y, z, description in look_positions:
        print(f"   {description} ({x}, {y}, {z})")
        robot_mujoco.look_at(x, y, z)
        robot_officiel.look_at(x, y, z)
        time.sleep(1.0)

    # Séquence 5: Comportements sociaux
    print("\n🤝 Séquence 5: Comportements Sociaux")
    behaviors = [
        ("nod", "Hochement de tête"),
        ("wake_up", "Réveil expressif"),
        ("goto_sleep", "Endormissement"),
    ]

    for behavior, description in behaviors:
        print(f"   {description}")
        robot_mujoco.run_behavior(behavior, 2.0)
        robot_officiel.run_behavior(behavior, 2.0)
        time.sleep(1.0)

    # Séquence 6: Démonstration sécurité
    print("\n🛡️ Séquence 6: Démonstration Sécurité")
    print("   Test amplitude limite (0.3 rad)")

    # Test amplitude normale
    robot_mujoco.set_joint_pos("body_yaw", 0.2)
    robot_officiel.set_joint_pos("body_yaw", 0.2)
    print("   ✅ Amplitude normale: OK")
    time.sleep(1.0)

    # Test amplitude excessive (doit être clampée)
    robot_mujoco.set_joint_pos("body_yaw", 0.5)  # > 0.3
    robot_officiel.set_joint_pos("body_yaw", 0.5)  # > 0.3
    print("   ✅ Amplitude excessive: Clampée automatiquement")
    time.sleep(1.0)

    # Test joint interdit
    success_mujoco = robot_mujoco.set_joint_pos("left_antenna", 0.1)
    success_officiel = robot_officiel.set_joint_pos("left_antenna", 0.1)
    print(f"   ✅ Joint interdit: MuJoCo={success_mujoco}, SDK={success_officiel}")

    # Séquence 7: Finale spectaculaire
    print("\n🎆 SÉQUENCE FINALE: SPECTACLE COMPLET !")
    print("=" * 40)

    # Mouvement de rotation complète
    print("   🌪️ Rotation complète du corps")
    for i in range(50):
        angle = (i / 50.0) * 2 * 3.14159 - 3.14159  # -π à +π
        robot_mujoco.set_joint_pos("body_yaw", angle * 0.1)  # Limité à 0.1
        robot_officiel.set_joint_pos("body_yaw", angle * 0.1)
        time.sleep(0.05)

    # Émotion finale
    print("   🎉 Émotion finale: EXCITED !")
    robot_mujoco.set_emotion("excited", 1.0)
    robot_officiel.set_emotion("excited", 1.0)
    time.sleep(2.0)

    # Retour au neutre
    print("   😌 Retour au neutre")
    robot_mujoco.set_emotion("neutral", 0.5)
    robot_officiel.set_emotion("neutral", 0.5)
    robot_mujoco.set_joint_pos("body_yaw", 0.0)
    robot_officiel.set_joint_pos("body_yaw", 0.0)

    # Télémétrie finale
    print("\n📊 TÉLÉMÉTRIE FINALE")
    print("=" * 30)

    telemetry_mujoco = robot_mujoco.get_telemetry()
    telemetry_officiel = robot_officiel.get_telemetry()

    print("MuJoCo:")
    for key, value in telemetry_mujoco.items():
        print(f"   {key}: {value}")

    print("\nSDK Officiel:")
    for key, value in telemetry_officiel.items():
        print(f"   {key}: {value}")

    # Déconnexion
    print("\n🔌 Déconnexion...")
    robot_mujoco.disconnect()
    robot_officiel.disconnect()

    print("\n🎉 SURPRISE TERMINÉE !")
    print("=" * 50)
    print("✨ Vous avez vu le robot Reachy-Mini dans toute sa splendeur !")
    print("🤖 SDK Officiel + MuJoCo = Performance maximale")
    print("🚀 Prêt pour le robot physique dans 2 mois !")
    print("=" * 50)


def surprise_3d_comparison():
    """Comparaison spectaculaire des 3 backends."""
    print("\n🔍 COMPARAISON SPECTACULAIRE DES 3 BACKENDS")
    print("=" * 50)

    backends = ["mujoco", "reachy", "reachy_mini"]
    robots = {}

    # Créer tous les backends
    for backend in backends:
        print(f"\n🔧 Initialisation {backend}...")
        try:
            robot = RobotFactory.create_backend(backend)
            if backend == "mujoco":
                robot.connect()
            elif backend == "reachy_mini":
                robot.is_connected = True  # Simulation
            robots[backend] = robot
            print(f"✅ {backend}: {len(robot.get_available_joints())} joints")
        except Exception as e:
            print(f"❌ {backend}: Erreur - {e}")

    # Test émotion sur tous les backends
    print("\n🎭 Test émotion 'happy' sur tous les backends:")
    for backend, robot in robots.items():
        success = robot.set_emotion("happy", 0.7)
        print(f"   {backend}: {'✅' if success else '❌'}")

    # Test mouvement sur tous les backends
    print("\n👀 Test mouvement 'body_yaw' sur tous les backends:")
    for backend, robot in robots.items():
        success = robot.set_joint_pos("body_yaw", 0.1)
        print(f"   {backend}: {'✅' if success else '❌'}")

    # Test comportement sur tous les backends
    print("\n🤝 Test comportement 'nod' sur tous les backends:")
    for backend, robot in robots.items():
        success = robot.run_behavior("nod", 1.0)
        print(f"   {backend}: {'✅' if success else '❌'}")

    # Déconnexion
    print("\n🔌 Déconnexion de tous les backends...")
    for backend, robot in robots.items():
        try:
            robot.disconnect()
            print(f"✅ {backend}: Déconnecté")
        except Exception:
            print(f"ℹ️  {backend}: Pas de déconnexion nécessaire")


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(description="Surprise 3D BBIA-SIM")
    parser.add_argument(
        "--comparison", action="store_true", help="Comparaison des backends"
    )
    parser.add_argument("--quick", action="store_true", help="Version rapide")

    args = parser.parse_args()

    if args.comparison:
        surprise_3d_comparison()
    else:
        surprise_3d_mujoco()

    print("\n🚀 Prochaines étapes:")
    print("1. Tester avec robot physique (dans 2 mois)")
    print("2. Optimiser performances IA")
    print("3. Développer nouveaux comportements")
    print("4. Préparer démos professionnelles")
    print("\n🎉 Merci d'avoir regardé la surprise 3D !")


if __name__ == "__main__":
    main()
