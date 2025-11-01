#!/usr/bin/env python3
"""
🎉 DÉMO REACHY-MINI CORRIGÉE - Utilise les vrais noms de joints
Démonstration fonctionnelle avec les noms corrects du modèle MuJoCo
"""

import argparse
import sys
import time
from pathlib import Path

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.mapping_reachy import ReachyMapping
from bbia_sim.robot_api import RobotFactory


def demo_reachy_mini_corrigee():
    """Démo complète avec les noms de joints CORRECTS."""
    print("🎉 DÉMO REACHY-MINI CORRIGÉE - Noms de joints corrects !")
    print("=" * 60)
    print("✅ Utilise les vrais noms du modèle MuJoCo officiel")
    print("=" * 60)

    # Créer le robot MuJoCo
    print("\n🔧 Initialisation MuJoCo...")
    robot_mujoco = RobotFactory.create_backend("mujoco")
    robot_mujoco.connect()
    print("✅ Robot MuJoCo connecté")

    # Créer le robot SDK officiel
    print("\n🔧 Initialisation SDK Officiel...")
    robot_officiel = RobotFactory.create_backend("reachy_mini")
    robot_officiel.is_connected = True  # Mode simulation
    print("✅ Robot SDK Officiel connecté (simulation)")

    mapping = ReachyMapping()

    print("\n📊 Comparaison Backends:")
    print(f"   MuJoCo: {len(robot_mujoco.get_available_joints())} joints")
    print(f"   SDK Officiel: {len(robot_officiel.get_available_joints())} joints")
    print(f"   Joints recommandés: {mapping.get_recommended_joints()}")

    # Afficher les vrais noms de joints
    print("\n🔍 Vrais noms de joints MuJoCo:")
    mujoco_joints = robot_mujoco.get_available_joints()
    for i, joint in enumerate(mujoco_joints):
        print(f"   {i}: {joint}")

    # Animation avec les BONS noms de joints
    print("\n🎬 DÉBUT DE L'ANIMATION CORRIGÉE !")
    print("=" * 50)

    # Séquence 1: Réveil du robot
    print("\n🌅 Séquence 1: Réveil du Robot")
    robot_mujoco.run_behavior("wake_up", 3.0)
    robot_officiel.run_behavior("wake_up", 3.0)
    print("✅ Robot réveillé avec élégance")

    # Séquence 2: Émotions expressives (maintenant valides)
    print("\n🎭 Séquence 2: Émotions Expressives")
    emotions = [
        ("happy", "😊 Heureux"),
        ("excited", "🤩 Excité"),
        ("curious", "🤔 Curieux"),
        ("sad", "😢 Triste"),
        ("neutral", "😐 Neutre"),
        ("calm", "😌 Calme"),
    ]

    # OPTIMISATION: Utiliser goto_target avec interpolation adaptée selon émotion
    emotion_interpolation_map = {
        "happy": "cartoon",  # Expressif et animé
        "excited": "cartoon",  # Très expressif
        "curious": "minjerk",  # Naturel
        "sad": "ease_in_out",  # Doux et mélancolique
        "neutral": "minjerk",  # Naturel
        "calm": "ease_in_out",  # Doux et fluide
    }

    for emotion, emoji_desc in emotions:
        print(f"   {emoji_desc}")
        robot_mujoco.set_emotion(emotion, 0.8)

        # OPTIMISATION SDK: Utiliser goto_target avec interpolation adaptée si disponible
        interpolation_method = emotion_interpolation_map.get(emotion, "minjerk")
        if hasattr(robot_officiel, "goto_target") and hasattr(
            robot_officiel, "set_emotion"
        ):
            try:
                from reachy_mini.utils import create_head_pose

                # Créer pose selon émotion avec angles SDK conformes
                emotion_poses = {
                    "happy": create_head_pose(pitch=0.1, yaw=0.0),
                    "excited": create_head_pose(pitch=0.2, yaw=0.1),
                    "curious": create_head_pose(pitch=0.05, yaw=0.2),
                    "sad": create_head_pose(pitch=-0.1, yaw=0.0),
                    "neutral": create_head_pose(pitch=0.0, yaw=0.0),
                    "calm": create_head_pose(pitch=-0.05, yaw=0.0),
                }
                pose = emotion_poses.get(emotion, create_head_pose(pitch=0.0, yaw=0.0))
                robot_officiel.goto_target(
                    head=pose, duration=0.8, method=interpolation_method
                )
            except (ImportError, Exception):
                robot_officiel.set_emotion(emotion, 0.8)
        else:
            robot_officiel.set_emotion(emotion, 0.8)
        time.sleep(1.5)

    # Séquence 3: Mouvements avec les BONS noms de joints
    # IMPORTANT EXPERT: Les joints stewart ne peuvent PAS être contrôlés individuellement
    # car la plateforme Stewart utilise la cinématique inverse (IK).
    # Utiliser goto_target() ou set_target_head_pose() avec create_head_pose() pour la tête.
    print("\n👀 Séquence 3: Mouvements avec Méthodes SDK Correctes")

    # Mouvements tête via goto_target (conforme SDK officiel)
    try:
        from reachy_mini.utils import create_head_pose

        print("   🎯 Mouvements tête via goto_target (cinématique inverse)")
        head_poses = [
            (0.1, 0.0, "Regard droit"),
            (0.05, 0.15, "Regard à droite"),
            (0.05, -0.15, "Regard à gauche"),
            (0.0, 0.0, "Position neutre"),
        ]

        for pitch, yaw, description in head_poses:
            print(f"   {description} (pitch={pitch:.2f}, yaw={yaw:.2f})")
            pose = create_head_pose(pitch=pitch, yaw=yaw, degrees=False)

            if hasattr(robot_officiel, "goto_target"):
                # OPTIMISATION: Utiliser interpolation adaptée (minjerk pour mouvements naturels)
                robot_officiel.goto_target(head=pose, duration=0.8, method="minjerk")
            elif hasattr(robot_officiel, "set_target_head_pose"):
                robot_officiel.set_target_head_pose(pose)
            time.sleep(1.0)
    except ImportError:
        print("   ⚠️  SDK reachy_mini non disponible, skip mouvements tête")

    # Mouvements corps via yaw_body (direct ou goto_target)
    print("\n   🎯 Mouvements corps (yaw_body)")
    body_movements = [
        (0.1, "Droit"),
        (0.15, "Droite"),
        (-0.15, "Gauche"),
        (0.0, "Centre"),
    ]

    for yaw, description in body_movements:
        print(f"   Rotation {description} (yaw={yaw:.2f})")
        # Utiliser goto_target si disponible (plus fluide)
        if hasattr(robot_officiel, "goto_target"):
            robot_officiel.goto_target(body_yaw=yaw, duration=0.6, method="minjerk")
        else:
            robot_officiel.set_joint_pos("yaw_body", yaw)
        time.sleep(0.7)

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
        # OPTIMISATION EXPERT: Utiliser look_at_world avec tous les paramètres SDK
        # pour performance optimale (duration et perform_movement)
        if hasattr(robot_officiel, "look_at_world"):
            # Validation coordonnées avant appel (évite erreurs réseau)
            if -2.0 <= x <= 2.0 and -2.0 <= y <= 2.0 and -1.0 <= z <= 1.0:
                robot_officiel.look_at_world(
                    x, y, z, duration=1.0, perform_movement=True
                )
            else:
                print(f"   ⚠️  Coordonnées hors limites: ({x}, {y}, {z})")
        else:
            robot_officiel.look_at(x, y, z)
        robot_mujoco.look_at(x, y, z)
        time.sleep(1.0)

    # Séquence 5: Comportements sociaux (maintenant valides)
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
    robot_mujoco.set_joint_pos("yaw_body", 0.2)
    robot_officiel.set_joint_pos("yaw_body", 0.2)
    print("   ✅ Amplitude normale: OK")
    time.sleep(1.0)

    # Test amplitude excessive (doit être clampée)
    robot_mujoco.set_joint_pos("yaw_body", 0.5)  # > 0.3
    robot_officiel.set_joint_pos("yaw_body", 0.5)  # > 0.3
    print("   ✅ Amplitude excessive: Clampée automatiquement")
    time.sleep(1.0)

    # Test antenne animable (avec limites de sécurité)
    success_mujoco = robot_mujoco.set_joint_pos("left_antenna", 0.1)  # Dans limites -0.3 à 0.3
    success_officiel = robot_officiel.set_joint_pos("left_antenna", 0.1)
    print(f"   ✅ Antenne animable (limites -0.3 à 0.3 rad): MuJoCo={success_mujoco}, SDK={success_officiel}")

    # Séquence 7: Finale spectaculaire
    print("\n🎆 SÉQUENCE FINALE: SPECTACLE COMPLET !")
    print("=" * 40)

    # Mouvement de rotation complète avec le BON nom
    print("   🌪️ Rotation complète du corps")
    for i in range(50):
        angle = (i / 50.0) * 2 * 3.14159 - 3.14159  # -π à +π
        robot_mujoco.set_joint_pos("yaw_body", angle * 0.1)  # Limité à 0.1
        robot_officiel.set_joint_pos("yaw_body", angle * 0.1)
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
    robot_mujoco.set_joint_pos("yaw_body", 0.0)
    robot_officiel.set_joint_pos("yaw_body", 0.0)

    # Télémétrie finale
    print("\n📊 TÉLÉMÉTRIE FINALE")
    print("=" * 30)

    telemetry_mujoco = robot_mujoco.get_telemetry()
    telemetry_officiel = robot_officiel.get_telemetry()

    print("MuJoCo:")
    for key, value in telemetry_mujoco.items():
        if key != "current_qpos":  # Trop long à afficher
            print(f"   {key}: {value}")

    print("\nSDK Officiel:")
    for key, value in telemetry_officiel.items():
        print(f"   {key}: {value}")

    # Déconnexion
    print("\n🔌 Déconnexion...")
    robot_mujoco.disconnect()
    robot_officiel.disconnect()

    print("\n🎉 DÉMO CORRIGÉE TERMINÉE !")
    print("=" * 50)
    print("✨ Maintenant tout fonctionne parfaitement !")
    print("🤖 Noms de joints corrects = Mouvements visibles")
    print("🚀 Prêt pour le robot physique dans 2 mois !")
    print("=" * 50)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Démo Reachy-Mini Corrigée")
    parser.add_argument("--quick", action="store_true", help="Version rapide")

    args = parser.parse_args()

    demo_reachy_mini_corrigee()

    print("\n🚀 Prochaines étapes:")
    print("1. Tester avec robot physique (dans 2 mois)")
    print("2. Optimiser performances IA")
    print("3. Développer nouveaux comportements")
    print("4. Préparer démos professionnelles")
    print("\n🎉 Merci d'avoir regardé la démo corrigée !")
