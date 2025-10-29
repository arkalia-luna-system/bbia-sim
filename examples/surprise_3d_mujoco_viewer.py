#!/usr/bin/env python3
"""
🎉 SURPRISE 3D BBIA-SIM - Visualisation Spectaculaire avec MuJoCo Viewer
Démonstration complète du robot Reachy-Mini avec SDK officiel
Version corrigée qui fonctionne vraiment !
"""

import argparse
import sys
import time
from pathlib import Path

import mujoco

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.mapping_reachy import ReachyMapping
from bbia_sim.robot_api import RobotFactory


def surprise_3d_mujoco_viewer():
    """Surprise 3D avec MuJoCo Viewer - Visualisation spectaculaire !"""
    print("🎉 SURPRISE 3D BBIA-SIM - Visualisation Spectaculaire !")
    print("=" * 60)
    print("🤖 Robot Reachy-Mini avec SDK Officiel + MuJoCo Viewer")
    print("🎨 Animation complète des capacités du robot")
    print("=" * 60)

    # Charger le modèle MuJoCo
    model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
    if not model_path.exists():
        print(f"❌ Modèle MuJoCo non trouvé: {model_path}")
        return

    print(f"\n🔧 Chargement modèle MuJoCo: {model_path}")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    # Créer le robot SDK officiel pour comparaison
    robot_officiel = RobotFactory.create_backend("reachy_mini")
    robot_officiel.is_connected = True  # Mode simulation

    mapping = ReachyMapping()

    print("\n📊 Comparaison:")
    print(f"   MuJoCo: {model.nq} joints")
    print(f"   SDK Officiel: {len(robot_officiel.get_available_joints())} joints")
    print(f"   Joints recommandés: {mapping.get_recommended_joints()}")

    # Debug: vérifier les positions initiales
    print("\n🔍 Debug positions initiales:")
    for joint_name in ["yaw_body", "stewart_1", "stewart_2"]:
        joint_id = get_joint_id(model, joint_name)
        if joint_id is not None:
            print(f"   {joint_name} (ID {joint_id}): {data.qpos[joint_id]:.3f}")
        else:
            print(f"   {joint_name}: NON TROUVÉ")

    # Créer le viewer MuJoCo avec contexte
    print("\n🖥️ Ouverture du viewer MuJoCo...")
    print("👀 Le viewer va s'ouvrir et rester ouvert !")
    print("🎬 Le robot va bouger automatiquement !")
    print("❌ Fermez le viewer pour arrêter")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("✅ Viewer MuJoCo ouvert !")
        print("🎬 Démarrage de l'animation spectaculaire...")

        # Animation spectaculaire
        print("\n🎬 DÉBUT DE L'ANIMATION SPECTACULAIRE !")
        print("=" * 50)
        print("👀 Regardez le viewer MuJoCo pour voir le robot bouger !")

        # Séquence 1: Réveil du robot
        print("\n🌅 Séquence 1: Réveil du Robot")
        animate_emotion(model, data, "wake_up", 3.0, viewer)
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
            animate_emotion(model, data, emotion, 1.5, viewer)
            robot_officiel.set_emotion(emotion, 0.8)
            time.sleep(1.5)

        # Séquence 3: Mouvements de tête complexes
        # IMPORTANT EXPERT: Les joints stewart ne peuvent PAS être contrôlés individuellement
        # car la plateforme Stewart utilise la cinématique inverse (IK).
        # Utiliser goto_target() ou set_target_head_pose() avec create_head_pose() pour la tête.
        print("\n👀 Séquence 3: Mouvements de Tête Complexes (SDK Correct)")
        try:
            from reachy_mini.utils import create_head_pose

            head_poses = [
                (0.1, 0.0, "Regard droit"),
                (0.05, 0.15, "Regard à droite"),
                (0.05, -0.15, "Regard à gauche"),
                (0.0, 0.0, "Position neutre"),
            ]

            for pitch, yaw, description in head_poses:
                print(f"   {description} (pitch={pitch:.2f}, yaw={yaw:.2f})")
                pose = create_head_pose(pitch=pitch, yaw=yaw, degrees=False)

                # Utiliser goto_target pour MuJoCo (simulation bas niveau OK pour viewer)
                animate_head_pose_mujoco(model, data, pitch, yaw, 2.0, viewer)

                # Utiliser méthode SDK correcte pour robot officiel
                if hasattr(robot_officiel, "goto_target"):
                    robot_officiel.goto_target(
                        head=pose, duration=0.8, method="minjerk"
                    )
                elif hasattr(robot_officiel, "set_target_head_pose"):
                    robot_officiel.set_target_head_pose(pose)
                time.sleep(0.5)

            # Mouvement corps (seul joint directement contrôlable)
            print("   Rotation corps (yaw_body)")
            if hasattr(robot_officiel, "goto_target"):
                robot_officiel.goto_target(body_yaw=0.1, duration=0.6, method="minjerk")
            else:
                robot_officiel.set_joint_pos("yaw_body", 0.1)
            animate_joint(model, data, "yaw_body", 2.0, viewer)
            time.sleep(0.5)
        except ImportError:
            print("   ⚠️  SDK reachy_mini non disponible, skip mouvements tête SDK")
            # Fallback: animation MuJoCo seulement (simulation bas niveau)
            animate_joint(model, data, "yaw_body", 2.0, viewer)

        # Séquence 4: Rotation complète du corps
        print("\n🌪️ Séquence 4: Rotation Complète du Corps")
        animate_body_rotation(model, data, 5.0, viewer)
        print("✅ Rotation spectaculaire terminée")

        # Séquence 5: Comportements sociaux
        print("\n🤝 Séquence 5: Comportements Sociaux")
        behaviors = [
            ("nod", "Hochement de tête"),
            ("wake_up", "Réveil expressif"),
            ("goto_sleep", "Endormissement"),
        ]

        for behavior, description in behaviors:
            print(f"   {description}")
            animate_behavior(model, data, behavior, 2.0, viewer)
            robot_officiel.run_behavior(behavior, 2.0)
            time.sleep(1.0)

        # Séquence 6: Finale spectaculaire
        print("\n🎆 SÉQUENCE FINALE: SPECTACLE COMPLET !")
        print("=" * 40)

        # Mouvement de rotation complète
        print("   🌪️ Rotation complète du corps")
        animate_body_rotation(model, data, 3.0, viewer)

        # Émotion finale
        print("   🎉 Émotion finale: EXCITED !")
        animate_emotion(model, data, "excited", 2.0, viewer)
        robot_officiel.set_emotion("excited", 1.0)
        time.sleep(2.0)

        # Retour au neutre
        print("   😌 Retour au neutre")
        animate_emotion(model, data, "neutral", 1.0, viewer)
        robot_officiel.set_emotion("neutral", 0.5)

        print("\n🎉 SURPRISE TERMINÉE !")
        print("=" * 50)
        print("✨ Vous avez vu le robot Reachy-Mini dans toute sa splendeur !")
        print("🤖 SDK Officiel + MuJoCo = Performance maximale")
        print("🚀 Prêt pour le robot physique dans 2 mois !")
        print("=" * 50)


def animate_head_pose_mujoco(model, data, pitch, yaw, duration, viewer):
    """Anime une pose de tête via MuJoCo direct (simulation bas niveau uniquement).

    ⚠️ IMPORTANT EXPERT: Cette fonction utilise data.qpos[stewart_*] directement
    uniquement pour la simulation MuJoCo de bas niveau (visualisation).
    Pour le robot physique ou avec le SDK, utiliser goto_target() ou
    set_target_head_pose() avec create_head_pose() (cinématique inverse requise).

    Args:
        model: Modèle MuJoCo
        data: Données MuJoCo
        pitch: Pitch en radians
        yaw: Yaw en radians
        duration: Durée animation
        viewer: Viewer MuJoCo
    """
    # Approximation pour MuJoCo direct: stewart_1 ≈ pitch, stewart_2 ≈ yaw
    # NOTE: Cette approximation est uniquement pour simulation MuJoCo bas niveau.
    # Le robot réel nécessite IK via goto_target() ou create_head_pose().
    steps = int(duration * 100)  # 100 Hz

    for _step in range(steps):
        # Animation douce vers la cible
        stewart_1_id = get_joint_id(model, "stewart_1")
        stewart_2_id = get_joint_id(model, "stewart_2")

        if stewart_1_id is not None:
            current_pitch = data.qpos[stewart_1_id]
            data.qpos[stewart_1_id] = current_pitch + (pitch - current_pitch) * 0.1
        if stewart_2_id is not None:
            current_yaw = data.qpos[stewart_2_id]
            data.qpos[stewart_2_id] = current_yaw + (yaw - current_yaw) * 0.1

        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(0.01)


def animate_emotion(model, data, emotion, duration, viewer):
    """Anime une émotion sur le robot MuJoCo (simulation bas niveau uniquement).

    ⚠️ IMPORTANT EXPERT: Cette fonction utilise data.qpos[stewart_*] directement
    uniquement pour la simulation MuJoCo de bas niveau (visualisation viewer).
    Pour le robot physique ou avec le SDK, utiliser set_emotion() ou goto_target()
    avec create_head_pose() (cinématique inverse requise).

    NOTE: L'utilisation de stewart_1/stewart_2 ici est une approximation simplifiée
    pour MuJoCo direct. Le robot réel nécessite IK.
    """
    # Mapping émotions vers mouvements (AMPLITUDES RÉDUITES)
    # NOTE: Utilisation stewart_1/stewart_2 uniquement pour MuJoCo direct (viewer)
    emotion_movements = {
        "happy": {"yaw_body": 0.1, "stewart_1": 0.05, "stewart_2": 0.05},
        "sad": {"yaw_body": -0.1, "stewart_1": -0.05, "stewart_2": -0.05},
        "excited": {"yaw_body": 0.15, "stewart_1": 0.1, "stewart_2": 0.1},
        "curious": {"yaw_body": 0.08, "stewart_2": 0.1},
        "neutral": {"yaw_body": 0.0, "stewart_1": 0.0, "stewart_2": 0.0},
        "calm": {"yaw_body": -0.05, "stewart_1": -0.05},
        "wake_up": {"yaw_body": 0.1, "stewart_1": 0.05, "stewart_2": 0.05},
        "goto_sleep": {"yaw_body": 0.0, "stewart_1": -0.1, "stewart_2": -0.1},
    }

    if emotion not in emotion_movements:
        return

    movements = emotion_movements[emotion]
    steps = int(duration * 100)  # 100 Hz

    for _step in range(steps):
        for joint_name, target_pos in movements.items():
            joint_id = get_joint_id(model, joint_name)
            if joint_id is not None:
                # Animation douce vers la position cible
                current_pos = data.qpos[joint_id]
                data.qpos[joint_id] = current_pos + (target_pos - current_pos) * 0.1

        # Mettre à jour la simulation et rendre
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)  # Calculer les positions
        viewer.sync()  # Synchroniser avec le viewer
        time.sleep(0.01)


def animate_joint(model, data, joint_name, duration, viewer, amplitude=0.1):
    """Anime un joint spécifique."""
    joint_id = get_joint_id(model, joint_name)
    if joint_id is None:
        return

    steps = int(duration * 100)  # 100 Hz

    for step in range(steps):
        progress = step / steps
        # Mouvement sinusoïdal
        angle = amplitude * (2 * progress - 1)  # -amplitude à +amplitude
        data.qpos[joint_id] = angle

        # Mettre à jour la simulation et rendre
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)  # Calculer les positions
        viewer.sync()  # Synchroniser avec le viewer
        time.sleep(0.01)


def animate_body_rotation(model, data, duration, viewer):
    """Anime une rotation complète du corps."""
    joint_id = get_joint_id(model, "yaw_body")
    if joint_id is None:
        return

    steps = int(duration * 100)  # 100 Hz

    for step in range(steps):
        progress = step / steps
        # Rotation complète (AMPLITUDE RÉDUITE)
        angle = (progress * 2 * 3.14159 - 3.14159) * 0.05  # Limité à 0.05
        data.qpos[joint_id] = angle

        # Mettre à jour la simulation et rendre
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)  # Calculer les positions
        viewer.sync()  # Synchroniser avec le viewer
        time.sleep(0.01)


def animate_behavior(model, data, behavior, duration, viewer):
    """Anime un comportement spécifique (simulation MuJoCo direct uniquement).

    ⚠️ IMPORTANT EXPERT: Cette fonction utilise data.qpos[stewart_*] directement
    uniquement pour la simulation MuJoCo de bas niveau. Pour le robot réel,
    utiliser les comportements SDK (wake_up, nod, goto_sleep) qui gèrent IK automatiquement.
    """
    if behavior == "nod":
        # Hochement de tête (approximation MuJoCo direct)
        # NOTE: Pour robot réel, utiliser robot.run_behavior("nod") qui utilise IK
        joint_id = get_joint_id(model, "stewart_1")
        if joint_id is not None:
            for _ in range(3):  # 3 hochements (AMPLITUDE RÉDUITE)
                data.qpos[joint_id] = 0.05
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(0.3)
                data.qpos[joint_id] = -0.05
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(0.3)
                data.qpos[joint_id] = 0.0
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(0.2)
    elif behavior == "wake_up":
        animate_emotion(model, data, "wake_up", duration, viewer)
    elif behavior == "goto_sleep":
        animate_emotion(model, data, "goto_sleep", duration, viewer)


def get_joint_id(model, joint_name):
    """Récupère l'ID d'un joint par son nom."""
    try:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        return joint_id if joint_id >= 0 else None
    except Exception:
        return None


def main():
    """Point d'entrée principal."""
    parser = argparse.ArgumentParser(
        description="Surprise 3D BBIA-SIM avec MuJoCo Viewer"
    )
    parser.add_argument("--quick", action="store_true", help="Version rapide")

    parser.parse_args()

    try:
        surprise_3d_mujoco_viewer()
    except KeyboardInterrupt:
        print("\n\n👋 Animation interrompue par l'utilisateur")
    except Exception as e:
        print(f"\n❌ Erreur: {e}")

    print("\n🚀 Prochaines étapes:")
    print("1. Tester avec robot physique (dans 2 mois)")
    print("2. Optimiser performances IA")
    print("3. Développer nouveaux comportements")
    print("4. Préparer démos professionnelles")
    print("\n🎉 Merci d'avoir regardé la surprise 3D !")


if __name__ == "__main__":
    main()
