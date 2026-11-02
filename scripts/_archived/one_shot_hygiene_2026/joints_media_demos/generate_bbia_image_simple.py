#!/usr/bin/env python3
"""
📸 Script simple pour générer une image PNG de BBIA
Ouvre le viewer MuJoCo avec une pose sympa, vous pouvez faire une capture d'écran
"""

import sys
import time
from pathlib import Path

import mujoco

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def generate_bbia_image_viewer(pose: str = "happy"):
    """Ouvre le viewer MuJoCo avec une pose sympa pour capture d'écran.

    Args:
        pose: Pose du robot ("happy", "neutral", "curious", "excited")
    """
    print("📸 Génération image BBIA - Viewer MuJoCo")
    print("=" * 60)

    # Charger le modèle MuJoCo
    model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
    if not model_path.exists():
        print(f"❌ Modèle MuJoCo non trouvé: {model_path}")
        return

    print(f"📂 Chargement modèle: {model_path}")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    print(f"✅ Modèle chargé: {model.nq} joints")

    # Configurer la pose selon l'émotion
    poses = {
        "happy": {
            "yaw_body": 0.1,
            "stewart_1": 0.05,
            "stewart_2": 0.05,
        },
        "neutral": {
            "yaw_body": 0.0,
            "stewart_1": 0.0,
            "stewart_2": 0.0,
        },
        "curious": {
            "yaw_body": 0.08,
            "stewart_2": 0.1,
        },
        "excited": {
            "yaw_body": 0.15,
            "stewart_1": 0.1,
            "stewart_2": 0.1,
        },
    }

    if pose not in poses:
        pose = "happy"
        print(f"⚠️  Pose '{pose}' inconnue, utilisation de 'happy'")

    print(f"🎭 Configuration pose: {pose}")

    # Appliquer la pose
    pose_config = poses[pose]
    for joint_name, angle in pose_config.items():
        try:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id >= 0:
                data.qpos[joint_id] = angle
        except Exception:
            pass

    # Mettre à jour la simulation
    mujoco.mj_forward(model, data)

    # Ouvrir le viewer
    print("\n🖥️  Ouverture du viewer MuJoCo...")
    print("📸 Instructions pour capture d'écran:")
    print("   • Sur macOS: ⌘ + ⇧ + 4 puis sélectionnez la fenêtre")
    print("   • Ou: ⌘ + ⇧ + 3 pour capture plein écran")
    print("   • L'image sera sauvegardée sur votre Bureau")
    print("   • Fermez le viewer (Échap) pour quitter")
    print()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Configurer la caméra à 180° (face optimal) immédiatement
            viewer.cam.azimuth = 180.0
            viewer.cam.elevation = -15.0
            viewer.cam.distance = 1.2  # Rapproché
            viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
            viewer.sync()

            print("✅ Viewer ouvert !")
            print("📸 Faites votre capture d'écran maintenant...")
            print("❌ Fermez le viewer (Échap) pour quitter")

            # Garder le viewer ouvert
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.01)

    except Exception as e:
        print(f"❌ Erreur: {e}")
        print("\n💡 Sur macOS, essayez avec mjpython:")
        print("   mjpython scripts/generate_bbia_image_simple.py")


def main():
    """Point d'entrée principal."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Ouvre le viewer MuJoCo pour capture d'écran BBIA"
    )
    parser.add_argument(
        "--pose",
        "-p",
        choices=["happy", "neutral", "curious", "excited"],
        default="happy",
        help="Pose du robot (défaut: happy)",
    )

    args = parser.parse_args()

    try:
        generate_bbia_image_viewer(pose=args.pose)
        print("\n🎉 Viewer fermé")
        return 0

    except KeyboardInterrupt:
        print("\n\n👋 Interrompu par l'utilisateur")
        return 1
    except Exception as e:
        print(f"\n❌ Erreur: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
