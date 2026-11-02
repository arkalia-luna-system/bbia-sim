#!/usr/bin/env python3
"""
📸 Génère une image PNG de BBIA depuis MuJoCo
Crée une belle image du robot Reachy Mini pour présentation
"""

import sys
from pathlib import Path

import mujoco
from PIL import Image

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def generate_bbia_image(
    output_path: str = "assets/images/bbia_robot.png",
    width: int = 1920,
    height: int = 1080,
    pose: str = "happy",
) -> bool:
    """Génère une image PNG du robot BBIA.

    Args:
        output_path: Chemin de sortie pour l'image PNG
        width: Largeur de l'image (défaut: 1920)
        height: Hauteur de l'image (défaut: 1080)
        pose: Pose du robot ("happy", "neutral", "curious", "excited")

    Returns:
        True si succès, False sinon
    """
    print("📸 Génération image BBIA...")
    print("=" * 60)

    # Charger le modèle MuJoCo
    model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
    if not model_path.exists():
        print(f"❌ Modèle MuJoCo non trouvé: {model_path}")
        return False

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

    # Créer le renderer
    print(f"🎨 Rendu image {width}x{height}...")
    try:
        renderer = mujoco.Renderer(model, height=height, width=width)
    except Exception as e:
        print(f"❌ Erreur création renderer: {e}")
        print("💡 Essayez d'installer: pip install mujoco")
        return False

    # Configurer la caméra (vue face optimale)
    # L'API Renderer : on configure la caméra via renderer.scene.camera
    renderer.scene.camera.azimuth = 180.0  # Face au robot
    renderer.scene.camera.elevation = -15.0  # Légèrement au-dessus
    renderer.scene.camera.distance = 1.2  # Distance rapprochée
    renderer.scene.camera.lookat[:] = [0.0, 0.0, 0.3]  # Point de regard

    # Mettre à jour la scène avec les données
    renderer.update_scene(data)

    # Rendre l'image
    image = renderer.render()

    # Convertir en PIL Image et sauvegarder
    print(f"💾 Sauvegarde: {output_path}")
    output_file = Path(output_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)

    # Convertir numpy array (RGB) en PIL Image
    pil_image = Image.fromarray(image)
    pil_image.save(output_path, "PNG")

    file_size = output_file.stat().st_size / 1024
    print(f"✅ Image sauvegardée: {output_path}")
    print(f"📊 Taille: {file_size:.1f} KB")
    print(f"📐 Dimensions: {width}x{height}")

    return True


def main():
    """Point d'entrée principal."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Génère une image PNG de BBIA depuis MuJoCo"
    )
    parser.add_argument(
        "--output",
        "-o",
        default="assets/images/bbia_robot.png",
        help="Chemin de sortie (défaut: assets/images/bbia_robot.png)",
    )
    parser.add_argument(
        "--width",
        "-w",
        type=int,
        default=1920,
        help="Largeur de l'image (défaut: 1920)",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=1080,
        help="Hauteur de l'image (défaut: 1080)",
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
        success = generate_bbia_image(
            output_path=args.output,
            width=args.width,
            height=args.height,
            pose=args.pose,
        )

        if success:
            print("\n🎉 Image générée avec succès !")
            print(f"📁 Fichier: {Path(args.output).absolute()}")
            return 0
        else:
            print("\n❌ Échec de génération")
            return 1

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
