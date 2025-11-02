#!/usr/bin/env python3
"""
🎉 DÉMO MUJOCO CONTINUE - Robot qui bouge en continu !
Le viewer reste ouvert et le robot bouge tout seul
"""

import math
import sys
import time
from pathlib import Path

import mujoco

try:
    import mujoco.viewer
except ImportError:
    # Fallback pour versions récentes de MuJoCo
    try:
        from mujoco import viewer as mujoco_viewer

        mujoco.viewer = mujoco_viewer
    except ImportError:
        print("❌ MuJoCo viewer non disponible dans cette version")
        exit(1)

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def demo_mujoco_continue():
    """Démo continue avec MuJoCo qui reste ouvert !"""
    print("🎉 DÉMO MUJOCO CONTINUE - Robot qui bouge en continu !")
    print("=" * 60)
    print("🤖 Robot Reachy-Mini avec MuJoCo qui reste ouvert")
    print("=" * 60)

    # Charger le modèle MuJoCo directement
    model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
    if not model_path.exists():
        print(f"❌ Modèle MuJoCo non trouvé: {model_path}")
        return

    print(f"\n🔧 Chargement modèle MuJoCo: {model_path}")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    print(f"✅ Modèle chargé: {model.nq} joints")

    # Afficher les joints disponibles
    print("\n🔍 Joints disponibles:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            print(f"   {i}: {name}")

    # Fonction pour obtenir l'ID d'un joint
    def get_joint_id(joint_name):
        try:
            return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        except Exception:
            return None

    # Lancer le viewer MuJoCo avec contexte
    print("\n🖥️ Lancement du viewer MuJoCo...")
    print("👀 Le viewer va s'ouvrir et rester ouvert !")
    print("🎬 Le robot va bouger automatiquement !")
    print("❌ Fermez le viewer pour arrêter")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Configurer la caméra à 180° (face optimal) immédiatement
        viewer.cam.azimuth = 180.0
        viewer.cam.elevation = -15.0
        viewer.cam.distance = 1.2  # Rapproché de 20%
        viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
        viewer.sync()

        print("✅ Viewer MuJoCo ouvert !")
        print("🎬 Démarrage de l'animation continue...")

        # Animation continue
        step_count = 0
        start_time = time.time()

        while True:
            current_time = time.time() - start_time

            # Animation du corps (yaw_body) - AMPLITUDE RÉDUITE
            body_id = get_joint_id("yaw_body")
            if body_id is not None:
                # Rotation sinusoïdale lente et douce
                body_angle = 0.1 * math.sin(current_time * 0.5)
                data.qpos[body_id] = body_angle

            # Animation de la tête (stewart joints) - AMPLITUDE RÉDUITE
            # ⚠️ IMPORTANT EXPERT: Utilisation stewart_* directement uniquement pour MuJoCo direct (viewer)
            # Pour robot physique/SDK, utiliser goto_target() ou set_target_head_pose() avec create_head_pose() (IK requise)
            # NOTE: Approximation stewart_1≈pitch, stewart_2≈yaw valide uniquement en sim MuJoCo bas niveau
            stewart_joints = ["stewart_1", "stewart_2", "stewart_3"]
            for i, joint_name in enumerate(stewart_joints):
                joint_id = get_joint_id(joint_name)
                if joint_id is not None:
                    # Mouvement sinusoïdal avec décalage (plus doux)
                    head_angle = 0.05 * math.sin(current_time * (0.8 + i * 0.2))
                    data.qpos[joint_id] = head_angle  # MuJoCo direct uniquement

            # Mettre à jour la simulation
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)

            # Synchroniser avec le viewer
            viewer.sync()

            step_count += 1

            # Affichage périodique
            if step_count % 100 == 0:
                print(
                    f"🔄 Animation continue... {step_count} steps, {current_time:.1f}s"
                )

            # Petite pause pour éviter de surcharger
            time.sleep(0.01)

    print("\n✅ Viewer fermé")
    print("🎉 Démo continue terminée !")


if __name__ == "__main__":
    try:
        demo_mujoco_continue()
    except KeyboardInterrupt:
        print("\n\n👋 Démo interrompue par l'utilisateur")
    except Exception as e:
        print(f"\n❌ Erreur: {e}")

    print("\n🚀 Prochaines étapes:")
    print("1. Tester avec robot physique (dans 2 mois)")
    print("2. Optimiser performances IA")
    print("3. Développer nouveaux comportements")
    print("4. Préparer démos professionnelles")
    print("\n🎉 Merci d'avoir regardé la démo continue !")
