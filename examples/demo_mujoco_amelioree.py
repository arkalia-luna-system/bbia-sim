#!/usr/bin/env python3
"""
🎉 DÉMO MUJOCO AMÉLIORÉE - Robot qui bouge de manière très visible !
Version avec mouvements plus grands et vérifications améliorées
"""

import math
import sys
import time
from pathlib import Path

import mujoco

try:
    import mujoco.viewer
except ImportError:
    try:
        from mujoco import viewer as mujoco_viewer

        mujoco.viewer = mujoco_viewer
    except ImportError:
        print("❌ MuJoCo viewer non disponible dans cette version")
        exit(1)

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def get_joint_id(model, joint_name):
    """Récupère l'ID d'un joint par son nom."""
    try:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        return joint_id if joint_id >= 0 else None
    except Exception:
        return None


def demo_mujoco_amelioree():
    """Démo améliorée avec mouvements très visibles !"""
    print("🎉 DÉMO MUJOCO AMÉLIORÉE - Robot qui bouge de manière très visible !")
    print("=" * 70)
    print("🤖 Robot Reachy-Mini avec animations fluides et visibles")
    print("=" * 70)

    # Charger le modèle MuJoCo directement
    model_path = Path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")
    if not model_path.exists():
        print(f"❌ Modèle MuJoCo non trouvé: {model_path}")
        return

    print(f"\n🔧 Chargement modèle MuJoCo: {model_path}")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    print(f"✅ Modèle chargé: {model.nq} joints, {model.njnt} joints nommés")

    # Afficher les joints disponibles et leurs IDs
    print("\n🔍 Joints disponibles:")
    joint_map = {}
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            joint_id = get_joint_id(model, name)
            joint_map[name] = joint_id
            print(
                f"   {name}: ID={joint_id}, qpos_index={model.jnt_qposadr[i] if joint_id is not None else 'N/A'}"
            )

    # Vérifier les joints critiques
    body_id = get_joint_id(model, "yaw_body")
    stewart_1_id = get_joint_id(model, "stewart_1")
    stewart_2_id = get_joint_id(model, "stewart_2")
    stewart_3_id = get_joint_id(model, "stewart_3")

    print("\n✅ Joints critiques trouvés:")
    print(f"   yaw_body: {body_id}")
    print(f"   stewart_1: {stewart_1_id}")
    print(f"   stewart_2: {stewart_2_id}")
    print(f"   stewart_3: {stewart_3_id}")

    # Lancer le viewer MuJoCo avec contexte
    print("\n🖥️ Lancement du viewer MuJoCo...")
    print("👀 Le viewer va s'ouvrir et rester ouvert !")
    print("🎬 Le robot va bouger automatiquement avec des mouvements VISIBLES !")
    print("❌ Fermez le viewer pour arrêter")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Configurer la caméra à 180° (face optimal) immédiatement
        viewer.cam.azimuth = 180.0
        viewer.cam.elevation = -15.0
        viewer.cam.distance = 1.2
        viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
        viewer.sync()

        print("✅ Viewer MuJoCo ouvert !")
        print("🎬 Démarrage de l'animation améliorée...")
        print("💡 Les mouvements sont maintenant PLUS VISIBLES !")

        # Animation continue avec mouvements plus grands
        step_count = 0
        start_time = time.time()

        while viewer.is_running():
            current_time = time.time() - start_time

            # Animation du corps (yaw_body) - AMPLITUDE AUGMENTÉE
            if body_id is not None:
                # Rotation sinusoïdale plus visible (amplitude 0.3 rad = ~17°)
                body_angle = 0.3 * math.sin(current_time * 0.8)
                qpos_idx = model.jnt_qposadr[body_id]
                data.qpos[qpos_idx] = body_angle

            # Animation de la tête (stewart joints) - AMPLITUDE AUGMENTÉE
            if stewart_1_id is not None:
                head_pitch = 0.15 * math.sin(current_time * 1.2)
                qpos_idx = model.jnt_qposadr[stewart_1_id]
                data.qpos[qpos_idx] = head_pitch

            if stewart_2_id is not None:
                head_yaw = 0.15 * math.cos(current_time * 1.0)
                qpos_idx = model.jnt_qposadr[stewart_2_id]
                data.qpos[qpos_idx] = head_yaw

            if stewart_3_id is not None:
                head_roll = 0.1 * math.sin(current_time * 0.6)
                qpos_idx = model.jnt_qposadr[stewart_3_id]
                data.qpos[qpos_idx] = head_roll

            # CRITIQUE: Mettre à jour la simulation dans le bon ordre
            mujoco.mj_forward(model, data)  # Calculer les positions
            mujoco.mj_step(model, data)  # Avancer la simulation

            # Synchroniser avec le viewer
            viewer.sync()

            step_count += 1

            # Affichage périodique avec valeurs actuelles
            if step_count % 100 == 0:
                body_val = (
                    data.qpos[model.jnt_qposadr[body_id]] if body_id is not None else 0
                )
                print(
                    f"🔄 Step {step_count} | t={current_time:.1f}s | yaw_body={body_val:.3f} rad"
                )

            # Délai pour 60 FPS
            time.sleep(1.0 / 60.0)

    print("\n✅ Viewer fermé")
    print("🎉 Démo améliorée terminée !")


if __name__ == "__main__":
    try:
        demo_mujoco_amelioree()
    except KeyboardInterrupt:
        print("\n\n👋 Démo interrompue par l'utilisateur")
    except Exception as e:
        import traceback

        print(f"\n❌ Erreur: {e}")
        traceback.print_exc()

    print("\n🚀 Prochaines étapes:")
    print("1. Tester avec robot physique (dans 2 mois)")
    print("2. Optimiser performances IA")
    print("3. Développer nouveaux comportements")
    print("4. Préparer démos professionnelles")
    print("\n🎉 Merci d'avoir regardé la démo améliorée !")
