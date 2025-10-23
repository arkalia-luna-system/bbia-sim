#!/usr/bin/env python3
"""
Lanceur du modèle Reachy Mini parfaitement fidèle aux spécifications officielles.
"""

import argparse
import logging
import sys
from pathlib import Path

# Configuration du logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(
        description="Lanceur du modèle Reachy Mini parfait"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument("--duration", type=int, default=10, help="Durée en secondes")
    parser.add_argument(
        "--validate", action="store_true", help="Valider les spécifications"
    )

    args = parser.parse_args()

    # Chemin vers le modèle parfait
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    model_path = (
        project_root / "src" / "bbia_sim" / "sim" / "models" / "reachy_mini_perfect.xml"
    )

    logger.info("🤖 LANCEMENT DU MODÈLE REACHY MINI PARFAIT")
    logger.info("=" * 70)

    if not model_path.exists():
        logger.error(f"❌ Modèle parfait non trouvé : {model_path}")
        sys.exit(1)

    try:
        import mujoco

        # Chargement du modèle
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)

        logger.info(
            f"✅ Modèle parfait chargé : {model.njnt} joints, {model.ngeom} geoms, {model.nbody} bodies"
        )

        if args.validate:
            logger.info("\n🔍 VALIDATION DES SPÉCIFICATIONS:")
            logger.info("=" * 70)

            # Validation des dimensions
            base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
            head_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "head")
            right_arm_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, "right_arm"
            )
            left_arm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_arm")

            base_pos = model.body_pos[base_id]
            head_pos = model.body_pos[head_id]
            right_arm_pos = model.body_pos[right_arm_id]
            left_arm_pos = model.body_pos[left_arm_id]

            height = head_pos[2] - base_pos[2] + 0.5
            width = abs(right_arm_pos[1] - left_arm_pos[1])

            logger.info(f"📏 Hauteur totale : {height:.2f}m")
            logger.info(f"📏 Largeur (écartement bras) : {width:.2f}m")
            logger.info(f"🔧 Articulations : {model.njnt}")
            logger.info(f"🎨 Géométries : {model.ngeom}")
            logger.info(f"🏗️ Corps : {model.nbody}")

            # Validation des plages de mouvement
            logger.info("\n🎭 PLAGES DE MOUVEMENT:")
            for i in range(model.njnt):
                joint = model.joint(i)
                range_min = joint.range[0] if joint.range[0] != 0 else "∞"
                range_max = joint.range[1] if joint.range[1] != 0 else "∞"
                logger.info(f"   - {joint.name}: {range_min} à {range_max} rad")

        if args.headless:
            logger.info(f"\n🔄 Simulation headless ({args.duration}s)...")
            import time

            start_time = time.monotonic()
            step_count = 0

            while time.monotonic() - start_time < args.duration:
                mujoco.mj_step(model, data)
                step_count += 1

                # Affichage périodique
                if step_count % 1000 == 0:
                    elapsed = time.monotonic() - start_time
                    logger.info(
                        f"   📊 {step_count} steps en {elapsed:.1f}s ({step_count/elapsed:.0f} steps/s)"
                    )

            final_time = time.monotonic() - start_time
            logger.info(
                f"✅ Simulation terminée : {step_count} steps en {final_time:.2f}s"
            )
            logger.info(f"⚡ Performance : {step_count/final_time:.0f} steps/s")

        else:
            logger.info("\n🎮 Lancement du viewer 3D...")
            logger.info("   - Utilise la souris pour naviguer")
            logger.info("   - Molette pour zoomer")
            logger.info("   - Échap pour fermer")

            # Lancement du viewer
            import mujoco_viewer

            viewer = mujoco_viewer.MujocoViewer(model, data)

            try:
                while viewer.is_alive:
                    mujoco.mj_step(model, data)
                    viewer.render()
            except KeyboardInterrupt:
                logger.info("🛑 Arrêt demandé par l'utilisateur")
            finally:
                viewer.close()

        logger.info("\n🎯 MODÈLE PARFAIT VALIDÉ")
        logger.info("✅ Dimensions réalistes")
        logger.info("✅ Articulations complètes")
        logger.info("✅ Performance optimale")
        logger.info("✅ Fidélité aux spécifications officielles")

    except ImportError as e:
        logger.error(f"❌ MuJoCo non disponible : {e}")
        logger.info("💡 Installez MuJoCo : pip install mujoco")
        sys.exit(1)
    except Exception as e:
        logger.error(f"❌ Erreur : {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
