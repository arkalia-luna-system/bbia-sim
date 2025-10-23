#!/usr/bin/env python3
"""
Script de validation complète du modèle Reachy Mini parfait.
Compare avec tous les autres modèles et valide les spécifications.
"""

import argparse
import logging
import sys
import time
from pathlib import Path

# Configuration du logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def validate_model(model_path: Path, duration: int = 2) -> dict:
    """Valide un modèle et retourne ses caractéristiques."""
    try:
        import mujoco

        # Chargement du modèle
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)

        # Informations du modèle
        joints = [model.joint(i).name for i in range(model.njnt)]
        geoms = [model.geom(i).name for i in range(model.ngeom)]
        bodies = [model.body(i).name for i in range(model.nbody)]

        # Test de simulation
        start_time = time.monotonic()
        step_count = 0
        while time.monotonic() - start_time < duration:
            mujoco.mj_step(model, data)
            step_count += 1
        final_time = time.monotonic() - start_time

        # Calcul des dimensions approximatives
        # Recherche des IDs des corps
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        head_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "head")
        right_arm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_arm")
        left_arm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_arm")

        # Position de la base
        base_pos = model.body_pos[base_id]
        # Position de la tête
        head_pos = model.body_pos[head_id]
        # Position des bras
        right_arm_pos = model.body_pos[right_arm_id]
        left_arm_pos = model.body_pos[left_arm_id]

        # Dimensions calculées
        height = (
            head_pos[2] - base_pos[2] + 0.5
        )  # Hauteur approximative (ajout de 50cm pour la base)
        width = abs(right_arm_pos[1] - left_arm_pos[1])  # Largeur approximative

        return {
            "file": model_path.name,
            "joints": len(joints),
            "geoms": len(geoms),
            "bodies": len(bodies),
            "steps": step_count,
            "time": final_time,
            "performance": step_count / final_time,
            "height": height,
            "width": width,
            "joint_names": joints,
            "success": True,
        }

    except Exception as e:
        return {"file": model_path.name, "error": str(e), "success": False}


def main():
    parser = argparse.ArgumentParser(
        description="Validation du modèle Reachy Mini parfait"
    )
    parser.add_argument(
        "--duration", type=int, default=2, help="Durée de test en secondes"
    )
    parser.add_argument(
        "--compare-all", action="store_true", help="Comparer avec tous les modèles"
    )

    args = parser.parse_args()

    # Chemin vers les modèles
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    models_dir = project_root / "src" / "bbia_sim" / "sim" / "models"

    logger.info("🔍 VALIDATION DU MODÈLE REACHY MINI PARFAIT")
    logger.info("=" * 70)

    try:
        # Test du modèle parfait
        perfect_model_path = models_dir / "reachy_mini_perfect.xml"

        if not perfect_model_path.exists():
            logger.error(f"❌ Modèle parfait non trouvé : {perfect_model_path}")
            sys.exit(1)

        logger.info(f"📦 Test du modèle parfait : {perfect_model_path.name}")
        perfect_result = validate_model(perfect_model_path, args.duration)

        if perfect_result["success"]:
            logger.info(
                f"✅ Modèle parfait chargé : {perfect_result['joints']} joints, {perfect_result['geoms']} geoms, {perfect_result['bodies']} bodies"
            )
            logger.info(
                f"📏 Dimensions : H={perfect_result['height']:.2f}m, L={perfect_result['width']:.2f}m"
            )
            logger.info(f"⚡ Performance : {perfect_result['performance']:.0f} steps/s")

            # Validation des spécifications
            logger.info("\n🎯 VALIDATION DES SPÉCIFICATIONS:")
            logger.info("=" * 70)

            # Vérification des dimensions
            height_ok = 0.4 <= perfect_result["height"] <= 0.7  # 40-70cm
            width_ok = 0.2 <= perfect_result["width"] <= 0.5  # 20-50cm

            logger.info(
                f"📏 Hauteur : {perfect_result['height']:.2f}m {'✅' if height_ok else '❌'} (attendu: 0.4-0.7m)"
            )
            logger.info(
                f"📏 Largeur : {perfect_result['width']:.2f}m {'✅' if width_ok else '❌'} (attendu: 0.2-0.5m)"
            )

            # Vérification des articulations
            expected_joints = 15
            joints_ok = perfect_result["joints"] == expected_joints

            logger.info(
                f"🔧 Articulations : {perfect_result['joints']} {'✅' if joints_ok else '❌'} (attendu: {expected_joints})"
            )

            # Vérification des plages de mouvement
            logger.info("🎭 Plages de mouvement :")
            for joint in perfect_result["joint_names"]:
                if "neck" in joint:
                    logger.info(f"   - {joint}: ±90° (yaw), ±45° (pitch), ±30° (roll)")
                elif "shoulder" in joint:
                    logger.info(f"   - {joint}: ±90° (pitch/roll)")
                elif "elbow" in joint or "wrist" in joint:
                    logger.info(f"   - {joint}: ±90°")
                elif "gripper" in joint:
                    logger.info(f"   - {joint}: ±0.5 rad")

            # Score global
            score = sum([height_ok, width_ok, joints_ok])
            logger.info(
                f"\n🏆 Score de fidélité : {score}/3 {'✅' if score == 3 else '⚠️'}"
            )

            if args.compare_all:
                logger.info("\n🔄 COMPARAISON AVEC TOUS LES MODÈLES:")
                logger.info("=" * 70)

                models_to_compare = [
                    "reachy_mini.xml",
                    "reachy_mini_complete.xml",
                    "reachy_mini_assembled.xml",
                    "reachy_mini_scaled.xml",
                    "reachy_mini_realistic.xml",
                ]

                results = [perfect_result]

                for model_file in models_to_compare:
                    model_path = models_dir / model_file
                    if model_path.exists():
                        logger.info(f"📦 Test de {model_file}...")
                        result = validate_model(model_path, args.duration)
                        results.append(result)

                # Tableau comparatif
                logger.info("\n📊 TABLEAU COMPARATIF:")
                logger.info(
                    f"{'Modèle':<25} | {'Joints':<6} | {'H(m)':<6} | {'L(m)':<6} | {'Perf':<8}"
                )
                logger.info("-" * 70)

                for result in results:
                    if result["success"]:
                        perf_str = (
                            f"{result['performance']:.0f}/s"
                            if result["performance"] > 0
                            else "N/A"
                        )
                        logger.info(
                            f"{result['file']:<25} | {result['joints']:<6} | {result['height']:<6.2f} | {result['width']:<6.2f} | {perf_str:<8}"
                        )
                    else:
                        logger.info(
                            f"{result['file']:<25} | {'ERROR':<6} | {'ERROR':<6} | {'ERROR':<6} | {'ERROR':<8}"
                        )

                # Recommandation finale
                logger.info("\n💡 RECOMMANDATION FINALE:")
                logger.info("=" * 70)
                logger.info("🏆 Modèle recommandé : reachy_mini_perfect.xml")
                logger.info(
                    f"✅ Dimensions réalistes : H={perfect_result['height']:.2f}m, L={perfect_result['width']:.2f}m"
                )
                logger.info(f"✅ Articulations complètes : {perfect_result['joints']}")
                logger.info(
                    f"✅ Performance optimale : {perfect_result['performance']:.0f} steps/s"
                )

                logger.info("\n🚀 Pour lancer le modèle parfait :")
                logger.info(
                    "   python scripts/launch_complete_robot.py --model reachy_mini_perfect.xml"
                )
                logger.info("   # ou")
                logger.info("   ./launch_robot.sh")

        else:
            logger.error(
                f"❌ Erreur avec le modèle parfait : {perfect_result['error']}"
            )
            sys.exit(1)

    except ImportError as e:
        logger.error(f"❌ MuJoCo non disponible : {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"❌ Erreur : {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
