#!/usr/bin/env python3
"""
Script de comparaison complète des modèles Reachy Mini
Compare les différents modèles pour identifier le meilleur assemblage
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
        description="Comparateur complet de modèles Reachy Mini"
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument("--duration", type=int, default=2, help="Durée en secondes")

    args = parser.parse_args()

    # Chemin vers les modèles
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    models_dir = project_root / "src" / "bbia_sim" / "sim" / "models"

    models = [
        ("reachy_mini.xml", "Modèle original (pièces détachées)"),
        ("reachy_mini_complete.xml", "Modèle complet (assemblage basique)"),
        ("reachy_mini_assembled.xml", "Modèle assemblé (positions corrigées)"),
        ("reachy_mini_scaled.xml", "Modèle à l'échelle (dimensions réalistes)"),
    ]

    logger.info("🔍 COMPARAISON COMPLÈTE DES MODÈLES REACHY MINI")
    logger.info("=" * 70)

    try:
        import mujoco

        results = []

        for model_file, description in models:
            model_path = models_dir / model_file

            if not model_path.exists():
                logger.warning(f"⚠️ Modèle non trouvé : {model_file}")
                continue

            logger.info(f"\n📦 Test du modèle : {model_file}")
            logger.info(f"📝 Description : {description}")

            try:
                # Chargement du modèle
                model = mujoco.MjModel.from_xml_path(str(model_path))
                data = mujoco.MjData(model)

                # Informations du modèle
                joints = [model.joint(i).name for i in range(model.njnt)]
                geoms = [model.geom(i).name for i in range(model.ngeom)]
                bodies = [model.body(i).name for i in range(model.nbody)]

                logger.info(
                    f"✅ Modèle chargé : {len(joints)} joints, {len(geoms)} geoms, {len(bodies)} bodies"
                )

                # Test de simulation
                if args.headless:
                    logger.info(f"🔄 Test simulation ({args.duration}s)...")
                    import time

                    start_time = time.monotonic()
                    step_count = 0
                    while time.monotonic() - start_time < args.duration:
                        mujoco.mj_step(model, data)
                        step_count += 1
                    final_time = time.monotonic() - start_time

                    logger.info(f"   ✅ {step_count} steps en {final_time:.2f}s")

                    results.append(
                        {
                            "file": model_file,
                            "description": description,
                            "joints": len(joints),
                            "geoms": len(geoms),
                            "bodies": len(bodies),
                            "steps": step_count,
                            "time": final_time,
                            "performance": step_count / final_time,
                        }
                    )
                else:
                    results.append(
                        {
                            "file": model_file,
                            "description": description,
                            "joints": len(joints),
                            "geoms": len(geoms),
                            "bodies": len(bodies),
                            "steps": 0,
                            "time": 0,
                            "performance": 0,
                        }
                    )

            except Exception as e:
                logger.error(f"❌ Erreur avec {model_file}: {e}")
                results.append(
                    {
                        "file": model_file,
                        "description": description,
                        "joints": 0,
                        "geoms": 0,
                        "bodies": 0,
                        "steps": 0,
                        "time": 0,
                        "performance": 0,
                        "error": str(e),
                    }
                )

        # Résumé des résultats
        logger.info("\n🎯 RÉSUMÉ COMPARATIF:")
        logger.info("=" * 70)
        logger.info(
            f"{'Modèle':<25} | {'Joints':<6} | {'Geoms':<6} | {'Bodies':<6} | {'Performance':<12}"
        )
        logger.info("-" * 70)

        for result in results:
            if "error" in result:
                logger.info(
                    f"{result['file']:<25} | {'ERROR':<6} | {'ERROR':<6} | {'ERROR':<6} | {'ERROR':<12}"
                )
            else:
                perf_str = (
                    f"{result['performance']:.0f} steps/s"
                    if result["performance"] > 0
                    else "N/A"
                )
                logger.info(
                    f"{result['file']:<25} | {result['joints']:<6} | {result['geoms']:<6} | {result['bodies']:<6} | {perf_str:<12}"
                )

        # Recommandation
        logger.info("\n💡 RECOMMANDATION:")
        logger.info("=" * 70)

        if results:
            # Trouver le meilleur modèle (plus de joints, bonne performance)
            best_model = max(
                results, key=lambda x: x["joints"] if "error" not in x else 0
            )

            if "error" not in best_model:
                logger.info(f"🏆 Meilleur modèle : {best_model['file']}")
                logger.info(f"📝 Description : {best_model['description']}")
                logger.info(f"✅ Articulations : {best_model['joints']}")
                logger.info(f"🎨 Géométries : {best_model['geoms']}")
                logger.info(f"🏗️ Corps : {best_model['bodies']}")

                if best_model["performance"] > 0:
                    logger.info(
                        f"⚡ Performance : {best_model['performance']:.0f} steps/s"
                    )

                logger.info("\n🚀 Pour lancer le meilleur modèle :")
                logger.info("   ./launch_robot.sh")
                logger.info("   # ou")
                logger.info(
                    f"   python scripts/launch_complete_robot.py --model {best_model['file']}"
                )
            else:
                logger.error("❌ Aucun modèle fonctionnel trouvé")

    except ImportError as e:
        logger.error(f"❌ MuJoCo non disponible : {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"❌ Erreur : {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
