#!/usr/bin/env python3
"""
Script de comparaison des modèles Reachy Mini
Compare les différents modèles pour voir les améliorations
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
    parser = argparse.ArgumentParser(description="Comparateur de modèles Reachy Mini")
    parser.add_argument(
        "--model1", default="reachy_mini.xml", help="Premier modèle à comparer"
    )
    parser.add_argument(
        "--model2",
        default="reachy_mini_assembled.xml",
        help="Deuxième modèle à comparer",
    )
    parser.add_argument("--headless", action="store_true", help="Mode headless")
    parser.add_argument("--duration", type=int, default=3, help="Durée en secondes")

    args = parser.parse_args()

    # Chemin vers les modèles
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    models_dir = project_root / "src" / "bbia_sim" / "sim" / "models"

    model1_path = models_dir / args.model1
    model2_path = models_dir / args.model2

    if not model1_path.exists():
        logger.error(f"❌ Modèle 1 non trouvé : {model1_path}")
        sys.exit(1)

    if not model2_path.exists():
        logger.error(f"❌ Modèle 2 non trouvé : {model2_path}")
        sys.exit(1)

    logger.info("🔍 COMPARAISON DES MODÈLES REACHY MINI")
    logger.info("=" * 50)
    logger.info(f"📁 Modèle 1 : {args.model1}")
    logger.info(f"📁 Modèle 2 : {args.model2}")

    try:
        import mujoco

        # Chargement des modèles
        logger.info("📦 Chargement des modèles...")
        model1 = mujoco.MjModel.from_xml_path(str(model1_path))
        model2 = mujoco.MjModel.from_xml_path(str(model2_path))

        data1 = mujoco.MjData(model1)
        data2 = mujoco.MjData(model2)

        logger.info("✅ Modèles chargés avec succès")

        # Comparaison des articulations
        logger.info("\n🤖 COMPARAISON DES ARTICULATIONS:")
        logger.info("-" * 40)

        joints1 = [model1.joint(i).name for i in range(model1.njnt)]
        joints2 = [model2.joint(i).name for i in range(model2.njnt)]

        logger.info(f"Modèle 1 ({args.model1}): {len(joints1)} articulations")
        logger.info(f"Modèle 2 ({args.model2}): {len(joints2)} articulations")

        if set(joints1) == set(joints2):
            logger.info("✅ Mêmes articulations dans les deux modèles")
        else:
            logger.info("❌ Articulations différentes")
            only_in_1 = set(joints1) - set(joints2)
            only_in_2 = set(joints2) - set(joints1)
            if only_in_1:
                logger.info(f"   Seulement dans {args.model1}: {only_in_1}")
            if only_in_2:
                logger.info(f"   Seulement dans {args.model2}: {only_in_2}")

        # Comparaison des géométries
        logger.info("\n🎨 COMPARAISON DES GÉOMÉTRIES:")
        logger.info("-" * 40)

        geoms1 = [model1.geom(i).name for i in range(model1.ngeom)]
        geoms2 = [model2.geom(i).name for i in range(model2.ngeom)]

        logger.info(f"Modèle 1 ({args.model1}): {len(geoms1)} géométries")
        logger.info(f"Modèle 2 ({args.model2}): {len(geoms2)} géométries")

        # Comparaison des corps
        logger.info("\n🏗️ COMPARAISON DES CORPS:")
        logger.info("-" * 40)

        bodies1 = [model1.body(i).name for i in range(model1.nbody)]
        bodies2 = [model2.body(i).name for i in range(model2.nbody)]

        logger.info(f"Modèle 1 ({args.model1}): {len(bodies1)} corps")
        logger.info(f"Modèle 2 ({args.model2}): {len(bodies2)} corps")

        # Test de simulation
        if args.headless:
            logger.info("\n🔄 TEST DE SIMULATION:")
            logger.info("-" * 40)

            import time

            # Test modèle 1
            logger.info(f"🧪 Test {args.model1}...")
            start_time = time.monotonic()
            step_count = 0
            while time.monotonic() - start_time < args.duration:
                mujoco.mj_step(model1, data1)
                step_count += 1
            final_time = time.monotonic() - start_time
            logger.info(f"   ✅ {step_count} steps en {final_time:.2f}s")

            # Test modèle 2
            logger.info(f"🧪 Test {args.model2}...")
            start_time = time.monotonic()
            step_count = 0
            while time.monotonic() - start_time < args.duration:
                mujoco.mj_step(model2, data2)
                step_count += 1
            final_time = time.monotonic() - start_time
            logger.info(f"   ✅ {step_count} steps en {final_time:.2f}s")

        logger.info("\n🎯 RÉSUMÉ:")
        logger.info("=" * 50)
        logger.info(
            f"✅ Modèle {args.model1}: {'Pièces détachées' if 'complete' not in args.model1 else 'Assemblage basique'}"
        )
        logger.info(
            f"✅ Modèle {args.model2}: {'Assemblage corrigé' if 'assembled' in args.model2 else 'Assemblage basique'}"
        )
        logger.info("💡 Utilisez le modèle 'assembled' pour voir le robot complet !")

    except ImportError as e:
        logger.error(f"❌ MuJoCo non disponible : {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"❌ Erreur : {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
