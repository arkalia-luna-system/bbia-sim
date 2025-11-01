#!/usr/bin/env python3
"""
Script pour visualiser une scène MuJoCo personnalisée.

Usage:
    python examples/view_scene_piece.py [chemin_vers_scene.xml]
    # Ou avec mjpython sur macOS:
    mjpython examples/view_scene_piece.py [chemin_vers_scene.xml]
"""

import logging
import sys
import time
from pathlib import Path

# Ajouter le répertoire racine au path
root_dir = Path(__file__).parent.parent
sys.path.insert(0, str(root_dir))

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("❌ Erreur : MuJoCo non disponible")
    print("💡 Installez : pip install mujoco")
    if sys.platform == "darwin":
        print("💡 Sur macOS, utilisez 'mjpython' au lieu de 'python'")
    sys.exit(1)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Charge et affiche une scène MuJoCo."""

    # Chemin vers la scène (argument ou défaut)
    if len(sys.argv) > 1:
        scene_path = Path(sys.argv[1])
        if not scene_path.is_absolute():
            scene_path = root_dir / scene_path
    else:
        # Scène par défaut : minimal.xml
        scene_path = root_dir / "src" / "bbia_sim" / "sim" / "scenes" / "minimal.xml"

    if not scene_path.exists():
        logger.error(f"❌ Scène non trouvée : {scene_path}")
        logger.info(
            "💡 Créez votre scène XML ou utilisez : python examples/view_scene_piece.py chemin/vers/scene.xml"
        )
        sys.exit(1)

    logger.info(f"📁 Chargement de la scène : {scene_path}")

    try:
        # Charger le modèle
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)

        logger.info("✅ Scène chargée avec succès")
        logger.info(f"🤖 Nombre d'objets géométriques : {model.ngeom}")

        logger.info("\n🎮 Lancement du viewer MuJoCo...")
        logger.info("💡 Contrôles :")
        logger.info("   • Souris : Rotation de la vue")
        logger.info("   • Molette : Zoom")
        logger.info("   • Clic droit : Déplacer la vue")
        logger.info("   • Échap : Fermer la fenêtre")

        # Lancer le viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Configurer la caméra (ajustez selon votre scène)
            viewer.cam.azimuth = 180.0
            viewer.cam.elevation = -15.0
            viewer.cam.distance = 2.5
            viewer.cam.lookat[:] = [0.0, 0.0, 0.3]

            # Synchroniser immédiatement
            viewer.sync()
            time.sleep(0.2)

            logger.info("✅ Fenêtre 3D ouverte")

            # Boucle de simulation
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.01)

    except mujoco.FatalError as e:
        logger.error(f"❌ Erreur MuJoCo : {e}")
        logger.info("💡 Vérifiez que tous les fichiers référencés existent")
        sys.exit(1)
    except Exception as e:
        logger.error(f"❌ Erreur : {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
