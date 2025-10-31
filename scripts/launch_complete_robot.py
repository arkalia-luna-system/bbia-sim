#!/usr/bin/env python3
"""Lanceur du robot Reachy Mini complet en 3D
Utilise le modèle MJCF corrigé avec assemblage correct des pièces.
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
    parser = argparse.ArgumentParser(description="Lanceur BBIA Reachy Mini complet")
    parser.add_argument(
        "--model", default="reachy_mini_assembled.xml", help="Modèle MJCF à utiliser"
    )
    parser.add_argument(
        "--headless", action="store_true", help="Mode headless (pas de fenêtre 3D)"
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=None,
        help="Durée en secondes (headless uniquement)",
    )
    parser.add_argument("--verbose", action="store_true", help="Mode verbeux")

    args = parser.parse_args()

    # Chemin vers le modèle (depuis le dossier scripts)
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    model_path = project_root / "src" / "bbia_sim" / "sim" / "models" / args.model

    if not model_path.exists():
        logger.error(f"❌ Modèle non trouvé : {model_path}")
        sys.exit(1)

    logger.info("🚀 Démarrage simulation BBIA Reachy Mini complet")
    logger.info(f"📁 Modèle : {model_path}")

    try:
        import mujoco
        import mujoco.viewer

        # Chargement du modèle
        logger.info("📦 Chargement du modèle MuJoCo...")
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)

        logger.info("✅ Modèle chargé avec succès")
        logger.info(
            f"🤖 Articulations disponibles : {[model.joint(i).name for i in range(model.njnt)]}"
        )

        if args.headless:
            logger.info("🔄 Mode headless activé")
            if args.duration:
                logger.info(f"⏱️ Durée : {args.duration}s")
                _run_headless_simulation(model, data, args.duration)
            else:
                logger.info("🔄 Simulation headless continue (Ctrl+C pour arrêter)")
                _run_headless_continuous(model, data)
        else:
            logger.info("🎮 Mode graphique activé")
            logger.info(
                "💡 Sur macOS, utilisez 'mjpython' au lieu de 'python' pour la fenêtre 3D"
            )
            _run_graphical_simulation(model, data)

    except ImportError as e:
        logger.error(f"❌ MuJoCo non disponible : {e}")
        logger.error("💡 Installez MuJoCo : pip install mujoco")
        sys.exit(1)
    except Exception as e:
        logger.error(f"❌ Erreur : {e}")
        sys.exit(1)


def _run_headless_simulation(model, data, duration):
    """Exécute une simulation headless avec durée fixe."""
    import time

    import mujoco

    start_time = time.monotonic()
    step_count = 0

    logger.info("🔄 Simulation headless démarrée")

    while time.monotonic() - start_time < duration:
        mujoco.mj_step(model, data)
        step_count += 1

        if step_count % 10000 == 0:
            elapsed = time.monotonic() - start_time
            logger.info(f"Step {step_count} - Temps écoulé: {elapsed:.2f}s")

    final_time = time.monotonic() - start_time
    logger.info(
        f"✅ Simulation headless terminée après {step_count} steps ({final_time:.2f}s)"
    )


def _run_headless_continuous(model, data):
    """Exécute une simulation headless continue."""
    import signal
    import time

    import mujoco

    step_count = 0
    start_time = time.monotonic()

    def signal_handler(sig, frame):
        logger.info("\n🛑 Arrêt demandé par l'utilisateur")
        final_time = time.monotonic() - start_time
        logger.info(
            f"✅ Simulation terminée après {step_count} steps ({final_time:.2f}s)"
        )
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    logger.info("🔄 Simulation headless continue démarrée")
    logger.info("💡 Appuyez sur Ctrl+C pour arrêter")

    try:
        while True:
            mujoco.mj_step(model, data)
            step_count += 1

            if step_count % 10000 == 0:
                elapsed = time.monotonic() - start_time
                logger.info(f"Step {step_count} - Temps écoulé: {elapsed:.2f}s")

    except KeyboardInterrupt:
        signal_handler(None, None)


def _run_graphical_simulation(model, data):
    """Exécute une simulation graphique avec fenêtre 3D."""
    import time

    import mujoco

    try:
        logger.info("🎮 Lancement de la fenêtre 3D MuJoCo...")
        logger.info("💡 Contrôles :")
        logger.info("   • Souris : Rotation de la vue")
        logger.info("   • Molette : Zoom")
        logger.info("   • Clic droit : Déplacer la vue")
        logger.info("   • Échap : Fermer la fenêtre")

        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Configurer la caméra à 180° (face optimal) immédiatement
            viewer.cam.azimuth = 180.0
            viewer.cam.elevation = -15.0
            viewer.cam.distance = 1.2  # Rapproché de 20%
            viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
            viewer.sync()

            logger.info("✅ Fenêtre 3D ouverte")
            logger.info("🎯 Vous devriez voir le robot Reachy Mini complet !")

            # Boucle de simulation
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()

                # Petit délai pour éviter de surcharger le CPU
                time.sleep(0.01)

    except Exception as e:
        logger.error(f"❌ Erreur fenêtre 3D : {e}")
        logger.error("💡 Solutions :")
        logger.error("   • Sur macOS : utilisez 'mjpython' au lieu de 'python'")
        logger.error("   • Ou utilisez : python -m bbia_sim --sim --headless")
        logger.error("   • Ou installez : pip install mujoco-python-viewer")
        raise


if __name__ == "__main__":
    main()
