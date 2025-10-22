"""Interface en ligne de commande pour BBIA-SIM."""

import argparse
import importlib.util
import logging
import sys
from pathlib import Path

from bbia_sim.sim.simulator import MuJoCoSimulator

logger = logging.getLogger(__name__)


def setup_logging(verbose: bool = False) -> None:
    """Configure le logging.

    Args:
        verbose: Si True, active le mode verbose
    """
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )


def main() -> None:
    """Point d'entrée principal du CLI."""
    parser = argparse.ArgumentParser(
        description="BBIA-SIM - Moteur cognitif Python pour robot Reachy Mini Wireless",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation:
  python -m bbia_sim --sim                    # Lance la simulation par défaut
  python -m bbia_sim --sim --scene simple     # Lance avec la scène simple
  python -m bbia_sim --sim --headless         # Mode headless
  python -m bbia_sim --awake                  # Séquence de réveil
  python -m bbia_sim --voice "Bonjour"        # Synthèse vocale
        """,
    )

    # Options principales
    parser.add_argument("--sim", action="store_true", help="Lance la simulation MuJoCo")
    parser.add_argument(
        "--scene",
        type=str,
        default="reachy_mini.xml",
        help="Scène de simulation à charger (reachy_mini.xml, minimal.xml)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Mode headless (pas de fenêtre graphique)",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=None,
        help="Durée de simulation en secondes (mode headless uniquement)",
    )

    # Options BBIA
    parser.add_argument(
        "--awake", action="store_true", help="Lance la séquence de réveil BBIA"
    )
    parser.add_argument("--voice", type=str, help="Texte à synthétiser vocalement")
    parser.add_argument(
        "--listen", action="store_true", help="Active la reconnaissance vocale"
    )

    # Options techniques
    parser.add_argument("--verbose", "-v", action="store_true", help="Mode verbose")
    parser.add_argument("--version", action="version", version="BBIA-SIM 1.0.0")

    args = parser.parse_args()

    # Configuration du logging
    setup_logging(args.verbose)

    try:
        if args.sim:
            run_simulation(args)
        elif args.awake:
            run_awake_sequence()
        elif args.voice:
            run_voice_synthesis(args.voice)
        elif args.listen:
            run_voice_recognition()
        else:
            # Mode par défaut : affichage de l'aide
            parser.print_help()

    except KeyboardInterrupt:
        logger.info("Arrêt demandé par l'utilisateur")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Erreur : {e}")
        sys.exit(1)


def run_simulation(args: argparse.Namespace) -> None:
    """Lance la simulation MuJoCo.

    Args:
        args: Arguments de la ligne de commande
    """
    logger.info("🚀 Démarrage de la simulation MuJoCo")

    # Vérification multi-OS pour le viewer
    if not args.headless:
        if sys.platform == "darwin":
            # Sur macOS, vérifier si mujoco.viewer est disponible
            if importlib.util.find_spec("mujoco.viewer") is None:
                logger.error(
                    "❌ Module mujoco.viewer non disponible sur macOS.\n"
                    "💡 Solutions :\n"
                    "  • Utilisez : mjpython -m bbia_sim --sim --verbose\n"
                    "  • Ou installez : pip install mujoco-python-viewer\n"
                    "  • Ou utilisez : python -m bbia_sim --sim --headless"
                )
                sys.exit(2)
            else:
                logger.info("✅ Viewer MuJoCo disponible sur macOS")
        else:
            # Linux/Windows : vérifier la disponibilité du viewer
            if importlib.util.find_spec("mujoco.viewer") is None:
                logger.warning(
                    "⚠️ Module mujoco.viewer non disponible.\n"
                    "💡 Installez : pip install mujoco-python-viewer\n"
                    "   Ou utilisez : python -m bbia_sim --sim --headless"
                )
            else:
                logger.info("✅ Viewer MuJoCo disponible")

    # Détermination du modèle à utiliser
    if args.scene == "reachy_mini.xml":
        model_path = Path(__file__).parent / "sim" / "models" / "reachy_mini.xml"
    elif args.scene == "minimal.xml":
        model_path = Path(__file__).parent / "sim" / "scenes" / "minimal.xml"
    else:
        # Essayer de charger directement le fichier spécifié
        model_path = Path(args.scene)
        if not model_path.is_absolute():
            model_path = Path(__file__).parent / "sim" / "models" / args.scene

    # Initialisation du simulateur
    try:
        simulator = MuJoCoSimulator(str(model_path))
        logger.info(f"Modèle chargé : {model_path}")

        # Affichage des articulations disponibles
        joints = simulator.get_available_joints()
        logger.info(f"Articulations disponibles : {joints}")

        # Lancement de la simulation
        simulator.launch_simulation(headless=args.headless, duration=args.duration)

    except FileNotFoundError as e:
        logger.error(f"Fichier non trouvé : {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Erreur lors du lancement de la simulation : {e}")
        sys.exit(1)


def run_awake_sequence() -> None:
    """Lance la séquence de réveil BBIA."""
    logger.info("✨ Démarrage de la séquence de réveil BBIA")

    try:
        from bbia_sim.bbia_awake import start_bbia_sim

        start_bbia_sim()
    except ImportError as e:
        logger.error(f"Impossible d'importer le module de réveil : {e}")
        sys.exit(1)


def run_voice_synthesis(text: str) -> None:
    """Lance la synthèse vocale.

    Args:
        text: Texte à synthétiser
    """
    logger.info(f"🗣️ Synthèse vocale : {text}")

    try:
        from bbia_sim.bbia_voice import dire_texte

        dire_texte(text)
    except ImportError as e:
        logger.error(f"Impossible d'importer le module vocal : {e}")
        sys.exit(1)


def run_voice_recognition() -> None:
    """Lance la reconnaissance vocale."""
    logger.info("👂 Activation de la reconnaissance vocale")

    try:
        from bbia_sim.bbia_voice import reconnaitre_parole

        text = reconnaitre_parole(duree=5)
        logger.info(f"Texte reconnu : {text}")
    except ImportError as e:
        logger.error(f"Impossible d'importer le module vocal : {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
