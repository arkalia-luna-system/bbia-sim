"""Interface en ligne de commande pour BBIA-SIM."""

import argparse
import importlib.util
import logging
import sys
import tempfile
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
        level=level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
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
        "--awake",
        action="store_true",
        help="Lance la séquence de réveil BBIA",
    )
    parser.add_argument("--voice", type=str, help="Texte à synthétiser vocalement")
    parser.add_argument(
        "--listen",
        action="store_true",
        help="Active la reconnaissance vocale",
    )

    # Options techniques
    parser.add_argument("--verbose", "-v", action="store_true", help="Mode verbose")
    parser.add_argument(
        "--doctor",
        action="store_true",
        help="Diagnostic de l'environnement BBIA-SIM",
    )
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
        elif args.doctor:
            run_doctor()
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
                    "  • Ou utilisez : python -m bbia_sim --sim --headless",
                )
                sys.exit(2)
            else:
                logger.info("✅ Viewer MuJoCo disponible sur macOS")
        # Linux/Windows : vérifier la disponibilité du viewer
        elif importlib.util.find_spec("mujoco.viewer") is None:
            logger.warning(
                "⚠️ Module mujoco.viewer non disponible.\n"
                "💡 Installez : pip install mujoco-python-viewer\n"
                "   Ou utilisez : python -m bbia_sim --sim --headless",
            )
        else:
            logger.info("✅ Viewer MuJoCo disponible")

    # Détermination du modèle à utiliser
    if args.scene == "reachy_mini.xml":
        model_path = (
            Path(__file__).parent / "sim" / "models" / "reachy_mini_REAL_OFFICIAL.xml"
        )
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


def run_doctor() -> None:
    """Lance le diagnostic de l'environnement BBIA-SIM."""
    logger.info("🔍 Diagnostic de l'environnement BBIA-SIM...")
    print("\n" + "=" * 60)
    print("🔍 DIAGNOSTIC BBIA-SIM")
    print("=" * 60 + "\n")

    checks = {}
    all_ok = True

    # Python version
    python_version = (
        f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    )
    python_ok = sys.version_info >= (3, 10)
    checks["Python version"] = {
        "status": python_ok,
        "value": python_version,
        "required": ">=3.10",
    }
    if not python_ok:
        all_ok = False

    # Reachy Mini SDK
    try:
        from reachy_mini import ReachyMini  # noqa: F401

        checks["Reachy Mini SDK"] = {"status": True, "value": "disponible"}
    except ImportError:
        checks["Reachy Mini SDK"] = {"status": False, "value": "non disponible"}
        all_ok = False

    # MuJoCo
    try:
        import mujoco  # noqa: F401

        checks["MuJoCo"] = {"status": True, "value": "disponible"}
    except ImportError:
        checks["MuJoCo"] = {"status": False, "value": "non disponible"}
        all_ok = False

    # Audio libraries
    try:
        import sounddevice  # noqa: F401

        checks["SoundDevice"] = {"status": True, "value": "disponible"}
    except ImportError:
        checks["SoundDevice"] = {"status": False, "value": "non disponible"}
        all_ok = False

    # Camera access
    try:
        import cv2  # noqa: F401

        checks["OpenCV"] = {"status": True, "value": "disponible"}
    except ImportError:
        checks["OpenCV"] = {"status": False, "value": "non disponible"}
        all_ok = False

    # Network connectivity
    try:
        import socket

        socket.create_connection(("8.8.8.8", 53), timeout=3)
        checks["Network"] = {"status": True, "value": "connecté"}
    except Exception:
        checks["Network"] = {"status": False, "value": "non connecté"}
        all_ok = False

    # File permissions
    try:
        # Utiliser tempfile.gettempdir() au lieu de /tmp hardcodé (B108)
        temp_dir = tempfile.gettempdir()
        test_file = Path(temp_dir) / "bbia_test_write"
        test_file.write_text("test")
        test_file.unlink()
        checks["File permissions"] = {"status": True, "value": "OK"}
    except Exception as e:
        checks["File permissions"] = {"status": False, "value": f"Erreur: {e}"}
        all_ok = False

    # Afficher résultats
    for check_name, check_info in checks.items():
        status_icon = "✅" if check_info["status"] else "❌"
        print(f"{status_icon} {check_name}: {check_info['value']}")
        if "required" in check_info:
            print(f"   Requis: {check_info['required']}")

    print("\n" + "=" * 60)
    if all_ok:
        print("✅ Tous les checks sont OK !")
    else:
        print("⚠️  Certains checks ont échoué")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
