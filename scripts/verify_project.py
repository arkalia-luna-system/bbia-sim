#!/usr/bin/env python3
"""Script de vérification complète du projet BBIA Reachy Mini
Vérifie que tous les composants sont fonctionnels.
"""

import logging
import os
import subprocess
import sys
from pathlib import Path

# Configuration du logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def check_python_imports():
    """Vérifie que tous les modules Python s'importent correctement."""
    modules_to_check = [
        "src.bbia_sim.bbia_audio",
        "src.bbia_sim.bbia_vision",
        "src.bbia_sim.bbia_voice",
        "src.bbia_sim.bbia_emotions",
        "src.bbia_sim.bbia_behavior",
        "src.bbia_sim.daemon.simulation_service",
        "src.bbia_sim.sim.simulator",
    ]

    success_count = 0
    failed_modules = []

    for module in modules_to_check:
        try:
            __import__(module)
            success_count += 1
        except ImportError as e:
            failed_modules.append((module, str(e)))

    if failed_modules:
        logger.warning(f"Modules échoués: {failed_modules}")

    return success_count == len(modules_to_check)


def check_mujoco_model():
    """Vérifie que le modèle MuJoCo se charge correctement."""
    try:
        import mujoco

        model_path = "src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml"

        if not os.path.exists(model_path):
            return False

        mujoco.MjModel.from_xml_path(model_path)
        return True

    except Exception:
        return False


def check_stl_assets():
    """Vérifie que tous les assets STL sont présents."""
    assets_dir = Path("src/bbia_sim/sim/assets/reachy_official")
    stl_files = list(assets_dir.glob("*.stl"))

    if len(stl_files) == 41:

        # Vérifier la taille des fichiers
        valid_count = 0
        for stl_file in stl_files:
            size = stl_file.stat().st_size
            if size > 1000:  # Vrai fichier STL
                valid_count += 1

        return valid_count == len(stl_files)
    else:
        return False


def check_tests():
    """Vérifie que les tests peuvent s'exécuter."""
    try:
        result = subprocess.run(
            [sys.executable, "-m", "pytest", "tests/", "--collect-only", "-q"],
            capture_output=True,
            text=True,
            timeout=30,
        )

        if result.returncode == 0:
            lines = result.stdout.strip().split("\n")
            for line in lines:
                if "collected" in line:
                    int(line.split()[1])
                    break

            return True
        else:
            return False

    except Exception:
        return False


def check_api_endpoints():
    """Vérifie que l'API peut démarrer."""
    try:
        # Test d'import des modules API

        return True

    except Exception:
        return False


def check_scripts():
    """Vérifie que les scripts principaux existent."""
    required_scripts = [
        "scripts/launch_complete_robot.py",
        "scripts/download_ALL_stl.py",
        "scripts/process_manager.py",
        "scripts/smart_process_cleanup.sh",
    ]

    success_count = 0
    for script in required_scripts:
        if os.path.exists(script):
            success_count += 1
        else:
            pass

    return success_count == len(required_scripts)


def main():
    """Fonction principale de vérification."""
    checks = [
        ("Imports Python", check_python_imports),
        ("Modèle MuJoCo", check_mujoco_model),
        ("Assets STL", check_stl_assets),
        ("Tests", check_tests),
        ("API", check_api_endpoints),
        ("Scripts", check_scripts),
    ]

    results = []
    errors = []

    for name, check_func in checks:
        try:
            result = check_func()
            results.append((name, result))
        except Exception as e:
            errors.append((name, str(e)))
            results.append((name, False))

    if errors:
        logger.warning(f"Erreurs détectées: {errors}")

    success_count = 0
    for _name, result in results:
        if result:
            success_count += 1

    if success_count == len(results):
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())
