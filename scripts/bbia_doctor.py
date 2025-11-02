#!/usr/bin/env python3
"""BBIA Doctor - Diagnostic environnement automatique.

Usage:
    python scripts/bbia_doctor.py
"""

import os
from pathlib import Path
import sys
from typing import Any

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def check_dependencies() -> dict[str, bool]:
    """Vérifie dépendances installées."""
    results: dict[str, bool] = {}
    dependencies = [
        "numpy",
        "opencv-python",
        "transformers",
        "sentence_transformers",
        "openai-whisper",
        "mujoco",
        "fastapi",
        "mediapipe",
        "ultralytics",
    ]

    for dep in dependencies:
        try:
            __import__(dep.replace("-", "_"))
            results[dep] = True
        except ImportError:
            results[dep] = False

    return results


def check_models_available() -> dict[str, bool]:
    """Vérifie modèles disponibles."""
    results: dict[str, bool] = {}
    try:
        from bbia_sim.bbia_huggingface import BBIAHuggingFace

        hf = BBIAHuggingFace()
        results["sentence-transformers"] = hf._sentence_model is not None
    except Exception:
        results["sentence-transformers"] = False

    return results


def check_environment() -> dict[str, Any]:
    """Vérifie variables d'environnement."""
    results: dict[str, Any] = {}
    env_vars = [
        "BBIA_DISABLE_AUDIO",
        "WHISPER_MODEL_SIZE",
        "PYTEST_TMPDIR",
    ]

    for var in env_vars:
        results[var] = os.environ.get(var, "non défini")

    return results


def check_configuration() -> dict[str, Any]:
    """Vérifie configuration projet."""
    results: dict[str, Any] = {}
    project_root = Path(__file__).parent.parent

    results["project_root_exists"] = project_root.exists()
    results["src_exists"] = (project_root / "src" / "bbia_sim").exists()
    results["tests_exists"] = (project_root / "tests").exists()

    return results


def generate_report() -> str:
    """Génère rapport diagnostic complet."""
    report_lines: list[str] = []
    report_lines.append("=" * 60)
    report_lines.append("BBIA DOCTOR - Diagnostic Environnement")
    report_lines.append("=" * 60)
    report_lines.append("")

    # Dépendances
    report_lines.append("📦 DÉPENDANCES:")
    deps = check_dependencies()
    for dep, available in deps.items():
        status = "✅" if available else "❌"
        report_lines.append(f"  {status} {dep}")
    report_lines.append("")

    # Modèles
    report_lines.append("🧠 MODÈLES:")
    models = check_models_available()
    for model, available in models.items():
        status = "✅" if available else "❌"
        report_lines.append(f"  {status} {model}")
    report_lines.append("")

    # Environnement
    report_lines.append("🌍 VARIABLES D'ENVIRONNEMENT:")
    env = check_environment()
    for var, value in env.items():
        report_lines.append(f"  {var} = {value}")
    report_lines.append("")

    # Configuration
    report_lines.append("⚙️  CONFIGURATION:")
    config = check_configuration()
    for key, value in config.items():
        status = "✅" if value else "❌"
        report_lines.append(f"  {status} {key}")
    report_lines.append("")

    # Recommandations
    report_lines.append("💡 RECOMMANDATIONS:")
    missing_deps = [dep for dep, available in deps.items() if not available]
    if missing_deps:
        report_lines.append(f"  ⚠️  Dépendances manquantes: {', '.join(missing_deps)}")
        report_lines.append(
            f"     Installer avec: pip install {' '.join(missing_deps)}"
        )
    else:
        report_lines.append("  ✅ Toutes les dépendances sont installées")

    report_lines.append("")
    report_lines.append("=" * 60)

    return "\n".join(report_lines)


def main() -> int:
    """Point d'entrée principal."""
    try:
        report = generate_report()
        print(report)

        # Code retour selon problèmes
        deps = check_dependencies()
        missing = [dep for dep, available in deps.items() if not available]
        return 1 if missing else 0

    except Exception as e:
        print(f"❌ Erreur diagnostic: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
