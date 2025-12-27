#!/usr/bin/env python3
"""BBIA Doctor - Diagnostic environnement automatique.

Usage:
    python scripts/bbia_doctor.py
"""

import os
import sys
from pathlib import Path
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
        "zenoh",  # Pour communication robot
        "reachy_mini",  # SDK officiel Reachy Mini
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


def check_zenoh() -> dict[str, Any]:
    """Vérifie connexion Zenoh locale."""
    results: dict[str, Any] = {}
    try:
        import zenoh  # type: ignore[import-untyped]

        results["zenoh_imported"] = True
        results["zenoh_version"] = getattr(zenoh, "__version__", "inconnue")

        # Tester création session locale
        try:
            from zenoh import Config  # type: ignore[import-untyped]

            config = Config()
            # Mode client pour connexion locale
            config.insert_json5("mode", '"client"')
            # Format Zenoh correct: utiliser tcp/ au lieu de tcp://
            # Note: En mode client sans connect, Zenoh utilise localhost par défaut
            # On peut omettre connect pour test local
            session = zenoh.open(config)
            session.close()
            results["zenoh_session"] = True
        except Exception as e:
            results["zenoh_session"] = False
            results["zenoh_error"] = str(e)
    except ImportError:
        results["zenoh_imported"] = False
        results["zenoh_session"] = False

    return results


def check_daemon() -> dict[str, Any]:
    """Vérifie daemon reachy-mini-daemon."""
    results: dict[str, Any] = {}
    import subprocess

    try:
        # Vérifier si la commande existe
        result = subprocess.run(
            ["which", "reachy-mini-daemon"], capture_output=True, text=True, timeout=2
        )
        results["daemon_installed"] = result.returncode == 0
        if result.returncode == 0:
            results["daemon_path"] = result.stdout.strip()
        else:
            results["daemon_path"] = "non trouvé"
    except Exception as e:
        results["daemon_installed"] = False
        results["daemon_error"] = str(e)

    return results


def check_network_preparation() -> dict[str, Any]:
    """Vérifie préparation réseau (WiFi)."""
    results: dict[str, Any] = {}
    import socket

    try:
        # Récupérer IP locale
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        results["local_ip"] = ip
        results["network_ok"] = True
    except Exception as e:
        results["local_ip"] = "inconnue"
        results["network_ok"] = False
        results["network_error"] = str(e)

    # Vérifier ports (si daemon tourne)
    ports_to_check = {8000: "API Daemon", 7447: "Zenoh"}
    results["ports"] = {}
    for port, name in ports_to_check.items():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.5)
            result = sock.connect_ex(("localhost", port))
            sock.close()
            results["ports"][port] = {"name": name, "open": result == 0}
        except Exception:
            results["ports"][port] = {"name": name, "open": False}

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

    # Zenoh (préparation robot)
    report_lines.append("🔌 ZENOH (Communication Robot):")
    zenoh_check = check_zenoh()
    if zenoh_check.get("zenoh_imported"):
        report_lines.append(
            f"  ✅ Zenoh installé: {zenoh_check.get('zenoh_version', 'inconnue')}"
        )
        if zenoh_check.get("zenoh_session"):
            report_lines.append("  ✅ Session Zenoh locale OK")
        else:
            report_lines.append(
                f"  ⚠️  Session Zenoh: {zenoh_check.get('zenoh_error', 'erreur')}"
            )
    else:
        report_lines.append("  ❌ Zenoh non installé (pip install eclipse-zenoh)")
    report_lines.append("")

    # Daemon (préparation robot)
    report_lines.append("🟣 DAEMON (Reachy Mini):")
    daemon_check = check_daemon()
    if daemon_check.get("daemon_installed"):
        report_lines.append(
            f"  ✅ Daemon installé: {daemon_check.get('daemon_path', 'inconnu')}"
        )
    else:
        report_lines.append("  ⚠️  Daemon non trouvé (pip install reachy-mini)")
    report_lines.append("")

    # Réseau (préparation WiFi)
    report_lines.append("📡 RÉSEAU (Préparation WiFi):")
    network_check = check_network_preparation()
    if network_check.get("network_ok"):
        report_lines.append(
            f"  ✅ IP locale: {network_check.get('local_ip', 'inconnue')}"
        )
    else:
        report_lines.append("  ⚠️  IP locale non déterminable")

    report_lines.append("  Ports:")
    for port, info in network_check.get("ports", {}).items():
        status = "✅" if info.get("open") else "⚠️ "
        report_lines.append(
            f"    {status} Port {port} ({info.get('name')}): {'OUVERT' if info.get('open') else 'FERMÉ (normal si daemon non lancé)'}"
        )
    report_lines.append("")

    # Recommandations
    report_lines.append("💡 RECOMMANDATIONS:")
    missing_deps = [dep for dep, available in deps.items() if not available]
    if missing_deps:
        report_lines.append(f"  ⚠️  Dépendances manquantes: {', '.join(missing_deps)}")
        report_lines.append(
            f"     Installer avec: pip install {' '.join(missing_deps)}",
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
