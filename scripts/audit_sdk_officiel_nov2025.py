#!/usr/bin/env python3
"""
Audit exhaustif vs SDK officiel Reachy Mini (Novembre 2025)
Basé sur https://github.com/pollen-robotics/reachy_mini
"""

import inspect
import subprocess
import sys
from pathlib import Path
from typing import Any

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# Tentative import SDK officiel
try:
    from reachy_mini import ReachyMini
    from reachy_mini.utils import create_head_pose

    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    ReachyMini = None
    create_head_pose = None

ISSUES = []
WARNINGS = []
SUCCESSES = []


def check_sdk_installation() -> dict[str, Any]:
    """Vérifier installation SDK officiel."""
    result = {"installed": False, "version": None, "path": None}

    try:
        result_pip = subprocess.run(
            ["pip", "show", "reachy-mini"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result_pip.returncode == 0:
            result["installed"] = True
            for line in result_pip.stdout.split("\n"):
                if line.startswith("Version:"):
                    result["version"] = line.split(":", 1)[1].strip()
                if line.startswith("Location:"):
                    result["path"] = line.split(":", 1)[1].strip()
    except Exception:
        pass

    return result


def check_daemon_command() -> bool:
    """Vérifier que reachy-mini-daemon est disponible."""
    try:
        result = subprocess.run(
            ["which", "reachy-mini-daemon"],
            capture_output=True,
            text=True,
            timeout=2,
        )
        return result.returncode == 0
    except Exception:
        return False


def check_sdk_methods() -> dict[str, list[str]]:
    """Vérifier toutes les méthodes SDK officiel."""
    if not SDK_AVAILABLE:
        return {"missing": [], "extra": [], "signature_mismatch": []}

    issues = {"missing": [], "extra": [], "signature_mismatch": []}

    # Méthodes officielles selon README
    official_methods = {
        "wake_up",
        "goto_sleep",
        "look_at_world",
        "look_at_image",
        "goto_target",
        "set_target",
        "get_current_joint_positions",
        "set_target_head_pose",
        "set_target_body_yaw",
        "get_current_head_pose",
        "get_present_antenna_joint_positions",
        "enable_motors",
        "disable_motors",
        "enable_gravity_compensation",
        "disable_gravity_compensation",
    }

    if ReachyMini:
        official_attrs = set(dir(ReachyMini))
        for method in official_methods:
            if method not in official_attrs:
                issues["missing"].append(method)

    return issues


def check_our_implementation() -> dict[str, Any]:
    """Vérifier notre implémentation."""
    from bbia_sim.backends.reachy_mini_backend import ReachyMiniBackend

    issues = {
        "missing_methods": [],
        "signature_issues": [],
        "return_type_issues": [],
    }

    # Vérifier méthodes obligatoires
    required_methods = [
        "wake_up",
        "goto_sleep",
        "look_at_world",
        "look_at_image",
        "goto_target",
        "get_current_joint_positions",
        "set_target_head_pose",
        "set_target_body_yaw",
        "enable_motors",
        "disable_motors",
    ]

    backend_class = ReachyMiniBackend
    backend_attrs = set(dir(backend_class))

    for method in required_methods:
        if method not in backend_attrs:
            issues["missing_methods"].append(method)

    return issues


def check_api_endpoints() -> dict[str, Any]:
    """Vérifier endpoints API REST."""
    from bbia_sim.daemon.app.main import app

    issues = {"missing": [], "extra": []}

    # Endpoints officiels selon README
    official_endpoints = [
        "/api/state/full",
        "/docs",  # OpenAPI
        "/",  # Dashboard
    ]

    routes = [route.path for route in app.routes]
    for endpoint in official_endpoints:
        # Vérifier si endpoint existe (exact ou avec variantes)
        found = False
        for route in routes:
            if endpoint in route or route == endpoint:
                found = True
                break
        if not found:
            issues["missing"].append(endpoint)

    return issues


def check_create_head_pose() -> dict[str, Any]:
    """Vérifier create_head_pose."""
    issues = {"available": False, "signature_ok": False}

    if create_head_pose:
        issues["available"] = True
        sig = inspect.signature(create_head_pose)
        params = list(sig.parameters.keys())

        # Selon README: create_head_pose(z=10, roll=15, degrees=True, mm=True)
        expected_params = {"z", "roll", "degrees", "mm", "pitch", "yaw"}
        if set(params).intersection(expected_params):
            issues["signature_ok"] = True

    return issues


def check_python_versions() -> dict[str, Any]:
    """Vérifier versions Python supportées."""
    # Officiel: Python 3.10 à 3.13
    python_version = sys.version_info
    issues = {
        "version": f"{python_version.major}.{python_version.minor}",
        "supported": False,
    }

    if 3.10 <= python_version.minor <= 3.13:
        issues["supported"] = True

    return issues


def main():
    """Fonction principale."""
    print("🔍 AUDIT EXHAUSTIF SDK OFFICIEL REACHY MINI\n")
    print("=" * 70)

    # 1. Installation SDK
    print("\n1️⃣ Vérification Installation SDK")
    print("-" * 70)
    sdk_info = check_sdk_installation()
    if sdk_info["installed"]:
        SUCCESSES.append(f"✅ SDK installé: {sdk_info['version']}")
        print(f"✅ SDK installé: version {sdk_info['version']}")
    else:
        ISSUES.append("❌ SDK reachy-mini non installé")
        print("❌ SDK reachy-mini non installé")
        print("   💡 Installez avec: pip install reachy-mini")

    # 2. Commande daemon
    print("\n2️⃣ Vérification Commande Daemon")
    print("-" * 70)
    if check_daemon_command():
        SUCCESSES.append("✅ Commande reachy-mini-daemon disponible")
        print("✅ Commande reachy-mini-daemon disponible")
    else:
        WARNINGS.append("⚠️ Commande reachy-mini-daemon non trouvée")
        print("⚠️ Commande reachy-mini-daemon non trouvée")

    # 3. Méthodes SDK
    print("\n3️⃣ Vérification Méthodes SDK Officiel")
    print("-" * 70)
    if SDK_AVAILABLE:
        sdk_methods = check_sdk_methods()
        if sdk_methods["missing"]:
            ISSUES.append(f"❌ Méthodes SDK manquantes: {sdk_methods['missing']}")
            print(f"❌ Méthodes manquantes: {sdk_methods['missing']}")
        else:
            SUCCESSES.append("✅ Toutes méthodes SDK disponibles")
            print("✅ Toutes méthodes SDK disponibles")
    else:
        WARNINGS.append("⚠️ SDK non importable - tests limités")
        print("⚠️ SDK non importable")

    # 4. Notre implémentation
    print("\n4️⃣ Vérification Notre Implémentation")
    print("-" * 70)
    our_impl = check_our_implementation()
    if our_impl["missing_methods"]:
        ISSUES.append(
            f"❌ Méthodes manquantes dans ReachyMiniBackend: {our_impl['missing_methods']}"
        )
        print(f"❌ Méthodes manquantes: {our_impl['missing_methods']}")
    else:
        SUCCESSES.append("✅ Toutes méthodes requises implémentées")
        print("✅ Toutes méthodes requises implémentées")

    # 5. API Endpoints
    print("\n5️⃣ Vérification Endpoints API REST")
    print("-" * 70)
    api_check = check_api_endpoints()
    if api_check["missing"]:
        ISSUES.append(f"❌ Endpoints manquants: {api_check['missing']}")
        print(f"❌ Endpoints manquants: {api_check['missing']}")
    else:
        SUCCESSES.append("✅ Endpoints API conformes")
        print("✅ Endpoints API conformes")

    # 6. create_head_pose
    print("\n6️⃣ Vérification create_head_pose")
    print("-" * 70)
    head_pose_check = check_create_head_pose()
    if head_pose_check["available"]:
        if head_pose_check["signature_ok"]:
            SUCCESSES.append("✅ create_head_pose disponible et signature OK")
            print("✅ create_head_pose disponible et signature OK")
        else:
            WARNINGS.append("⚠️ create_head_pose signature peut différer")
            print("⚠️ Signature create_head_pose à vérifier")
    else:
        ISSUES.append("❌ create_head_pose non disponible")
        print("❌ create_head_pose non disponible")

    # 7. Versions Python
    print("\n7️⃣ Vérification Versions Python")
    print("-" * 70)
    python_check = check_python_versions()
    if python_check["supported"]:
        SUCCESSES.append(
            f"✅ Python {python_check['version']} supporté (officiel: 3.10-3.13)"
        )
        print(f"✅ Python {python_check['version']} supporté")
    else:
        WARNINGS.append(
            f"⚠️ Python {python_check['version']} - officiel supporte 3.10-3.13"
        )
        print(f"⚠️ Python {python_check['version']} (officiel: 3.10-3.13)")

    # Résumé
    print("\n" + "=" * 70)
    print("\n📊 RÉSUMÉ")
    print("=" * 70)
    print(f"\n✅ Succès: {len(SUCCESSES)}")
    print(f"⚠️ Avertissements: {len(WARNINGS)}")
    print(f"❌ Problèmes: {len(ISSUES)}")

    if ISSUES:
        print("\n❌ PROBLÈMES À CORRIGER:")
        for issue in ISSUES:
            print(f"   {issue}")

    if WARNINGS:
        print("\n⚠️ AVERTISSEMENTS:")
        for warning in WARNINGS:
            print(f"   {warning}")

    print("\n✅ POINTS CONFORMES:")
    for success in SUCCESSES:
        print(f"   {success}")

    return len(ISSUES) == 0


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
