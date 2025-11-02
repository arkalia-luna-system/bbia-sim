#!/usr/bin/env python3
"""Vérifie et corrige tous les audits et corrections mentionnés dans les MD."""

import json
import shutil
from pathlib import Path
from typing import Any

BBIA_ROOT = Path(__file__).parent.parent
OFFICIAL_ROOT = Path("/Volumes/T7/reachy_mini")

# Fichiers critiques à vérifier
CRITICAL_FILES = {
    "kinematics_data.json": {
        "official": (
            OFFICIAL_ROOT / "src" / "reachy_mini" / "assets" / "kinematics_data.json"
        ),
        "bbia": (
            BBIA_ROOT / "src" / "bbia_sim" / "sim" / "assets" / "kinematics_data.json"
        ),
        "status": "missing",
        "priority": "CRITICAL",
    },
    "constants.py": {
        "official": OFFICIAL_ROOT / "src" / "reachy_mini" / "utils" / "constants.py",
        "bbia": BBIA_ROOT / "src" / "bbia_sim" / "utils" / "constants.py",
        "status": "missing",
        "priority": "HIGH",
    },
}


def check_corrections() -> dict[str, Any]:
    """Vérifie toutes les corrections mentionnées dans les MD."""
    status = {"completed": [], "missing": [], "needs_update": []}

    # 1. Vérifier kinematics_data.json
    if CRITICAL_FILES["kinematics_data.json"]["official"].exists():
        if not CRITICAL_FILES["kinematics_data.json"]["bbia"].exists():
            status["missing"].append(
                {
                    "file": "kinematics_data.json",
                    "priority": "CRITICAL",
                    "action": "copy",
                },
            )
        else:
            status["completed"].append("kinematics_data.json")

    # 2. Vérifier constants.py
    if CRITICAL_FILES["constants.py"]["official"].exists():
        if not CRITICAL_FILES["constants.py"]["bbia"].exists():
            status["missing"].append(
                {
                    "file": "constants.py",
                    "priority": "HIGH",
                    "action": "create_adapted",
                },
            )
        else:
            status["completed"].append("constants.py")

    # 3. Vérifier corrections code
    move_py = BBIA_ROOT / "src" / "bbia_sim" / "daemon" / "app" / "routers" / "move.py"
    backend_adapter = (
        BBIA_ROOT / "src" / "bbia_sim" / "daemon" / "app" / "backend_adapter.py"
    )

    if move_py.exists():
        content = move_py.read_text()
        # Vérifier /goto sans method
        if "goto_target(" in content:
            call_part = (
                content.split("goto_target(")[1].split(")")[0]
                if "goto_target(" in content
                else ""
            )
            if "method=" not in call_part or '"method"' not in call_part:
                status["completed"].append("POST /goto without method param")

    if backend_adapter.exists():
        content = backend_adapter.read_text()
        checks = [
            ("target_head_pose", "target properties"),
            ("goto_joint_positions", "goto_joint_positions method"),
            ("get_urdf", "get_urdf method"),
            ("play_sound", "play_sound method"),
        ]
        for check, name in checks:
            if check in content:
                status["completed"].append(name)

    return status


def apply_missing_corrections(status: dict[str, Any]) -> None:
    """Applique les corrections manquantes."""
    for item in status["missing"]:
        if item["action"] == "copy":
            # Créer répertoire si nécessaire
            CRITICAL_FILES[item["file"]]["bbia"].parent.mkdir(
                parents=True,
                exist_ok=True,
            )
            # Copier fichier
            shutil.copy2(
                CRITICAL_FILES[item["file"]]["official"],
                CRITICAL_FILES[item["file"]]["bbia"],
            )
            print(f"✅ Copié: {item['file']}")

        elif item["action"] == "create_adapted":
            # Créer constants.py adapté pour BBIA
            official_content = CRITICAL_FILES["constants.py"]["official"].read_text()
            # Adapter les chemins pour BBIA
            adapted_content = (
                official_content.replace("import reachy_mini", "import bbia_sim")
                .replace("reachy_mini", "bbia_sim")
                .replace(
                    "descriptions/reachy_mini/urdf",
                    "sim/models",  # URDF dans sim/models
                )
                .replace("assets/models", "sim/assets")  # Assets dans sim/assets
            )

            CRITICAL_FILES["constants.py"]["bbia"].parent.mkdir(
                parents=True,
                exist_ok=True,
            )
            CRITICAL_FILES["constants.py"]["bbia"].write_text(adapted_content)
            print(f"✅ Créé: {item['file']} (adapté pour BBIA)")


def main() -> None:
    """Point d'entrée principal."""
    print("🔍 Vérification corrections et audits...")

    status = check_corrections()

    print(f"\n✅ Complétés: {len(status['completed'])}")
    print(f"❌ Manquants: {len(status['missing'])}")

    if status["missing"]:
        print("\n🔧 Application corrections manquantes...")
        apply_missing_corrections(status)

    # Sauvegarder rapport
    report_file = BBIA_ROOT / "logs" / "verification_corrections.json"
    report_file.parent.mkdir(exist_ok=True)
    with open(report_file, "w") as f:
        json.dump(status, f, indent=2, ensure_ascii=False)

    print(f"\n✅ Rapport: {report_file}")


if __name__ == "__main__":
    main()
