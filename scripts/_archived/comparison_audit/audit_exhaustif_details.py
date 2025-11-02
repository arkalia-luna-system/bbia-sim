#!/usr/bin/env python3
"""Audit exhaustif pointilleux - tous les détails (doc, tests, assets, démos, etc.)."""

import json
import re
from pathlib import Path

# Chemins
BBIA_ROOT = Path(__file__).parent.parent
OFFICIAL_ROOT = Path("/Volumes/T7/reachy_mini")

report = {
    "documentation": [],
    "tests": [],
    "assets": [],
    "examples": [],
    "constants": [],
    "error_messages": [],
    "config_files": [],
}


def compare_docstrings() -> None:
    """Compare docstrings entre fichiers."""
    # À implémenter


def compare_tests() -> None:
    """Compare structure des tests."""
    official_tests = list((OFFICIAL_ROOT / "tests").glob("test_*.py"))
    bbia_tests = list((BBIA_ROOT / "tests").glob("test_*.py"))

    official_names = {t.stem for t in official_tests}
    bbia_names = {t.stem for t in bbia_tests}

    missing = official_names - bbia_names
    extra = bbia_names - official_names

    if missing:
        report["tests"].append(
            {"type": "tests_missing", "items": sorted(missing), "severity": "HIGH"},
        )

    if extra:
        report["tests"].append(
            {"type": "tests_extra", "items": sorted(extra), "severity": "INFO"},
        )


def compare_assets() -> None:
    """Compare assets (WAV, STL, JSON, etc.)."""
    # Assets audio officiels
    official_audio = list(
        (OFFICIAL_ROOT / "src" / "reachy_mini" / "assets").glob("*.wav"),
    )
    bbia_audio = list((BBIA_ROOT / "assets" / "voice").glob("*.wav"))

    official_wav = {f.name for f in official_audio if not f.name.startswith("._")}
    bbia_wav = {f.name for f in bbia_audio}

    missing_audio = official_wav - bbia_wav
    if missing_audio:
        report["assets"].append(
            {
                "type": "audio_missing",
                "files": sorted(missing_audio),
                "severity": "MEDIUM",
            },
        )


def compare_examples() -> None:
    """Compare exemples/démos."""
    official_examples = list((OFFICIAL_ROOT / "examples").glob("*.py"))
    bbia_examples = list((BBIA_ROOT / "examples" / "reachy_mini").glob("*.py"))

    official_names = {e.stem for e in official_examples}
    bbia_names = {e.stem for e in bbia_examples}

    missing = official_names - bbia_names
    if missing:
        report["examples"].append(
            {"type": "examples_missing", "files": sorted(missing), "severity": "MEDIUM"},
        )


def compare_constants() -> None:
    """Compare constantes."""
    official_constants = (
        OFFICIAL_ROOT / "src" / "reachy_mini" / "utils" / "constants.py"
    )
    if official_constants.exists():
        with open(official_constants) as f:
            official_content = f.read()

        # Chercher constantes dans BBIA
        constants_found = []
        for pattern in [r"URDF_ROOT_PATH", r"ASSETS_ROOT_PATH", r"MODELS_ROOT_PATH"]:
            if re.search(pattern, official_content):
                constants_found.append(pattern)

        report["constants"].append(
            {
                "type": "constants_check",
                "official_file": str(official_constants),
                "constants": constants_found,
            },
        )


def compare_error_messages() -> None:
    """Compare messages d'erreur."""
    # Rechercher patterns d'erreur communs
    patterns = [
        r"HTTPException.*404",
        r"HTTPException.*400",
        r"ValueError.*WARNING",
        r"FileNotFoundError",
    ]

    for pattern in patterns:
        official_matches = []
        bbia_matches = []

        for router_file in (
            OFFICIAL_ROOT / "src" / "reachy_mini" / "daemon" / "app" / "routers"
        ).glob("*.py"):
            with open(router_file) as f:
                content = f.read()
                if re.search(pattern, content):
                    official_matches.append(str(router_file))

        for router_file in (
            BBIA_ROOT / "src" / "bbia_sim" / "daemon" / "app" / "routers"
        ).glob("*.py"):
            with open(router_file) as f:
                content = f.read()
                if re.search(pattern, content):
                    bbia_matches.append(str(router_file))

        if official_matches or bbia_matches:
            report["error_messages"].append(
                {
                    "pattern": pattern,
                    "official": len(official_matches),
                    "bbia": len(bbia_matches),
                },
            )


def compare_config_files() -> None:
    """Compare fichiers de configuration."""
    # pyproject.toml
    official_pyproject = OFFICIAL_ROOT / "pyproject.toml"
    bbia_pyproject = BBIA_ROOT / "pyproject.toml"

    if official_pyproject.exists() and bbia_pyproject.exists():
        with open(official_pyproject) as f:
            official_content = f.read()
        with open(bbia_pyproject) as f:
            bbia_content = f.read()

        # Comparer sections clés
        sections = ["[project]", "[project.optional-dependencies]", "[tool.ruff]"]
        for section in sections:
            if section in official_content and section not in bbia_content:
                report["config_files"].append(
                    {
                        "type": "section_missing",
                        "file": "pyproject.toml",
                        "section": section,
                        "severity": "MEDIUM",
                    },
                )


def main() -> None:
    """Point d'entrée principal."""
    print("🔍 Audit exhaustif pointilleux...")

    compare_tests()
    compare_assets()
    compare_examples()
    compare_constants()
    compare_error_messages()
    compare_config_files()

    # Sauvegarder rapport
    output_file = BBIA_ROOT / "logs" / "audit_exhaustif_details.json"
    output_file.parent.mkdir(exist_ok=True)

    with open(output_file, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"✅ Rapport sauvegardé: {output_file}")

    # Afficher résumé
    total_issues = sum(len(v) for v in report.values())
    print(f"\n📊 Résumé: {total_issues} différences détectées")
    for category, items in report.items():
        if items:
            print(f"  - {category}: {len(items)}")


if __name__ == "__main__":
    main()
