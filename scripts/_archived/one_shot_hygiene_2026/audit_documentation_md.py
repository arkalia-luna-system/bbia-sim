#!/usr/bin/env python3
"""Script d'audit des fichiers MD principaux contre le code réel.

Vérifie ligne par ligne chaque affirmation dans la documentation
et détecte les incohérences avec le code source.
"""

import re
import subprocess
from pathlib import Path
from typing import Any

# Fichiers MD principaux à auditer
MD_FILES = [
    "README.md",
    "docs/guides/GUIDE_DEMARRAGE.md",
    "docs/guides/GUIDE_AVANCE.md",
    "docs/conformite/CONFORMITE_REACHY_MINI_COMPLETE.md",
    "docs/guides/REACHY_MINI_WIRELESS_COMPLETE_GUIDE.md",
    "docs/architecture/ARCHITECTURE_OVERVIEW.md",
    "docs/guides_techniques/TESTING_GUIDE.md",
    "docs/api/CONTRATS_REST_WS.md",
    "docs/status.md",
    "docs/references/INDEX_THEMATIQUE.md",
]

# Patterns à vérifier
PATTERNS = {
    "test_count": [
        r"(\d+)\+?\s*tests?",
        r"(\d+)\s*tests?\s+pass",
        r"(\d+)/(\d+)\s*tests?",
    ],
    "file_exists": [
        r"`([a-zA-Z0-9_/\.-]+\.py)`",
        r"([a-zA-Z0-9_/\.-]+\.py)",
        r"`([a-zA-Z0-9_/\.-]+\.sh)`",
    ],
    "class_exists": [
        r"class\s+([A-Z][a-zA-Z0-9_]+)",
        r"`([A-Z][a-zA-Z0-9_]+)`",
    ],
    "method_exists": [
        r"`?([a-z_]+)\(\)`?",
        r"def\s+([a-z_]+)\(",
    ],
    "path_exists": [
        r"`([a-zA-Z0-9_/\.-]+)`",
    ],
}


def count_tests() -> int:
    """Compte le nombre réel de tests."""
    try:
        result = subprocess.run(
            ["python", "-m", "pytest", "--collect-only", "-q"],
            check=False,
            capture_output=True,
            text=True,
            timeout=30,
        )
        # Extraire le nombre depuis la sortie
        match = re.search(r"(\d+)\s+tests? collected", result.stdout)
        if match:
            return int(match.group(1))
    except Exception:
        pass
    return 0


def check_file_exists(filepath: str) -> bool:
    """Vérifie si un fichier existe."""
    return Path(filepath).exists()


def check_class_exists(classname: str, root_dir: Path) -> bool:
    """Vérifie si une classe existe dans le code."""
    for py_file in root_dir.rglob("*.py"):
        if py_file.name.startswith("test_"):
            continue
        try:
            content = py_file.read_text()
            if f"class {classname}" in content:
                return True
        except Exception:
            pass
    return False


def check_method_exists(methodname: str, classname: str | None, root_dir: Path) -> bool:
    """Vérifie si une méthode existe."""
    for py_file in root_dir.rglob("*.py"):
        if py_file.name.startswith("test_"):
            continue
        try:
            content = py_file.read_text()
            if classname:
                # Chercher dans une classe spécifique
                class_pattern = rf"class\s+{classname}[^:]*:.*?def\s+{methodname}\("
                if re.search(class_pattern, content, re.DOTALL):
                    return True
            # Chercher méthode standalone
            elif f"def {methodname}(" in content:
                return True
        except Exception:
            pass
    return False


def audit_md_file(md_path: Path, root_dir: Path) -> list[dict[str, Any]]:
    """Audite un fichier MD contre le code réel."""
    issues = []
    content = md_path.read_text()

    # Vérifier nombre de tests
    test_count_real = count_tests()
    test_matches = re.findall(r"(\d+)\+?\s*tests?", content, re.IGNORECASE)
    for match in test_matches:
        count_claimed = int(match)
        if (
            count_claimed != test_count_real
            and abs(count_claimed - test_count_real) > 50
        ):
            issues.append(
                {
                    "type": "test_count_mismatch",
                    "line": "N/A",
                    "claimed": count_claimed,
                    "real": test_count_real,
                    "file": str(md_path),
                },
            )

    # Vérifier fichiers mentionnés
    file_matches = re.findall(r"`([a-zA-Z0-9_/\.-]+\.(?:py|sh|xml|stl))`", content)
    for file_match in file_matches:
        if not check_file_exists(file_match):
            # Vérifier si c'est un chemin relatif depuis la racine
            if not check_file_exists(root_dir / file_match):
                issues.append(
                    {
                        "type": "file_not_found",
                        "file_mentioned": file_match,
                        "md_file": str(md_path),
                    },
                )

    # Vérifier classes mentionnées
    class_matches = re.findall(r"`?([A-Z][a-zA-Z0-9_]+)`?(?:\s+class|\(|\.)", content)
    for class_match in class_matches:
        if class_match not in ["True", "False", "None", "Dict", "List", "Any"]:
            if not check_class_exists(class_match, root_dir):
                issues.append(
                    {
                        "type": "class_not_found",
                        "class_mentioned": class_match,
                        "md_file": str(md_path),
                    },
                )

    return issues


def main():
    """Fonction principale."""
    root_dir = Path(__file__).parent.parent
    all_issues = []

    print("🔍 AUDIT DOCUMENTATION MD\n")
    print("=" * 80)

    for md_file in MD_FILES:
        md_path = root_dir / md_file
        if not md_path.exists():
            print(f"❌ Fichier non trouvé: {md_file}")
            continue

        print(f"\n📄 Auditing: {md_file}")
        issues = audit_md_file(md_path, root_dir)
        all_issues.extend(issues)

        if issues:
            print(f"  ⚠️  {len(issues)} problème(s) détecté(s)")
            for issue in issues[:5]:  # Afficher max 5 par fichier
                print(
                    f"    - {issue['type']}: {issue.get('file_mentioned', issue.get('class_mentioned', 'N/A'))}",
                )
        else:
            print("  ✅ Aucun problème détecté")

    print("\n" + "=" * 80)
    print("\n📊 RÉSUMÉ:")
    print(f"  Total problèmes: {len(all_issues)}")
    print(f"  Fichiers audités: {len(MD_FILES)}")

    # Générer rapport
    report_path = root_dir / "docs" / "audit" / "AUDIT_DOCUMENTATION_MD_RAPPORT.md"
    report_path.parent.mkdir(parents=True, exist_ok=True)

    with open(report_path, "w") as f:
        f.write("# 🔍 AUDIT DOCUMENTATION MD - RAPPORT\n\n")
        f.write(f"**Date**: {Path(__file__).stat().st_mtime}\n\n")
        f.write(f"**Total problèmes**: {len(all_issues)}\n\n")
        f.write("## Problèmes détectés\n\n")
        for issue in all_issues[:50]:  # Limiter à 50
            f.write(f"- **{issue['type']}**: {issue}\n")

    print(f"\n📄 Rapport généré: {report_path}")

    return len(all_issues)


if __name__ == "__main__":
    exit(main())
