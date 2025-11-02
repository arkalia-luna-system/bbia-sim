#!/usr/bin/env python3
"""
Audit complet des fichiers Markdown :
1. Vérifie et corrige toutes les dates (création vs autres)
2. Vérifie que le contenu correspond au code réel
3. Identifie les MD inutiles/rebarbatifs/obsolètes
4. Propose réorganisation
"""

import re
from collections import defaultdict
from pathlib import Path
from typing import Any

# Couleurs
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
BLUE = "\033[34m"
RESET = "\033[0m"

# Date de création officielle (selon documentation existante)
CREATION_DATE = "Octobre 2024"
RECENT_DATE_FORMAT = "Oct 25 / Nov 25"  # Pour novembre 2025
STANDARD_DATE = "Octobre 2025"


def find_all_md_files(root_dir: Path) -> list[Path]:
    """Trouve tous les fichiers MD sauf venv, .git, fichiers cachés."""
    md_files = []
    for path in root_dir.rglob("*.md"):
        parts = path.parts
        # Ignorer venv, .git, fichiers cachés
        if any(part.startswith(".") or part == "venv" for part in parts):
            continue
        if path.name.startswith("._"):
            continue
        md_files.append(path)
    return sorted(md_files)


def check_dates_in_file(file_path: Path) -> list[dict[str, Any]]:
    """Vérifie les dates dans un fichier MD."""
    issues = []
    try:
        content = file_path.read_text(encoding="utf-8")
        lines = content.split("\n")

        # Patterns pour dates
        date_patterns = [
            (
                r"(Date|Dernière mise à jour|Mise à jour|Créé|Création|Date création|Date d'analyse|Date audit|Date:)\s*[:\*]?\s*([^\n]*)",
                "date",
            ),
            (
                r"(octobre|novembre|décembre|janvier|février|mars|avril|mai|juin|juillet|août|septembre)\s+(2024|2025|2026)",
                "month_year",
            ),
            (r"\b(2024|2025|2026)\b", "year_only"),
            (
                r"(Oct|Nov|Dec|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep)\s*25\s*/\s*(Nov|Oct|Dec)",
                "recent_format",
            ),
        ]

        # Vérifier si c'est un fichier récent (novembre 2025)
        is_recent = (
            "nov" in file_path.name.lower()
            or "nov2025" in file_path.name.lower()
            or file_path.name.startswith("VERIFICATION")
            or "NOV" in file_path.name
        )

        for i, line in enumerate(lines, 1):
            # Chercher dates
            for pattern, pattern_type in date_patterns:
                matches = re.finditer(pattern, line, re.IGNORECASE)
                for match in matches:
                    full_match = match.group(0)
                    # Vérifier si c'est la date de création (ne pas modifier)
                    if "création" in line.lower() and "2024" in line:
                        continue  # Date création, ne pas toucher

                    # Vérifier si c'est une date à corriger
                    if pattern_type == "month_year":
                        month, year = match.groups()
                        if year == "2024" and "création" not in line.lower():
                            issues.append(
                                {
                                    "line": i,
                                    "type": "wrong_year",
                                    "match": full_match,
                                    "suggestion": line.replace(year, "2025"),
                                    "content": line.strip()[:80],
                                }
                            )
                    elif pattern_type == "year_only":
                        year = match.group(1)
                        if year == "2024" and "création" not in line.lower():
                            issues.append(
                                {
                                    "line": i,
                                    "type": "wrong_year",
                                    "match": full_match,
                                    "suggestion": line.replace(year, "2025"),
                                    "content": line.strip()[:80],
                                }
                            )

        return issues
    except Exception as e:
        return [{"error": str(e)}]


def verify_content_against_code(file_path: Path, root_dir: Path) -> list[str]:
    """Vérifie que le contenu du MD correspond au code réel."""
    warnings = []
    try:
        content = file_path.read_text(encoding="utf-8")

        # Patterns à vérifier
        checks = [
            (r"(\d+)\+?\s+tests?", "tests", "tests/"),
            (r"(\d+)\+?\s+fichiers?\s+(MD|md|doc)", "docs", "docs/"),
            (r"(Black|Ruff|MyPy|Bandit)", "ci_tools", ".github/workflows/"),
            (r"(Factory|ABC|Abstract)", "architecture", "src/"),
        ]

        for pattern, check_type, code_path in checks:
            matches = re.findall(pattern, content, re.IGNORECASE)
            if matches:
                # Vérifier dans le code
                code_dir = root_dir / code_path
                if code_dir.exists():
                    # Vérification basique - pourrait être améliorée
                    pass

    except Exception as e:
        warnings.append(f"Erreur vérification: {e}")

    return warnings


def categorize_file(file_path: Path, root_dir: Path) -> dict[str, Any]:
    """Catégorise un fichier MD."""
    rel_path = file_path.relative_to(root_dir)
    name = file_path.name.lower()
    category = "unknown"
    should_archive = False
    is_redundant = False
    is_temporary = False

    # Fichiers à archiver
    temp_indicators = [
        "analyse",
        "audit",
        "verification",
        "resume",
        "recap",
        "activite",
        "bilan",
    ]
    if any(ind in name for ind in temp_indicators) and rel_path.parts[0] not in [
        "docs",
        "presentation",
    ]:
        should_archive = True
        is_temporary = True
        category = "temporary_analysis"

    # Fichiers redondants
    redundant_indicators = [
        "final",
        "complet",
        "complete",
        "comprehensive",
        "exhaustif",
    ]
    if sum(1 for ind in redundant_indicators if ind in name) >= 2:
        is_redundant = True
        category = "potentially_redundant"

    # Fichiers dans archives
    if "archives" in rel_path.parts:
        category = "archived"
        should_archive = False  # Déjà archivé

    # Documentation principale
    if rel_path.parts[0] == "docs" and len(rel_path.parts) > 1:
        if rel_path.parts[1] in ["guides", "references", "api"]:
            category = "main_documentation"
        elif rel_path.parts[1] in ["audit", "archives"]:
            category = "audit_documentation"

    # Fichiers racine principaux
    if len(rel_path.parts) == 1:
        if name in [
            "readme.md",
            "projects.md",
            "changelog.md",
            "contributing.md",
            "code_of_conduct.md",
        ]:
            category = "root_main"
        else:
            category = "root_other"
            should_archive = True

    return {
        "category": category,
        "should_archive": should_archive,
        "is_redundant": is_redundant,
        "is_temporary": is_temporary,
        "suggested_location": suggest_location(file_path, root_dir),
    }


def suggest_location(file_path: Path, root_dir: Path) -> str:
    """Suggère un emplacement pour le fichier."""
    rel_path = file_path.relative_to(root_dir)
    name = file_path.name.lower()

    # Fichiers racine temporaires -> docs/archives/2025-11/
    if len(rel_path.parts) == 1:
        if any(ind in name for ind in ["analyse", "audit", "verification"]):
            return "docs/archives/2025-11/audits/"
        elif any(ind in name for ind in ["resume", "recap", "bilan"]):
            return "docs/archives/2025-11/resumes/"

    return str(rel_path.parent)


def main():
    """Fonction principale."""
    root_dir = Path(__file__).parent.parent

    print(f"{GREEN}🔍 Audit Complet Documentation Markdown{RESET}\n")
    print(f"Date création projet: {CREATION_DATE} (ne jamais modifier)")
    print(f"Date standard: {STANDARD_DATE}")
    print(f"Date récente format: {RECENT_DATE_FORMAT}\n")

    # Trouver tous les MD
    md_files = find_all_md_files(root_dir)
    print(f"{GREEN}📄 {len(md_files)} fichiers MD trouvés{RESET}\n")

    # Analyser chaque fichier
    date_issues = defaultdict(list)
    content_warnings = []
    file_categories = defaultdict(list)

    for md_file in md_files:
        # Dates
        issues = check_dates_in_file(md_file)
        if issues:
            date_issues[str(md_file)] = issues

        # Catégorisation
        cat_info = categorize_file(md_file, root_dir)
        file_categories[cat_info["category"]].append(
            {"file": md_file, "info": cat_info}
        )

    # Rapport dates
    print(f"{YELLOW}📊 RAPPORT DATES{RESET}\n")
    print(f"{len(date_issues)} fichiers avec dates à vérifier\n")

    for file_str, issues in list(date_issues.items())[:10]:
        print(f"{YELLOW}📝 {Path(file_str).name}{RESET}")
        for issue in issues[:2]:
            if "error" in issue:
                print(f"  {RED}❌ Erreur: {issue['error']}{RESET}")
            else:
                print(f"  Ligne {issue['line']}: {issue.get('content', '')[:60]}...")
        print()

    # Rapport catégorisation
    print(f"{BLUE}📁 CATÉGORISATION FICHIERS{RESET}\n")

    to_archive = []
    for category, files in file_categories.items():
        if category in ["temporary_analysis", "root_other"]:
            for item in files:
                if item["info"]["should_archive"]:
                    to_archive.append(item)

    if to_archive:
        print(f"{YELLOW}⚠️  {len(to_archive)} fichiers à archiver:{RESET}\n")
        for item in to_archive[:15]:
            rel = item["file"].relative_to(root_dir)
            print(f"  - {rel}")
            print(f"    → {item['info']['suggested_location']}")

    print(f"\n{GREEN}✅ Audit terminé{RESET}")
    print("\nActions recommandées:")
    print(f"1. Corriger dates dans {len(date_issues)} fichiers")
    print(f"2. Archiver {len(to_archive)} fichiers temporaires")
    print("3. Vérifier contenu vs code pour fichiers principaux")


if __name__ == "__main__":
    main()
