#!/usr/bin/env python3
"""
Script pour auditer et corriger les dates dans tous les fichiers MD.
Recherche les dates et les met à jour selon les règles :
- Date de création = premier commit Git (ne jamais modifier)
- Dates récentes (Oct 25 / Nov 25) = Octobre 2025 / Novembre 2025
- Autres dates = Octobre 2025
"""

import re
import subprocess
from pathlib import Path

# Couleurs pour output
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
RESET = "\033[0m"


def get_first_commit_date():
    """Trouve la date du premier commit Git."""
    try:
        result = subprocess.run(
            ["git", "log", "--all", "--reverse", "--format=%ai"],
            capture_output=True,
            text=True,
            check=True,
        )
        first_date = result.stdout.strip().split("\n")[0]
        if first_date:
            return first_date.split()[0]  # Juste la date YYYY-MM-DD
    except Exception:
        pass
    return None


def find_md_files(root_dir):
    """Trouve tous les fichiers MD sauf venv et fichiers cachés."""
    md_files = []
    for path in Path(root_dir).rglob("*.md"):
        # Ignorer venv, .git, fichiers cachés
        if any(part.startswith(".") or part == "venv" for part in path.parts):
            continue
        if path.name.startswith("._"):
            continue
        md_files.append(path)
    return sorted(md_files)


def analyze_md_file(file_path, first_commit_date):
    """Analyse un fichier MD pour trouver les dates à corriger."""
    issues = []
    try:
        content = file_path.read_text(encoding="utf-8")

        # Patterns de dates à chercher
        patterns = [
            r"(Date|Dernière mise à jour|Mise à jour|Créé|Création|Date création):\s*[^\n]*(2024|2025|Novembre|Octobre|novembre|octobre)",
            r"\*\*Date[^\*]*\*\*[^\n]*(2024|2025|Novembre|Octobre|novembre|octobre)",
            r"Version[^\n]*(2024|2025)",
            r"octobre\s+2024|Octobre\s+2024|OCTOBRE\s+2024",
            r"novembre\s+2024|Novembre\s+2024|NOVEMBRE\s+2024",
        ]

        for i, line in enumerate(content.split("\n"), 1):
            for pattern in patterns:
                matches = re.finditer(pattern, line, re.IGNORECASE)
                for match in matches:
                    issues.append(
                        {
                            "line": i,
                            "content": line.strip(),
                            "match": match.group(),
                        }
                    )

        return issues
    except Exception as e:
        return [{"error": str(e)}]


def main():
    """Fonction principale."""
    root_dir = Path(__file__).parent.parent

    print(f"{GREEN}🔍 Audit des fichiers MD - Dates{RESET}\n")

    # Trouver date création
    first_date = get_first_commit_date()
    if first_date:
        print(f"{GREEN}✅ Date de création (premier commit Git): {first_date}{RESET}\n")
    else:
        print(f"{YELLOW}⚠️  Impossible de trouver la date du premier commit{RESET}\n")

    # Trouver tous les MD
    md_files = find_md_files(root_dir)
    print(f"{GREEN}📄 {len(md_files)} fichiers MD trouvés{RESET}\n")

    # Analyser
    files_with_issues = []
    for md_file in md_files:
        issues = analyze_md_file(md_file, first_date)
        if issues:
            files_with_issues.append(
                {
                    "file": md_file,
                    "issues": issues,
                }
            )

    # Rapport
    print(f"{YELLOW}📊 RAPPORT{RESET}\n")
    print(f"{len(files_with_issues)} fichiers avec dates à vérifier\n")

    for item in files_with_issues[:20]:  # Limiter à 20 pour l'instant
        print(f"{YELLOW}📝 {item['file']}{RESET}")
        for issue in item["issues"][:3]:  # 3 premières issues par fichier
            if "error" in issue:
                print(f"  {RED}❌ Erreur: {issue['error']}{RESET}")
            else:
                print(f"  Ligne {issue['line']}: {issue['match'][:60]}...")
        print()


if __name__ == "__main__":
    main()
