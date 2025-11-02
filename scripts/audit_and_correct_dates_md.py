#!/usr/bin/env python3
"""
Script unifié pour auditer et corriger les dates dans tous les fichiers MD.

Fusion de :
- audit_dates_md.py (audit)
- audit_md_dates.py (git commit date)
- correct_dates_md.py (correction)
- correct_dates_md_final.py (correction robuste)

Usage:
    python scripts/audit_and_correct_dates_md.py           # Audit + correction
    python scripts/audit_and_correct_dates_md.py --audit-only  # Audit uniquement
    python scripts/audit_and_correct_dates_md.py --correct    # Correction uniquement
"""

import argparse
import json
import re
import subprocess
from pathlib import Path

# Date de création réelle (depuis PROJECTS.md mentionne Avril 2025)
CREATION_DATE = "Avril 2025"
DATE_OCT_2025 = "octobre 2025"
DATE_OCT_2025_CAPS = "Octobre 2025"
DATE_OCT_NOV_2025 = "Oct 25 / Nov 25"

# Fichiers MD récents (à mettre Oct 25 / Nov 25)
RECENTS_INDICATORS = [
    "ANALYSE_PROFIL_PROFESSIONNEL",
    "ANALYSE_VERIFICATION_PROJET",
    "RECAP_4_RECOMMANDATIONS",
    "CHECKLIST_VALIDATION_HARDWARE_DECEMBRE",
    "RAPPORT_AUDIT_EXHAUSTIF_DETAILS_NOV2025",
    "VERIFICATION_FINALE_NOV2025",
    "ACTIVITE_GIT_ANALYSE",
]

# Patterns de dates à corriger
DATE_PATTERNS = [
    (r"\b2024\b(?!.*création|Création)", "2025"),
    (r"\b2026\b", "2025"),
    (r"\bjanvier 2025\b", DATE_OCT_2025),
    (r"\bJanvier 2025\b", DATE_OCT_2025_CAPS),
    (r"\bfévrier 2025\b", DATE_OCT_2025),
    (r"\bFévrier 2025\b", DATE_OCT_2025_CAPS),
    (r"\bmars 2025\b", DATE_OCT_2025),
    (r"\bMars 2025\b", DATE_OCT_2025_CAPS),
    (r"\bmai 2025\b", DATE_OCT_2025),
    (r"\bMai 2025\b", DATE_OCT_2025_CAPS),
    (r"\bjuin 2025\b", DATE_OCT_2025),
    (r"\bJuin 2025\b", DATE_OCT_2025_CAPS),
    (r"\bjuillet 2025\b", DATE_OCT_2025),
    (r"\bJuillet 2025\b", DATE_OCT_2025_CAPS),
    (r"\baoût 2025\b", DATE_OCT_2025),
    (r"\bAoût 2025\b", DATE_OCT_2025_CAPS),
    (r"\bseptembre 2025\b", DATE_OCT_2025),
    (r"\bSeptembre 2025\b", DATE_OCT_2025_CAPS),
    (r"\bnovembre 2025\b", DATE_OCT_NOV_2025),
    (r"\bNovembre 2025\b", DATE_OCT_NOV_2025),
    (r"\bdécembre 2025\b", DATE_OCT_2025),
    (r"\bDécembre 2025\b", DATE_OCT_2025_CAPS),
]


def get_first_commit_date() -> str | None:
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


def is_creation_date_context(line: str) -> bool:
    """Détermine si la ligne parle de la date de création."""
    creation_keywords = [
        "date création",
        "date de création",
        "créé",
        "première release",
        "depuis octobre 2024",
        "commits depuis",
        "initial",
        "début",
    ]
    line_lower = line.lower()
    return any(keyword in line_lower for keyword in creation_keywords)


def is_recent_file(filename: str) -> bool:
    """Vérifie si un fichier est récent (Nov 2025)."""
    return any(ind in filename for ind in RECENTS_INDICATORS)


def find_md_files(root: Path) -> list[Path]:
    """Trouve tous les fichiers MD."""
    md_files = []
    for path in root.rglob("*.md"):
        if any(
            ignore in str(path)
            for ignore in [".git", "venv", ".venv", "htmlcov", "__pycache__"]
        ):
            continue
        if path.name.startswith("._"):
            continue
        md_files.append(path)
    return sorted(md_files)


def audit_file(filepath: Path) -> dict:
    """Audit un fichier MD pour trouver les dates incorrectes."""
    try:
        content = filepath.read_text(encoding="utf-8")
    except Exception as e:
        return {"error": str(e), "file": str(filepath)}

    issues = []
    is_recent = is_recent_file(filepath.name)

    # Chercher dates incorrectes
    for pattern, replacement in DATE_PATTERNS:
        matches = list(re.finditer(pattern, content, re.IGNORECASE))
        if matches:
            for match in matches:
                line_num = content[: match.start()].count("\n") + 1
                old_text = match.group(0)
                # Ne pas corriger la date de création
                if (
                    "création"
                    in content[max(0, match.start() - 50) : match.start()].lower()
                    and "avril" in old_text.lower()
                ):
                    continue
                issues.append(f"Ligne {line_num}: '{old_text}' → '{replacement}'")

    # Chercher "Dernière mise à jour" ou dates récentes
    last_update_pattern = r"Dernière mise à jour\s*[:]\s*(.+?)(\n|$)"
    matches = list(re.finditer(last_update_pattern, content, re.IGNORECASE))
    for match in matches:
        line_num = content[: match.start()].count("\n") + 1
        current_date = match.group(1).strip()
        if is_recent and DATE_OCT_NOV_2025 not in current_date:
            issues.append(
                f"Ligne {line_num}: Date récente à mettre à jour → '{DATE_OCT_NOV_2025}'"
            )

    return {
        "file": str(filepath),
        "is_recent": is_recent,
        "issues": issues,
        "line_count": len(content.splitlines()),
    }


def correct_dates_in_content(content: str, file_path: Path) -> tuple[str, list[str]]:
    """Corrige les dates dans le contenu."""
    changes = []
    lines = content.split("\n")
    is_recent = is_recent_file(file_path.name)

    for i, line in enumerate(lines):
        original_line = line

        # Ne pas modifier les dates de création
        if is_creation_date_context(line) and "2024" in line:
            continue

        # Appliquer remplacements
        for pattern, replacement in DATE_PATTERNS:
            matches = list(re.finditer(pattern, line, re.IGNORECASE))
            if matches:
                for match in matches:
                    # Ne pas modifier si c'est la date de création avec "avril"
                    context_before = content[
                        max(0, match.start() - 100) : match.start()
                    ].lower()
                    if "avril" in match.group(0).lower() and (
                        "création" in context_before or "creation" in context_before
                    ):
                        continue
                    changes.append(f"'{match.group(0)}' → '{replacement}'")
                line = re.sub(pattern, replacement, line, flags=re.IGNORECASE)

        # Fichiers récents: mettre "Dernière mise à jour" à "Oct 2025 / Nov 2025"
        if is_recent:
            pattern = r"(Dernière mise à jour\s*[:]\s*)([^\n]+)"

            def replacer(m):
                date = m.group(2).strip()
                if (
                    DATE_OCT_NOV_2025 not in date
                    and "octobre 2025" not in date.lower()
                ):
                    return f"{m.group(1)}Oct 2025 / Nov 2025"
                return m.group(0)

            line = re.sub(pattern, replacer, line, flags=re.IGNORECASE)

        lines[i] = line

    return "\n".join(lines), changes


def main():
    """Fonction principale."""
    parser = argparse.ArgumentParser(
        description="Audit et correction des dates dans les fichiers MD"
    )
    parser.add_argument(
        "--audit-only",
        action="store_true",
        help="Effectuer uniquement l'audit (ne pas corriger)",
    )
    parser.add_argument(
        "--correct",
        action="store_true",
        help="Effectuer uniquement la correction (sans audit)",
    )
    args = parser.parse_args()

    root = Path(__file__).parent.parent
    md_files = find_md_files(root)

    # Récupérer date création depuis git
    first_commit_date = get_first_commit_date()
    if first_commit_date:
        print(f"✅ Date de création (premier commit Git): {first_commit_date}\n")

    print(f"📋 Traitement de {len(md_files)} fichiers MD\n")

    # Audit
    files_with_issues = []
    if not args.correct:
        print("🔍 Audit des dates...")
        for md_file in md_files:
            result = audit_file(md_file)
            if result.get("issues"):
                files_with_issues.append(result)
                print(f"⚠️  {md_file.name}")
                for issue in result["issues"][:3]:
                    print(f"   {issue}")

        print(f"\n✅ {len(md_files) - len(files_with_issues)} fichiers OK")
        print(f"⚠️  {len(files_with_issues)} fichiers avec problèmes")

        # Générer rapport
        report = Path("artifacts/audit_dates_md.json")
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(
            json.dumps(files_with_issues, indent=2, ensure_ascii=False)
        )
        print(f"\n📄 Rapport sauvegardé: {report}\n")

    # Correction
    if not args.audit_only:
        print("🔧 Correction des dates...")
        files_changed = []
        total_changes = 0

        for md_file in md_files:
            try:
                content = md_file.read_text(encoding="utf-8")
                new_content, changes = correct_dates_in_content(content, md_file)

                if changes:
                    md_file.write_text(new_content, encoding="utf-8")
                    files_changed.append((md_file, changes))
                    total_changes += len(changes)
                    print(f"✅ {md_file.relative_to(root)}")
                    for change in changes[:3]:
                        print(f"   {change}")
            except Exception as e:
                print(f"❌ Erreur {md_file}: {e}")

        print(f"\n✅ {len(files_changed)} fichiers modifiés ({total_changes} changements)")


if __name__ == "__main__":
    main()

