#!/usr/bin/env python3
"""Audit et correction des dates dans tous les fichiers MD."""

import re
from pathlib import Path

# Date de création réelle (depuis PROJECTS.md mentionne Avril 2025)
DATE_CREATION = "Avril 2025"

# Dates à utiliser
DATE_OCT_2025 = "octobre 2025"
DATE_OCT_2025_CAPS = "Octobre 2025"
DATE_OCT_NOV_2025 = "Oct 2025 / Nov 2025"

# Patterns de dates à trouver et corriger
DATE_PATTERNS = [
    # Patterns incorrects à corriger
    (r"2024", "2025"),
    (r"2026", "2025"),
    (r"janvier 2025", DATE_OCT_2025),
    (r"février 2025", DATE_OCT_2025),
    (r"mars 2025", DATE_OCT_2025),
    (r"avril 2025", DATE_OCT_2025),  # Sauf date de création
    (r"mai 2025", DATE_OCT_2025),
    (r"juin 2025", DATE_OCT_2025),
    (r"juillet 2025", DATE_OCT_2025),
    (r"août 2025", DATE_OCT_2025),
    (r"septembre 2025", DATE_OCT_2025),
    (r"novembre 2025", DATE_OCT_NOV_2025),
    (r"décembre 2025", DATE_OCT_2025),
    # Patterns avec majuscules
    (r"Janvier 2025", DATE_OCT_2025_CAPS),
    (r"Février 2025", DATE_OCT_2025_CAPS),
    (r"Mars 2025", DATE_OCT_2025_CAPS),
    (r"Mai 2025", DATE_OCT_2025_CAPS),
    (r"Juin 2025", DATE_OCT_2025_CAPS),
    (r"Juillet 2025", DATE_OCT_2025_CAPS),
    (r"Août 2025", DATE_OCT_2025_CAPS),
    (r"Septembre 2025", DATE_OCT_2025_CAPS),
    (r"Novembre 2025", DATE_OCT_NOV_2025),
    (r"Décembre 2025", DATE_OCT_2025_CAPS),
]

# Fichiers MD récents (à mettre Oct 25 / Nov 25)
RECENTS_INDICATORS = [
    "ANALYSE_PROFIL_PROFESSIONNEL",
    "ANALYSE_VERIFICATION_PROJET",
    "RECAP_4_RECOMMANDATIONS",
    "CHECKLIST_VALIDATION_HARDWARE_DECEMBRE",
    "RAPPORT_AUDIT_EXHAUSTIF_DETAILS_NOV2025",
    "VERIFICATION_FINALE_NOV2025",
]


def is_recent_file(filename: str) -> bool:
    """Vérifie si un fichier est récent (Nov 2025)."""
    return any(ind in filename for ind in RECENTS_INDICATORS)


def find_md_files(root: Path) -> list[Path]:
    """Trouve tous les fichiers MD."""
    md_files = []
    for path in root.rglob("*.md"):
        # Ignorer venv, .git, etc.
        if any(
            ignore in str(path)
            for ignore in [".git", "venv", ".venv", "htmlcov", "__pycache__"]
        ):
            continue
        md_files.append(path)
    return sorted(md_files)


def audit_file(filepath: Path) -> dict:
    """Audit un fichier MD."""
    try:
        content = filepath.read_text(encoding="utf-8")
    except Exception as e:
        return {"error": str(e), "file": str(filepath)}

    issues = []
    changes = []
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
                changes.append((pattern, replacement))

    # Chercher "Dernière mise à jour" ou dates récentes
    last_update_pattern = r"Dernière mise à jour\s*[:]\s*(.+?)(\n|$)"
    matches = list(re.finditer(last_update_pattern, content, re.IGNORECASE))
    for match in matches:
        line_num = content[: match.start()].count("\n") + 1
        current_date = match.group(1).strip()
        if is_recent and DATE_OCT_NOV_2025 not in current_date:
            issues.append(
                f"Ligne {line_num}: Date récente à mettre à jour → '{DATE_OCT_NOV_2025}'",
            )

    return {
        "file": str(filepath),
        "is_recent": is_recent,
        "issues": issues,
        "changes": changes,
        "line_count": len(content.splitlines()),
    }


def main():
    """Audit principal."""
    root = Path()
    md_files = find_md_files(root)

    print(f"📋 Audit de {len(md_files)} fichiers MD\n")

    files_with_issues = []
    for md_file in md_files:
        result = audit_file(md_file)
        if result.get("issues"):
            files_with_issues.append(result)
            print(f"⚠️  {md_file.name}")
            for issue in result["issues"]:
                print(f"   {issue}")

    print(f"\n✅ {len(md_files) - len(files_with_issues)} fichiers OK")
    print(f"⚠️  {len(files_with_issues)} fichiers avec problèmes")

    # Générer rapport
    report = Path("artifacts/audit_dates_md.json")
    report.parent.mkdir(parents=True, exist_ok=True)
    import json

    report.write_text(json.dumps(files_with_issues, indent=2, ensure_ascii=False))
    print(f"\n📄 Rapport sauvegardé: {report}")


if __name__ == "__main__":
    main()
