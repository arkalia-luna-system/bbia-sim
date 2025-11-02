#!/usr/bin/env python3
"""Correction automatique des dates dans tous les fichiers MD."""

import re
from pathlib import Path

# Fichiers récents (Nov 2025)
RECENTS = [
    "ANALYSE_PROFIL_PROFESSIONNEL",
    "ANALYSE_VERIFICATION_PROJET",
    "RECAP_4_RECOMMANDATIONS",
    "CHECKLIST_VALIDATION_HARDWARE_DECEMBRE",
    "RAPPORT_AUDIT_EXHAUSTIF_DETAILS_NOV2025",
    "VERIFICATION_FINALE_NOV2025",
    "ACTIVITE_GIT_ANALYSE",
]

# Patterns de remplacement
REPLACEMENTS = [
    # Années incorrectes
    (
        r"\b2024\b(?!.*création|Création)",
        "2025",
    ),  # Ne pas toucher les dates de création
    (r"\b2026\b", "2025"),
    # Mois incorrects → Octobre 2025
    (r"\bjanvier 2025\b", "octobre 2025"),
    (r"\bJanvier 2025\b", "Octobre 2025"),
    (r"\bfévrier 2025\b", "octobre 2025"),
    (r"\bFévrier 2025\b", "Octobre 2025"),
    (r"\bmars 2025\b", "octobre 2025"),
    (r"\bMars 2025\b", "Octobre 2025"),
    # Avril : garder seulement pour date de création
    (r"\bavril 2025\b(?!.*[Cc]réation)", "octobre 2025"),
    (r"\bAvril 2025\b(?!.*[Cc]réation)", "Octobre 2025"),
    (r"\bmai 2025\b", "octobre 2025"),
    (r"\bMai 2025\b", "Octobre 2025"),
    (r"\bjuin 2025\b", "octobre 2025"),
    (r"\bJuin 2025\b", "Octobre 2025"),
    (r"\bjuillet 2025\b", "octobre 2025"),
    (r"\bJuillet 2025\b", "Octobre 2025"),
    (r"\baoût 2025\b", "octobre 2025"),
    (r"\bAoût 2025\b", "Octobre 2025"),
    (r"\bseptembre 2025\b", "octobre 2025"),
    (r"\bSeptembre 2025\b", "Octobre 2025"),
    # Novembre → Oct 2025 / Nov 2025 pour fichiers récents
    (r"\bnovembre 2025\b", "Oct 2025 / Nov 2025"),
    (r"\bNovembre 2025\b", "Oct 2025 / Nov 2025"),
    (r"\bdécembre 2025\b", "octobre 2025"),
    (r"\bDécembre 2025\b", "Octobre 2025"),
]


def is_recent_file(filename: str) -> bool:
    """Vérifie si fichier récent."""
    return any(ind in filename for ind in RECENTS)


def correct_file(filepath: Path, dry_run: bool = False) -> tuple[int, list[str]]:
    """Corrige les dates dans un fichier."""
    try:
        content = filepath.read_text(encoding="utf-8")
    except Exception as e:
        return 0, [f"Erreur lecture: {e}"]

    original = content
    changes = []
    is_recent = is_recent_file(filepath.name)

    # Appliquer remplacements
    for pattern, replacement in REPLACEMENTS:
        matches = list(re.finditer(pattern, content, re.IGNORECASE))
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
            content = re.sub(pattern, replacement, content, flags=re.IGNORECASE)

    # Fichiers récents: mettre "Dernière mise à jour" à "Oct 2025 / Nov 2025"
    if is_recent:
        pattern = r"(Dernière mise à jour\s*[:]\s*)([^\n]+)"

        def replacer(m):
            date = m.group(2).strip()
            if "Oct 2025 / Nov 2025" not in date and "octobre 2025" not in date.lower():
                return f"{m.group(1)}Oct 2025 / Nov 2025"
            return m.group(0)

        content = re.sub(pattern, replacer, content, flags=re.IGNORECASE)

    if content != original and not dry_run:
        filepath.write_text(content, encoding="utf-8")

    return len(changes), changes


def main():
    """Correction principale."""
    root = Path()
    md_files = sorted(root.rglob("*.md"))
    md_files = [
        f
        for f in md_files
        if not any(ignore in str(f) for ignore in [".git", "venv", ".venv", "htmlcov"])
    ]

    print(f"🔧 Correction de {len(md_files)} fichiers MD\n")

    total_changes = 0
    files_changed = []

    for md_file in md_files:
        count, changes = correct_file(md_file, dry_run=False)
        if count > 0:
            total_changes += count
            files_changed.append((md_file, count, changes))
            print(f"✅ {md_file.name}: {count} correction(s)")
            if changes:
                for change in changes[:3]:  # Afficher max 3 changements
                    print(f"   {change}")
                if len(changes) > 3:
                    print(f"   ... et {len(changes) - 3} autres")

    print("\n📊 Résumé:")
    print(f"   {len(files_changed)} fichiers modifiés")
    print(f"   {total_changes} corrections appliquées")


if __name__ == "__main__":
    main()
