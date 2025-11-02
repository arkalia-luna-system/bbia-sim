#!/usr/bin/env python3
"""Correction automatique des dates dans tous les fichiers MD selon les règles :
- Date création : Octobre 2024 (NE JAMAIS MODIFIER)
- Dates récentes (novembre) : Oct 25 / Nov 25
- Autres dates : Octobre 2025
"""

import re
from pathlib import Path

CREATION_DATE = "Octobre 2024"
RECENT_DATE_FORMAT = "Oct 25 / Nov 25"
STANDARD_DATE = "Octobre 2025"


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


def is_recent_file(file_path: Path) -> bool:
    """Détermine si le fichier est récent (novembre 2025)."""
    name_lower = file_path.name.lower()
    recent_indicators = ["nov", "nov2025", "verification", "nov25"]
    return any(ind in name_lower for ind in recent_indicators) or "2025-11" in str(
        file_path.parent,
    )


def correct_dates_in_content(content: str, file_path: Path) -> tuple[str, list[str]]:
    """Corrige les dates dans le contenu."""
    changes = []
    lines = content.split("\n")
    is_recent = is_recent_file(file_path)

    for i, line in enumerate(lines):
        original_line = line

        # Ne pas modifier les dates de création
        if is_creation_date_context(line) and "2024" in line:
            continue

        # Corriger "avril 2024" ou "april 2024" → "octobre 2024" (seulement si contexte création)
        if "avril 2024" in line.lower() or "april 2024" in line.lower():
            if is_creation_date_context(line):
                line = re.sub(
                    r"(avril|april)\s+2024",
                    "octobre 2024",
                    line,
                    flags=re.IGNORECASE,
                )
                if line != original_line:
                    changes.append(f"L{i+1}: {line[:60]}...")

        # Corriger années incorrectes (2024 → 2025) sauf si contexte création
        if not is_creation_date_context(line):
            # 2024 → 2025 (sauf dans contexte création)
            if (
                re.search(r"\b2024\b", line)
                and "création" not in line.lower()
                and "octobre 2024" not in line
            ):
                line = re.sub(r"\b2024\b", "2025", line)
                if line != original_line:
                    changes.append(f"L{i+1}: {line[:60]}...")

            # Corriger dates mois + année
            # "octobre 2024" → "octobre 2025" (sauf si contexte création)
            if "octobre 2024" in line.lower() and not is_creation_date_context(line):
                line = re.sub(
                    r"octobre\s+2024",
                    "octobre 2025",
                    line,
                    flags=re.IGNORECASE,
                )
                if line != original_line:
                    changes.append(f"L{i+1}: {line[:60]}...")

        # Pour fichiers récents, mettre format "Oct 25 / Nov 25"
        if is_recent:
            # "octobre 2025" ou "novembre 2025" en début → "Oct 25 / Nov 25"
            if i < 10 and (
                "octobre 2025" in line.lower() or "novembre 2025" in line.lower()
            ):
                if re.search(
                    r"date.*:.*(octobre|novembre)\s+2025",
                    line,
                    re.IGNORECASE,
                ):
                    line = re.sub(
                        r"(octobre|novembre)\s+2025",
                        RECENT_DATE_FORMAT,
                        line,
                        flags=re.IGNORECASE,
                        count=1,
                    )
                    if line != original_line:
                        changes.append(f"L{i+1}: {line[:60]}...")

        lines[i] = line

    return "\n".join(lines), changes


def find_all_md_files(root_dir: Path) -> list[Path]:
    """Trouve tous les fichiers MD."""
    md_files = []
    for path in root_dir.rglob("*.md"):
        # Exclure venv, .git, fichiers cachés
        parts = path.parts
        if any(part.startswith(".") for part in parts):
            continue
        if "venv" in parts or "venv-" in str(path):
            continue
        if path.name.startswith("._"):
            continue
        md_files.append(path)
    return sorted(md_files)


def main():
    """Fonction principale."""
    root_dir = Path(__file__).parent.parent
    md_files = find_all_md_files(root_dir)

    print(f"🔍 Correction des dates dans {len(md_files)} fichiers MD\n")
    print("Règles:")
    print(f"  - Date création: {CREATION_DATE} (ne jamais modifier)")
    print(f"  - Dates récentes: {RECENT_DATE_FORMAT}")
    print(f"  - Autres dates: {STANDARD_DATE}\n")

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
                print(f"✅ {md_file.relative_to(root_dir)}")
                for change in changes[:3]:
                    print(f"   {change}")
        except Exception as e:
            print(f"❌ Erreur {md_file}: {e}")

    print(f"\n✅ {len(files_changed)} fichiers modifiés ({total_changes} changements)")


if __name__ == "__main__":
    main()
