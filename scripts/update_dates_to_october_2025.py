#!/usr/bin/env python3
"""Script pour remplacer toutes les dates dans les fichiers .md par des dates d'octobre 2025."""

import re
from pathlib import Path


def replace_dates_to_october_2025(content: str) -> str:
    """Remplace toutes les dates par des dates d'octobre 2025."""

    # Pattern pour dates YYYY-MM-DD
    def replace_yyyy_mm_dd(match):
        date_str = match.group(0)
        year, month, day = date_str.split("-")

        # Si déjà en octobre 2025, garde tel quel
        if year == "2025" and month == "10":
            return date_str

        # Convertit en octobre 2025
        # Garde le jour si valide pour octobre (1-31), sinon utilise le jour modulo 31 + 1
        day_int = int(day)
        if day_int > 31:
            day_int = 31
        elif day_int < 1:
            day_int = 1

        return f"2025-10-{day_int:02d}"

    # Pattern pour dates DD/MM/YYYY
    def replace_dd_mm_yyyy(match):
        date_str = match.group(0)
        parts = date_str.split("/")
        if len(parts) == 3:
            day, month, year = parts
            day_int = int(day)
            if day_int > 31:
                day_int = 31
            elif day_int < 1:
                day_int = 1
            return f"{day_int:02d}/10/2025"
        return date_str

    # Remplace dates format YYYY-MM-DD
    content = re.sub(
        r"\b(20[0-9]{2})-([0-9]{2})-([0-9]{2})\b", replace_yyyy_mm_dd, content
    )

    # Remplace dates format DD/MM/YYYY
    content = re.sub(
        r"\b([0-9]{1,2})/([0-9]{1,2})/(20[0-9]{2})\b", replace_dd_mm_yyyy, content
    )

    return content


def process_md_files(root_dir: str):
    """Traite tous les fichiers .md dans le répertoire."""
    root = Path(root_dir)

    # Exclure les dossiers système et fichiers de métadonnées
    exclude_dirs = {
        "venv",
        "venv-vision",
        "venv-voice",
        "venv-vision-py310",
        "dist",
        "build",
        "__pycache__",
        ".git",
        "htmlcov",
        "src/bbia_sim.egg-info",
        "node_modules",
    }

    md_files = []
    for md_file in root.rglob("*.md"):
        # Ignorer les fichiers de métadonnées macOS
        if md_file.name.startswith("._"):
            continue

        # Ignorer les fichiers dans les dossiers exclus
        if any(excluded in str(md_file) for excluded in exclude_dirs):
            continue

        md_files.append(md_file)

    print(f"Traitement de {len(md_files)} fichiers .md...")

    updated_count = 0
    for md_file in md_files:
        try:
            # Lire le contenu
            with open(md_file, encoding="utf-8", errors="ignore") as f:
                original_content = f.read()

            # Remplacer les dates
            new_content = replace_dates_to_october_2025(original_content)

            # Écrire si changé
            if original_content != new_content:
                with open(md_file, "w", encoding="utf-8") as f:
                    f.write(new_content)
                updated_count += 1
                print(f"✓ Mis à jour: {md_file.relative_to(root)}")

        except Exception as e:
            print(f"✗ Erreur avec {md_file}: {e}")

    print(f"\n✅ Terminé: {updated_count} fichiers mis à jour sur {len(md_files)}")


if __name__ == "__main__":
    root_directory = "/Volumes/T7/bbia-reachy-sim"
    process_md_files(root_directory)
