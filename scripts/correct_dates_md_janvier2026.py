#!/usr/bin/env python3
"""
Correction automatique des dates obsolètes dans les fichiers MD - 26 Janvier 2026
Met à jour uniquement les dates "Dernière mise à jour" qui sont obsolètes
Préserve les dates historiques (réception robot, montage, etc.)
"""

import re
from pathlib import Path
from typing import List, Tuple

# Configuration
CURRENT_DATE = "26 Janvier 2026"
ROOT_DIR = Path(__file__).parent.parent

# Dates historiques à préserver (ne pas modifier)
HISTORICAL_DATES = [
    "18 Décembre 2025",  # Réception robot
    "20 Décembre 2025",  # Montage robot
    "22 Décembre 2025",  # Tests initiaux
    "17 Janvier 2026",   # Réception moteurs
    "21 Janvier 2026",   # Vérification QC
    "20 Janvier 2026",   # Analyse repo
    "6 Janvier 2026",    # Email envoyé
    "7 Janvier 2026",    # Dates historiques
    "8 Janvier 2026",
    "14 Janvier 2026",
    "15 Janvier 2026",
    "24 Novembre 2025",  # Dates historiques
    "26 Novembre 2025",
    "7 Décembre 2025",
    "8 Décembre 2025",   # Peut être historique selon contexte
]

# Patterns pour "Dernière mise à jour" (à mettre à jour)
UPDATE_PATTERNS = [
    (r"(?i)(\*\*)?Dernière mise à jour\s*[:*]\s*(\*\*)?\s*(\d{1,2}\s+\w+\s+2025)", 
     r"\1Dernière mise à jour\2 : \2" + CURRENT_DATE),
    (r"(?i)(\*\*)?dernière mise à jour\s*[:*]\s*(\*\*)?\s*(\d{1,2}\s+\w+\s+2025)",
     r"\1Dernière mise à jour\2 : \2" + CURRENT_DATE),
    (r"(?i)(\*\*)?Dernière mise à jour\s*[:*]\s*(\*\*)?\s*(\d{1,2}\s+\w+\s+2025)",
     r"\1Dernière mise à jour\2 : \2" + CURRENT_DATE),
    # Pattern plus simple
    (r"(?i)(Dernière mise à jour\s*[:*]\s*)(\d{1,2}\s+\w+\s+2025)",
     r"\1" + CURRENT_DATE),
]


def is_historical_date(date_str: str) -> bool:
    """Vérifie si une date est historique et doit être préservée."""
    date_lower = date_str.lower()
    for hist_date in HISTORICAL_DATES:
        if hist_date.lower() in date_lower:
            return True
    return False


def should_update_date(line: str, date_str: str) -> bool:
    """Détermine si une date doit être mise à jour."""
    # Ne pas mettre à jour si c'est une date historique
    if is_historical_date(date_str):
        return False
    
    # Mettre à jour seulement les "Dernière mise à jour"
    if "dernière mise à jour" in line.lower():
        # Vérifier que ce n'est pas dans un contexte historique
        # (ex: "Mise à jour 8 Décembre 2025" dans un historique)
        if re.search(r"(?i)(mise à jour|update)\s+\d{1,2}\s+\w+\s+2025", line):
            # Si c'est dans un contexte de liste ou historique, ne pas modifier
            if any(keyword in line.lower() for keyword in ["historique", "changelog", "release", "version"]):
                return False
        return True
    
    return False


def correct_file(file_path: Path, dry_run: bool = True) -> Tuple[int, List[str]]:
    """Corrige les dates obsolètes dans un fichier."""
    try:
        content = file_path.read_text(encoding="utf-8")
        original_content = content
        changes = []
        
        lines = content.split("\n")
        modified_lines = []
        
        for i, line in enumerate(lines, 1):
            original_line = line
            
            # Chercher les dates dans la ligne
            for pattern, replacement in UPDATE_PATTERNS:
                matches = list(re.finditer(pattern, line))
                for match in reversed(matches):  # Parcourir en sens inverse pour préserver les indices
                    date_str = match.group(2) if len(match.groups()) >= 2 else match.group(0)
                    
                    # Vérifier si on doit mettre à jour
                    if should_update_date(line, date_str):
                        # Remplacer la date
                        new_line = re.sub(pattern, replacement, line, count=1)
                        if new_line != line:
                            line = new_line
                            changes.append(f"Ligne {i}: {date_str} → {CURRENT_DATE}")
            
            modified_lines.append(line)
        
        new_content = "\n".join(modified_lines)
        
        if new_content != original_content:
            if not dry_run:
                file_path.write_text(new_content, encoding="utf-8")
            return len(changes), changes
        
        return 0, []
    
    except Exception as e:
        print(f"⚠️  Erreur traitement {file_path}: {e}")
        return 0, []


def main():
    """Fonction principale."""
    import sys
    
    dry_run = "--apply" not in sys.argv
    
    if dry_run:
        print("🔍 Mode DRY-RUN (pas de modifications)")
        print("   Utilisez --apply pour appliquer les corrections\n")
    else:
        print("✏️  Mode APPLICATION (modifications réelles)\n")
    
    print(f"📅 Mise à jour des dates vers : {CURRENT_DATE}\n")
    
    # Trouver tous les fichiers MD
    md_files = []
    for path in ROOT_DIR.rglob("*.md"):
        parts = path.parts
        if any(
            part.startswith(".")
            or part == "venv"
            or part == "__pycache__"
            or "_archive" in part
            or "_archived" in part
            for part in parts
        ):
            continue
        if path.name.startswith("._"):
            continue
        md_files.append(path)
    
    print(f"📁 {len(md_files)} fichiers MD trouvés\n")
    
    total_changes = 0
    files_modified = []
    
    for file_path in sorted(md_files):
        relative_path = file_path.relative_to(ROOT_DIR)
        num_changes, changes = correct_file(file_path, dry_run=dry_run)
        
        if num_changes > 0:
            total_changes += num_changes
            files_modified.append((relative_path, num_changes, changes))
            print(f"✅ {relative_path}: {num_changes} modification(s)")
            if len(changes) <= 3:
                for change in changes:
                    print(f"   - {change}")
    
    print("\n" + "=" * 80)
    print("📊 RÉSUMÉ")
    print("=" * 80)
    print(f"✅ Fichiers modifiés : {len(files_modified)}")
    print(f"📝 Total modifications : {total_changes}")
    
    if dry_run:
        print("\n💡 Pour appliquer les corrections, relancez avec --apply")
    else:
        print("\n✅ Corrections appliquées avec succès !")


if __name__ == "__main__":
    main()
