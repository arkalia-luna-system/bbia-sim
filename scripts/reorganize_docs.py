#!/usr/bin/env python3
"""Script de réorganisation de la documentation BBIA-SIM

Réorganise la documentation selon une structure professionnelle claire.
"""

import os
import shutil
from pathlib import Path
from typing import Dict, List, Tuple

# Configuration
DOCS_ROOT = Path(__file__).parent.parent / "docs"
ARCHIVE_NETTOYAGE = DOCS_ROOT / "archive" / "nettoyage-2025"

# Mapping des fichiers à déplacer
FILES_TO_MOVE: Dict[str, str] = {
    # Fichiers historiques → archive
    "BILAN_FINAL_DOCUMENTATION.md": "archive/nettoyage-2025/",
    "RAPPORT_NETTOYAGE_FINAL.md": "archive/nettoyage-2025/",
    "STATUT_NETTOYAGE_FINAL.md": "archive/nettoyage-2025/",
    "ANALYSE_DOUBLONS_MD.md": "archive/nettoyage-2025/",
    "ANALYSE_DOUBLONS_COMPLETE.md": "archive/nettoyage-2025/",
    "FICHIERS_MD_A_SUPPRIMER.md": "archive/nettoyage-2025/",
    
    # Fichiers de tâches → archive/tasks
    "A_FAIRE_RESTANT.md": "archive/tasks/",
    
    # FAQ → getting-started
    "FAQ.md": "getting-started/troubleshooting.md",
    
    # Guides système → development
    "GUIDE_SYSTEME_TESTS.md": "development/testing.md",
    
    # Status → reference
    "status.md": "reference/project-status.md",
    
    # GOOD_FIRST_ISSUES → getting-started
    "GOOD_FIRST_ISSUES.md": "getting-started/contributing.md",
}

# Dossiers à renommer/déplacer
FOLDERS_TO_MOVE: Dict[str, str] = {
    "references": "reference",  # Renommer en singulier
}

# Dossiers à consolider
CONSOLIDATIONS: Dict[str, List[str]] = {
    "development": [
        "architecture/",
        "api/",
        "guides_techniques/",  # Fusionner dans development/
    ],
    "quality": [
        "conformite/",  # → quality/compliance/
        "qualite/",      # → quality/validation/
        "audit/",       # → quality/audits/
    ],
    "hardware": [
        "reachy/",      # → hardware/reachy-mini/
        "mouvements/",  # → hardware/
        "robot/",       # → hardware/
    ],
    "deployment": [
        "deploy/",      # → deployment/
        "ci/",          # → deployment/ci-cd/
    ],
}

def ensure_dirs():
    """Crée les nouveaux dossiers nécessaires."""
    new_dirs = [
        "getting-started",
        "development",
        "development/architecture",
        "development/api",
        "quality",
        "quality/compliance",
        "quality/audits",
        "quality/checklists",
        "hardware",
        "hardware/reachy-mini",
        "deployment",
        "archive/nettoyage-2025",
        "archive/tasks",
    ]
    
    for dir_path in new_dirs:
        full_path = DOCS_ROOT / dir_path
        full_path.mkdir(parents=True, exist_ok=True)
        print(f"✅ Créé: {dir_path}/")

def move_files():
    """Déplace les fichiers selon le mapping."""
    moved = []
    errors = []
    
    for source_file, target_path in FILES_TO_MOVE.items():
        source = DOCS_ROOT / source_file
        target_dir = DOCS_ROOT / target_path
        
        if not source.exists():
            print(f"⚠️  Fichier non trouvé: {source_file}")
            continue
        
        # Si target_path est un fichier (pas un dossier)
        if target_path.endswith(".md"):
            target = DOCS_ROOT / target_path
            target.parent.mkdir(parents=True, exist_ok=True)
        else:
            target = target_dir / source_file
            target_dir.mkdir(parents=True, exist_ok=True)
        
        try:
            shutil.move(str(source), str(target))
            moved.append((source_file, target_path))
            print(f"✅ Déplacé: {source_file} → {target_path}")
        except Exception as e:
            errors.append((source_file, str(e)))
            print(f"❌ Erreur: {source_file} - {e}")
    
    return moved, errors

def rename_folders():
    """Renomme les dossiers."""
    for old_name, new_name in FOLDERS_TO_MOVE.items():
        old_path = DOCS_ROOT / old_name
        new_path = DOCS_ROOT / new_name
        
        if old_path.exists() and not new_path.exists():
            try:
                shutil.move(str(old_path), str(new_path))
                print(f"✅ Renommé: {old_name}/ → {new_name}/")
            except Exception as e:
                print(f"❌ Erreur renommage {old_name}: {e}")

def main():
    """Fonction principale."""
    print("🔧 Réorganisation de la documentation BBIA-SIM\n")
    print("=" * 60)
    
    # Vérifier qu'on est dans le bon répertoire
    if not DOCS_ROOT.exists():
        print(f"❌ Erreur: {DOCS_ROOT} n'existe pas")
        return 1
    
    print(f"📁 Répertoire docs: {DOCS_ROOT}\n")
    
    # Créer les nouveaux dossiers
    print("📂 Création des nouveaux dossiers...")
    ensure_dirs()
    print()
    
    # Déplacer les fichiers
    print("📦 Déplacement des fichiers...")
    moved, errors = move_files()
    print(f"\n✅ {len(moved)} fichiers déplacés")
    if errors:
        print(f"❌ {len(errors)} erreurs")
    print()
    
    # Renommer les dossiers
    print("🔄 Renommage des dossiers...")
    rename_folders()
    print()
    
    print("=" * 60)
    print("✅ Réorganisation terminée!")
    print("\n⚠️  IMPORTANT: Vérifiez les liens internes dans les fichiers")
    print("   et mettez à jour les index si nécessaire.")
    
    return 0

if __name__ == "__main__":
    exit(main())

