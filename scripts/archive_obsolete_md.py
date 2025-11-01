#!/usr/bin/env python3
"""Script pour archiver les fichiers MD obsolètes avec note d'archive."""

import os
import shutil
from pathlib import Path

# Fichiers obsolètes à archiver dans docs/audit/
OBSOLETE_AUDIT_FILES = [
    # Corrections terminées (versions intermédiaires remplacées)
    "docs/audit/CORRECTIONS_FINALES_ANTENNES_V2.md",
    "docs/audit/CORRECTIONS_FINALES_V3_ULTIME.md",
    "docs/audit/CORRECTIONS_ANTENNES_COMPLETES.md",
    "docs/audit/CORRECTION_ANTENNES_ANIMABLES.md",
    "docs/audit/VÉRIFICATION_FINALE_ANTENNES.md",
    "docs/audit/RÉSUMÉ_FINAL_CORRECTIONS_ANTENNES.md",
    "docs/audit/CORRECTIONS_DOCUMENTATION_MD_APPLIQUEES.md",
    "docs/audit/CORRECTIONS_APPLIQUEES_2025_01_31.md",
    "docs/audit/CORRECTIONS_FINALES_2025_01_31.md",
    
    # Bilans finaux terminés
    "docs/audit/BILAN_FINAL_CORRECTIONS_TESTS.md",
    "docs/audit/BILAN_FINAL_TESTS.md",
    "docs/audit/BILAN_COMPLET_TESTS.md",
    "docs/audit/RESUME_FINAL_100_POURCENT.md",
    "docs/audit/RESUME_FINAL_TESTS.md",
    "docs/audit/RESUME_TESTS_COMPLET.md",
    "docs/audit/RESUME_TESTS_FINAUX.md",
    "docs/audit/TESTS_CREATION_COMPLETE.md",
    "docs/audit/TESTS_MANQUANTS_CREES.md",
    "docs/audit/MOCK_ROBOT_TESTS_2025_01_31.md",
    
    # Vérifications terminées
    "docs/audit/VERIFICATION_AUDIT_STRICT_COMPLETE.md",
    "docs/audit/VERIFICATION_COMPLETE_AUDIT.md",
    "docs/audit/VERIFICATION_FINALE_COMPLETE.md",
    "docs/audit/VERIFICATION_FINALE_NOV2025.md",
    "docs/audit/VERIFICATION_MARKDOWN_COMPLETE_2025.md",
    "docs/audit/VALIDATION_FINALE_COMPLETE.md",
    
    # Optimisations terminées
    "docs/audit/OPTIMISATIONS_TESTS_2025_01_31.md",
    
    # Réorganisations terminées
    "docs/audit/RESUME_REORGANISATION_TESTS.md",
    "docs/audit/RESUME_CONSOLIDATION_SCRIPTS_AUDIT.md",
    
    # Documents remplacés par des versions plus récentes
    "docs/audit/BILAN_CORRECTIONS_PARTIELLE.md",
    "docs/audit/BILAN_FINAL_MISE_A_JOUR.md",
    "docs/audit/RESUME_BILAN_FINAL.md",
    "docs/audit/STATUS_FINAL_COMPLET.md",
    "docs/audit/STATUT_FINAL_AUDIT.md",
    
    # Analyses terminées
    "docs/audit/ANALYSE_COMPLETE_ETAT_PROJET_2025_01_31.md",
    "docs/audit/ANALYSE_TESTS_EXEMPLES_MANQUANTS.md",
    "docs/audit/ANALYSE_MEDIUM_PRIORITY.md",
    "docs/audit/ANALYSE_DIFFERENCES_SIGNATURES_METHODES.md",
    "docs/audit/DIFFERENCES_SUBTILES_DETECTEES.md",
    "docs/audit/RAPPORT_EXEMPLES_REACHY_MINI.md",
    
    # Corrections en cours (terminées maintenant)
    "docs/audit/CORRECTIONS_EN_COURS.md",
    "docs/audit/PLAN_CORRECTIONS_METHODIQUE.md",
    "docs/audit/CE_QUI_RESTE_VRAIMENT_A_FAIRE.md",
    "docs/audit/PROCHAINES_ETAPES_AUDIT.md",
    
    # Notes d'archive
    "docs/audit/NOTE_ARCHIVES_ANTENNES.md",
    
    # Documents intermédiaires remplacés
    "docs/audit/AUDIT_EXHAUSTIF_COMPLET_2025.md",
    "docs/audit/AUDIT_COMPLET_MODIFICATIONS_2025.md",
    "docs/audit/AUDIT_COMPLET_PROJET_2025.md",
    "docs/audit/AUDIT_CRITIQUE_REVISITE_2025.md",
    "docs/audit/AUDIT_TESTS_ORGANISATION.md",
    
    # Corrections terminées dans docs/corrections/
    "docs/corrections/TOUTES_DEMOS_CORRIGEES.md",
    "docs/corrections/CORRECTIONS_ERREURS_COMPLETE_2025.md",
    "docs/corrections/DEMO_3D_CORRIGEE.md",
    "docs/corrections/ERROR_CORRECTION_REPORT.md",
    
    # Architecture obsolète
    "docs/architecture/ARCHITECTURE.md",
    
    # Versions intermédiaires remplacées par FINAL
    "docs/AUDIT_DOCUMENTATION_COMPLETE.md",  # Remplacé par AUDIT_DOCUMENTATION_FINAL.md
    "docs/conformite/AUDIT_CONFORMITE_COMPLETE.md",  # Remplacé par AUDIT_CONFORMITE_FINAL.md
    "docs/conformite/CHECKLIST_AUDIT_SYSTEMATIQUE.md",  # Remplacé par CHECKLIST_AUDIT_SYSTEMATIQUE_FINAL.md
    "docs/conformite/AUDIT_EXHAUSTIF_FINAL_2025_01_31.md",  # Version intermédiaire
]

ARCHIVE_HEADER = """---
**⚠️ ARCHIVE - DOCUMENT HISTORIQUE ⚠️**

Ce document a été archivé car il est devenu obsolète ou a été remplacé par une version plus récente.
Il est conservé à des fins de référence historique uniquement.

**Date d'archivage** : octobre 2025
**Raison** : Document terminé/obsolète/remplacé
---

"""

def add_archive_header(file_path):
    """Ajoute l'en-tête d'archive au début du fichier."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Vérifier si l'en-tête existe déjà
    if content.startswith('---\n**⚠️ ARCHIVE'):
        return False
    
    # Ajouter l'en-tête
    new_content = ARCHIVE_HEADER + content
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    return True

def archive_files():
    """Archive les fichiers obsolètes."""
    base_dir = Path('/Volumes/T7/bbia-reachy-sim')
    audits_archive_dir = base_dir / 'docs' / 'archives' / 'audits_termines'
    corrections_archive_dir = base_dir / 'docs' / 'archives' / 'corrections_terminees'
    architecture_archive_dir = base_dir / 'docs' / 'archives' / 'organisation'
    
    audits_archive_dir.mkdir(parents=True, exist_ok=True)
    corrections_archive_dir.mkdir(parents=True, exist_ok=True)
    architecture_archive_dir.mkdir(parents=True, exist_ok=True)
    
    archived_count = 0
    skipped_count = 0
    
    for file_path_str in OBSOLETE_AUDIT_FILES:
        file_path = base_dir / file_path_str
        
        if not file_path.exists():
            print(f"⚠️  Fichier introuvable: {file_path_str}")
            skipped_count += 1
            continue
        
        # Déterminer le dossier d'archive selon le type de fichier
        if 'corrections/' in file_path_str:
            archive_dir = corrections_archive_dir
        elif 'architecture/' in file_path_str:
            archive_dir = architecture_archive_dir
        elif 'conformite/' in file_path_str or file_path_str.startswith('docs/AUDIT_'):
            archive_dir = audits_archive_dir  # Audits de conformité
        else:
            archive_dir = audits_archive_dir
        
        # Créer le nom de destination
        filename = file_path.name
        dest_path = archive_dir / filename
        
        # Si le fichier existe déjà dans l'archive, ajouter un suffixe
        if dest_path.exists():
            stem = file_path.stem
            suffix = file_path.suffix
            counter = 1
            while dest_path.exists():
                dest_path = archive_dir / f"{stem}_v{counter}{suffix}"
                counter += 1
        
        # Ajouter l'en-tête d'archive
        add_archive_header(file_path)
        
        # Déplacer le fichier
        shutil.move(str(file_path), str(dest_path))
        print(f"✅ Archivé: {file_path_str} → {dest_path.relative_to(base_dir)}")
        archived_count += 1
    
    print(f"\n📦 Résumé:")
    print(f"   ✅ {archived_count} fichiers archivés")
    print(f"   ⚠️  {skipped_count} fichiers introuvables")

if __name__ == '__main__':
    archive_files()

