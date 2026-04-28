#!/usr/bin/env python3
"""
Script pour organiser les fichiers anciens et dupliqués
Déplace les fichiers dans les dossiers d'archive appropriés
"""
import os
import shutil
from pathlib import Path

# Fichiers à archiver
FILES_TO_ARCHIVE = {
    "versions_anciennes": [
        "bbia_logo_1024x1024.png",  # Ancien format
        "bbia_logo_optimiser.png",  # Ancien format optimisé
    ],
}

# Fichiers à vérifier (peuvent être supprimés si dupliqués)
FILES_TO_CHECK = [
    "bbia_logo_1024x1024.png",
    "bbia_logo_optimiser.png",
]


def organize_files():
    """Organise les fichiers dans les dossiers d'archive"""
    print("=" * 70)
    print("📁 ORGANISATION DES FICHIERS")
    print("=" * 70)

    current_dir = Path(".")
    moved_count = 0
    skipped_count = 0

    # Créer les dossiers d'archive s'ils n'existent pas
    for archive_dir in FILES_TO_ARCHIVE.keys():
        archive_path = current_dir / archive_dir
        if not archive_path.exists():
            archive_path.mkdir(parents=True, exist_ok=True)
            print(f"📁 Dossier créé: {archive_dir}/")

    # Déplacer les fichiers
    for archive_dir, files in FILES_TO_ARCHIVE.items():
        print(f"\n📦 Dossier: {archive_dir}/")
        for filename in files:
            source = current_dir / filename
            dest = current_dir / archive_dir / filename

            if source.exists():
                if dest.exists():
                    print(f"   ⚠️  {filename} existe déjà dans {archive_dir}/")
                    skipped_count += 1
                else:
                    try:
                        shutil.move(str(source), str(dest))
                        print(f"   ✅ {filename} → {archive_dir}/")
                        moved_count += 1
                    except Exception as e:
                        print(f"   ❌ Erreur lors du déplacement de {filename}: {e}")
            else:
                print(f"   ℹ️  {filename} n'existe pas (déjà déplacé ou supprimé)")

    # Vérifier les fichiers dupliqués
    print("\n🔍 Vérification des fichiers dupliqués:")
    for filename in FILES_TO_CHECK:
        file_path = current_dir / filename
        if file_path.exists():
            print(f"   ⚠️  {filename} existe encore dans le dossier principal")
            print("      → Considérer le déplacer dans versions_anciennes/")
        else:
            print(f"   ✅ {filename} n'est plus dans le dossier principal")

    print("\n✅ Résumé:")
    print(f"   • {moved_count} fichier(s) déplacé(s)")
    print(f"   • {skipped_count} fichier(s) ignoré(s) (déjà présent)")

    # Lister la structure finale
    print("\n📂 Structure finale:")
    print("   final/")
    print("   ├── logos actuels (v2)")
    print("   ├── versions_anciennes/")
    print("   │   └── fichiers archivés")
    print("   └── svg_anciens/")
    print("       └── SVG anciens")


if __name__ == "__main__":
    organize_files()
