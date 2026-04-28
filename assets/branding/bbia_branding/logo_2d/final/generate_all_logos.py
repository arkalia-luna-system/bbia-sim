#!/usr/bin/env python3
"""
Script pour générer tous les logos manquants
Utilise le script de calques pour créer toutes les déclinaisons
"""
import os
import sys
import subprocess
from create_horizontal_logo import get_all_layers, extract_path_bounds
import xml.etree.ElementTree as ET


def check_missing_logos():
    """Vérifie quels logos manquent"""
    required = {
        "mark_only": {
            "svg": "bbia_mark_only_v2_SOURCE.svg",
            "png": "bbia_mark_only_v2.png",
            "png_512": "bbia_mark_only_512x512.png",
            "favicon": "bbia_favicon_32x32.png",
        },
        "vertical": {
            "svg": "bbia_logo_vertical_v2_SOURCE.svg",
            "png": "bbia_logo_vertical_v2.png",
        },
        "horizontal": {
            "svg": "bbia_logo_horizontal_SOURCE.svg",
            "png": "bbia_logo_horizontal.png",
        },
    }

    missing = []
    existing = []

    for category, files in required.items():
        for file_type, filename in files.items():
            if os.path.exists(filename):
                existing.append(f"✅ {filename}")
            else:
                missing.append(f"❌ {filename} ({category} - {file_type})")

    return missing, existing


def create_missing_logos():
    """Crée tous les logos manquants"""
    print("🔍 Vérification des logos existants...")
    missing, existing = check_missing_logos()

    print("\n📋 LOGOS EXISTANTS :")
    for item in existing:
        print(f"   {item}")

    print("\n📋 LOGOS MANQUANTS :")
    if not missing:
        print("   ✅ Tous les logos sont présents !")
        return True

    for item in missing:
        print(f"   {item}")

    # Vérifier si le logo horizontal PNG manque
    if not os.path.exists("bbia_logo_horizontal.png"):
        print("\n📤 Création du logo horizontal PNG...")
        try:
            # Utiliser le script existant
            result = subprocess.run(
                [sys.executable, "create_horizontal_logo.py"],
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                print("✅ Logo horizontal PNG créé")
            else:
                print(f"⚠️  Erreur : {result.stderr}")
        except Exception as e:
            print(f"❌ Erreur : {e}")

    # Vérifier les autres fichiers manquants
    print("\n💡 Pour créer les autres logos manquants :")
    print("   • Ouvrir les SVG dans Inkscape")
    print("   • Exporter en PNG avec les bonnes dimensions")

    return len(missing) == 0


def organize_files():
    """Organise les fichiers dans la bonne structure"""
    print("\n📁 Organisation des fichiers...")

    # Créer dossiers si nécessaire
    dirs = ["versions_anciennes", "svg_anciens"]
    for d in dirs:
        if not os.path.exists(d):
            os.makedirs(d)
            print(f"   ✅ Dossier créé : {d}")

    # Déplacer les fichiers ORIGINAL
    original_files = [f for f in os.listdir(".") if f.endswith("_ORIGINAL.png")]
    if original_files:
        print(f"\n📦 Déplacement de {len(original_files)} fichier(s) ORIGINAL...")
        for f in original_files:
            dest = os.path.join("versions_anciennes", f)
            if not os.path.exists(dest):
                os.rename(f, dest)
                print(f"   ✅ {f} → versions_anciennes/")
            else:
                print(f"   ⚠️  {dest} existe déjà")


if __name__ == "__main__":
    print("=" * 70)
    print("🎨 GÉNÉRATION DE TOUS LES LOGOS")
    print("=" * 70)

    # Organiser les fichiers
    organize_files()

    # Vérifier et créer les logos manquants
    all_created = create_missing_logos()

    print("\n" + "=" * 70)
    if all_created:
        print("✅ TOUS LES LOGOS SONT PRÉSENTS !")
    else:
        print("⚠️  Certains logos manquent encore")
    print("=" * 70)
