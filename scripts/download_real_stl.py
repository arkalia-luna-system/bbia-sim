#!/usr/bin/env python3
"""
Script pour télécharger les VRAIS fichiers STL du repo officiel Reachy Mini
Résout le problème des pointeurs Git LFS
"""

import os
from pathlib import Path

import requests

# Liste des STL essentiels pour le robot Reachy Mini
ESSENTIAL_STL_FILES = [
    "body_down_3dprint.stl",
    "body_top_3dprint.stl",
    "head_front_3dprint.stl",
    "head_back_3dprint.stl",
    "mp01062_stewart_arm_3.stl",
    "stewart_link_rod.stl",
    "stewart_tricap_3dprint.stl",
    "stewart_main_plate_3dprint.stl",
    "stewart_link_ball.stl",
    "stewart_link_ball__2.stl",
    "head_mic_3dprint.stl",
    "body_foot_3dprint.stl",
    "body_turning_3dprint.stl",
]


def download_real_stl():
    """Télécharge les vrais fichiers STL depuis le repo officiel"""

    # Créer le dossier de destination
    assets_dir = Path("src/bbia_sim/sim/assets/reachy_official")
    assets_dir.mkdir(parents=True, exist_ok=True)

    # URL de base du repo officiel
    base_url = "https://github.com/pollen-robotics/reachy_mini/raw/main/src/reachy_mini/descriptions/reachy_mini/mjcf/assets/"

    print("🚀 Téléchargement des VRAIS fichiers STL...")
    print("=" * 50)

    success_count = 0
    total_count = len(ESSENTIAL_STL_FILES)

    for stl_file in ESSENTIAL_STL_FILES:
        print(f"📥 Téléchargement de {stl_file}...")

        try:
            # Télécharger le fichier
            response = requests.get(base_url + stl_file, timeout=30)

            if response.status_code == 200:
                # Sauvegarder le fichier
                file_path = assets_dir / stl_file
                with open(file_path, "wb") as f:
                    f.write(response.content)

                # Vérifier la taille
                file_size = os.path.getsize(file_path)

                if file_size > 1000:  # Vrai fichier STL
                    print(f"✅ {stl_file}: {file_size:,} bytes - SUCCÈS")
                    success_count += 1
                else:
                    print(
                        f"❌ {stl_file}: {file_size} bytes - TROP PETIT (pointeur Git LFS)"
                    )
                    os.remove(file_path)  # Supprimer le fichier corrompu

            else:
                print(f"❌ {stl_file}: Erreur HTTP {response.status_code}")

        except Exception as e:
            print(f"❌ {stl_file}: Erreur - {e}")

    print("\n" + "=" * 50)
    print(
        f"📊 Résultat: {success_count}/{total_count} fichiers téléchargés avec succès"
    )

    if success_count == total_count:
        print("🎉 TOUS les fichiers STL ont été téléchargés correctement !")
        return True
    else:
        print("⚠️  Certains fichiers n'ont pas pu être téléchargés")
        return False


def verify_stl_quality():
    """Vérifie la qualité des fichiers STL téléchargés"""

    assets_dir = Path("src/bbia_sim/sim/assets/reachy_official")

    print("\n🔍 Vérification de la qualité des STL...")
    print("=" * 50)

    stl_files = list(assets_dir.glob("*.stl"))

    valid_count = 0
    for stl_file in stl_files:
        file_size = stl_file.stat().st_size

        # Lire les premiers bytes pour vérifier le format
        with open(stl_file, "rb") as f:
            header = f.read(100)

        # Vérifier si c'est un vrai STL binaire ou ASCII
        is_valid = file_size > 1000 and (  # Taille suffisante
            b"solid" in header or b"\x00" in header  # STL ASCII
        )  # STL binaire

        if is_valid:
            print(f"✅ {stl_file.name}: {file_size:,} bytes - VALIDE")
            valid_count += 1
        else:
            print(f"❌ {stl_file.name}: {file_size} bytes - CORROMPU")

    print(f"\n📊 {valid_count}/{len(stl_files)} fichiers STL valides")
    return valid_count == len(stl_files)


if __name__ == "__main__":
    print("🛠️  Script de téléchargement des STL officiels Reachy Mini")
    print("=" * 60)

    # Télécharger les fichiers
    success = download_real_stl()

    if success:
        # Vérifier la qualité
        verify_stl_quality()
        print("\n🎯 Prochaine étape: Tester le modèle avec les vrais STL")
    else:
        print("\n❌ Échec du téléchargement - Vérifiez votre connexion")
