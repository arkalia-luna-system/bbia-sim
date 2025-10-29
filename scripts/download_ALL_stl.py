#!/usr/bin/env python3
"""Script pour télécharger TOUS les fichiers STL du repo officiel Reachy Mini
Résout définitivement le problème des assets manquants.
"""

import logging
import os
from pathlib import Path

import requests


def get_all_stl_files():
    """Récupère la liste de TOUS les fichiers STL du repo officiel."""
    # URL de l'API GitHub pour le dossier assets
    api_url = "https://api.github.com/repos/pollen-robotics/reachy_mini/contents/src/reachy_mini/descriptions/reachy_mini/mjcf/assets"

    try:
        response = requests.get(api_url, timeout=30)
        response.raise_for_status()

        files = response.json()
        stl_files = [f["name"] for f in files if f["name"].endswith(".stl")]

        return stl_files

    except Exception as exc:
        logging.warning("Échec récupération liste STL: %s", exc)
        return []


def download_all_stl():
    """Télécharge TOUS les fichiers STL depuis le repo officiel."""
    # Créer le dossier de destination
    assets_dir = Path("src/bbia_sim/sim/assets/reachy_official")
    assets_dir.mkdir(parents=True, exist_ok=True)

    # Récupérer la liste des STL
    stl_files = get_all_stl_files()

    if not stl_files:
        return False

    # URL de base du repo officiel
    base_url = "https://github.com/pollen-robotics/reachy_mini/raw/main/src/reachy_mini/descriptions/reachy_mini/mjcf/assets/"

    success_count = 0
    total_count = len(stl_files)

    for _i, stl_file in enumerate(stl_files, 1):

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
                    success_count += 1
                else:
                    os.remove(file_path)  # Supprimer le fichier corrompu

            else:
                logging.warning(
                    "Téléchargement échoué pour %s: HTTP %s",
                    stl_file,
                    response.status_code,
                )

        except Exception as exc:
            logging.warning("Erreur lors du téléchargement de %s: %s", stl_file, exc)

    return success_count == total_count


def verify_all_stl():
    """Vérifie la qualité de TOUS les fichiers STL téléchargés."""
    assets_dir = Path("src/bbia_sim/sim/assets/reachy_official")

    stl_files = list(assets_dir.glob("*.stl"))

    valid_count = 0
    corrupted_count = 0

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
            valid_count += 1
        else:
            corrupted_count += 1

    return valid_count == len(stl_files)


if __name__ == "__main__":

    # Télécharger TOUS les fichiers
    success = download_all_stl()

    if success:
        # Vérifier la qualité
        verify_all_stl()
    else:
        logging.warning("Téléchargement des STL incomplet")
