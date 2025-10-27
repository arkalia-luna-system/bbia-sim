#!/usr/bin/env python3
"""
Crée une GIF animée avec 2 positions du robot
1. Neutre
2. Tête inclinée vers haut-gauche
"""

import glob
import os

from PIL import Image


def create_robot_gif():
    """Crée une GIF avec les 2 positions."""

    print("=" * 70)
    print("🎬 CRÉATION GIF ROBOT")
    print("=" * 70)

    # Rechercher les images avec glob
    matches = glob.glob("assets/images/*Capture*.png")
    print(f"🔍 Images trouvées: {len(matches)}")

    if len(matches) < 2:
        print("❌ Pas assez d'images!")
        return

    # Prendre les 2 plus récentes
    matches.sort(key=os.path.getmtime)
    img1 = matches[0]  # Ancienne: Neutre
    img2 = matches[1]  # Nouvelle: Tête inclinée

    print(f"📸 Image 1 (neutre): {os.path.basename(img1)}")
    print(f"📸 Image 2 (inclinée): {os.path.basename(img2)}")
    print()

    print("📸 Image 1: Position neutre")
    print("📸 Image 2: Tête inclinée haut-gauche")
    print()

    # Charger images
    print("🔄 Chargement images...")
    im1 = Image.open(img1)
    im2 = Image.open(img2)

    # Ajuster tailles (prendre la plus petite)
    min_width = min(im1.width, im2.width)
    min_height = min(im1.height, im2.height)

    im1 = im1.resize((min_width, min_height), Image.Resampling.LANCZOS)
    im2 = im2.resize((min_width, min_height), Image.Resampling.LANCZOS)

    print(f"✅ Images chargées: {min_width}x{min_height}")
    print()

    # Créer GIF
    output_path = "assets/images/robot_animation.gif"
    print(f"🎬 Création GIF: {output_path}")

    # Doubler chaque image pour animer plus lentement
    images = [im1, im1, im2, im2]

    images[0].save(
        output_path,
        save_all=True,
        append_images=images[1:],
        duration=1500,  # 1.5 secondes par image
        loop=0,
    )

    print("✅ GIF créée!")
    print(f"📁 Fichier: {output_path}")
    print(f"📊 Taille: {os.path.getsize(output_path) / 1024:.1f} KB")
    print()
    print("=" * 70)


if __name__ == "__main__":
    create_robot_gif()
