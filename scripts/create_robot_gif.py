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
    
    # Filtrer les captures du 27 octobre (récentes)
    recent_captures = [m for m in matches if "2025-10-27" in m]
    
    if len(recent_captures) < 2:
        print("❌ Pas assez d'images récentes!")
        return
    
    # Trier par date pour avoir l'ordre chronologique
    recent_captures.sort(key=os.path.getmtime)
    
    print(f"📸 Capture récentes: {len(recent_captures)}")
    for i, img_path in enumerate(recent_captures, 1):
        print(f"   {i}. {os.path.basename(img_path)}")
    print()

    # Charger toutes les images
    print("🔄 Chargement images...")
    images_loaded = [Image.open(img_path) for img_path in recent_captures]

    # Ajuster tailles - garder résolution plus élevée pour meilleure qualité
    target_width = 700  # Résolution cible pour bonne qualité
    target_height = 600
    
    images_resized = [
        img.resize((target_width, target_height), Image.Resampling.LANCZOS)
        for img in images_loaded
    ]

    print(f"✅ {len(images_resized)} images chargées: {target_width}x{target_height}")
    print()

    # Créer GIF
    output_path = "assets/images/robot_animation.gif"
    print(f"🎬 Création GIF: {output_path}")

    # Doubler chaque image pour animer plus lentement
    images_animated = []
    for img in images_resized:
        images_animated.append(img)
        images_animated.append(img)  # Duplicate for slower animation

    images_animated[0].save(
        output_path,
        save_all=True,
        append_images=images_animated[1:],
        duration=1200,  # 1.2 secondes par image
        loop=0,
        optimize=False,  # Désactiver optimisation pour meilleure qualité
        quality=95,  # Haute qualité
    )

    print("✅ GIF créée!")
    print(f"📁 Fichier: {output_path}")
    print(f"📊 Taille: {os.path.getsize(output_path) / 1024:.1f} KB")
    print()
    print("=" * 70)


if __name__ == "__main__":
    create_robot_gif()
