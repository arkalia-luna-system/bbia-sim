#!/usr/bin/env python3
"""
Script pour améliorer le contraste du favicon
Crée une version plus lisible avec meilleur contraste
"""
import os
from PIL import Image, ImageEnhance, ImageFilter
from pathlib import Path


def improve_favicon_contrast(input_path, output_path):
    """Améliore le contraste du favicon"""
    print(f"📸 Chargement: {input_path}")

    # Charger l'image
    img = Image.open(input_path)

    # Convertir en RGBA si nécessaire
    if img.mode != "RGBA":
        img = img.convert("RGBA")

    print(f"   Taille originale: {img.size}")
    print(f"   Mode: {img.mode}")

    # Créer une version avec meilleur contraste
    # Option 1: Augmenter le contraste
    enhancer = ImageEnhance.Contrast(img)
    img_contrast = enhancer.enhance(1.5)  # Augmenter de 50%

    # Option 2: Augmenter la luminosité
    enhancer_bright = ImageEnhance.Brightness(img_contrast)
    img_bright = enhancer_bright.enhance(1.2)  # Augmenter de 20%

    # Option 3: Renforcer les bords pour plus de netteté
    img_sharp = img_bright.filter(ImageFilter.SHARPEN)

    # Sauvegarder
    img_sharp.save(output_path, "PNG", optimize=True)
    print(f"✅ Favicon amélioré sauvegardé: {output_path}")

    return img_sharp


def create_high_contrast_version(input_path, output_path):
    """Crée une version haute résolution pour mieux voir les détails"""
    img = Image.open(input_path)

    if img.mode != "RGBA":
        img = img.convert("RGBA")

    # Créer une version 128x128 pour mieux voir
    img_large = img.resize((128, 128), Image.Resampling.LANCZOS)

    # Améliorer le contraste
    enhancer = ImageEnhance.Contrast(img_large)
    img_contrast = enhancer.enhance(2.0)  # Doubler le contraste

    # Augmenter la saturation pour plus de couleur
    enhancer_sat = ImageEnhance.Color(img_contrast)
    img_sat = enhancer_sat.enhance(1.3)

    img_sat.save(output_path, "PNG", optimize=True)
    print(f"✅ Version haute résolution créée: {output_path}")

    return img_sat


def create_simplified_favicon(input_path, output_path):
    """Crée une version simplifiée du favicon (sans détails fins)"""
    img = Image.open(input_path)

    if img.mode != "RGBA":
        img = img.convert("RGBA")

    # Redimensionner à 32x32 avec meilleur algorithme
    img_32 = img.resize((32, 32), Image.Resampling.LANCZOS)

    # Augmenter fortement le contraste
    enhancer = ImageEnhance.Contrast(img_32)
    img_contrast = enhancer.enhance(2.5)  # Augmenter de 150%

    # Augmenter la luminosité
    enhancer_bright = ImageEnhance.Brightness(img_contrast)
    img_bright = enhancer_bright.enhance(1.3)  # Augmenter de 30%

    # Renforcer les bords
    img_sharp = img_bright.filter(ImageFilter.SHARPEN)

    # Sauvegarder
    img_sharp.save(output_path, "PNG", optimize=True)
    print(f"✅ Favicon simplifié créé: {output_path}")

    return img_sharp


def main():
    """Fonction principale"""
    print("=" * 70)
    print("🔧 AMÉLIORATION DU FAVICON - CONTRASTE")
    print("=" * 70)

    current_dir = Path(".")
    input_file = current_dir / "bbia_favicon_32x32.png"

    if not input_file.exists():
        print(f"❌ Fichier non trouvé: {input_file}")
        print("   Tentative avec bbia_mark_only_v2.png...")
        input_file = current_dir / "bbia_mark_only_v2.png"
        if not input_file.exists():
            print(f"❌ Fichier non trouvé: {input_file}")
            return

    # Créer les versions améliorées
    print(f"\n📸 Traitement de: {input_file.name}")

    # Version améliorée (remplace l'original)
    output_improved = current_dir / "bbia_favicon_32x32_improved.png"
    improve_favicon_contrast(input_file, output_improved)

    # Version simplifiée (nouvelle version optimale)
    output_simplified = current_dir / "bbia_favicon_32x32_simplified.png"
    create_simplified_favicon(input_file, output_simplified)

    # Version haute résolution pour comparaison
    output_large = current_dir / "bbia_favicon_128x128_comparison.png"
    create_high_contrast_version(input_file, output_large)

    print("\n✅ Résumé:")
    print(f"   • {output_improved.name} - Version améliorée")
    print(f"   • {output_simplified.name} - Version simplifiée (recommandée)")
    print(f"   • {output_large.name} - Version 128x128 pour comparaison")

    print("\n💡 Prochaines étapes:")
    print("   1. Comparer les versions créées")
    print("   2. Choisir la meilleure version")
    print("   3. Remplacer bbia_favicon_32x32.png si nécessaire")
    print("   4. Retester avec open_visual_tests.py")


if __name__ == "__main__":
    main()
