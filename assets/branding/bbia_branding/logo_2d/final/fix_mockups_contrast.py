#!/usr/bin/env python3
"""
Script pour améliorer le contraste des mockups
Recrée les mockups avec meilleur contraste
"""
import os
from PIL import Image, ImageDraw, ImageFont, ImageEnhance
from pathlib import Path

# Couleurs de test (selon COULEURS_OFFICIELLES.md)
COLORS = {
    "fond_clair": "#FFFFFF",  # Blanc
    "fond_sombre": "#020202",  # Noir foncé BBIA (détails)
    "fond_bleu": "#008181",  # Bleu BBIA (fond logo)
    "fond_noir": "#000000",  # Noir pur BBIA (texte)
}

# Taille des mockups
MOCKUP_SIZE = (800, 600)
LOGO_SIZE = (400, 400)


def enhance_logo_contrast(logo_img):
    """Améliore le contraste du logo"""
    # Augmenter le contraste
    enhancer = ImageEnhance.Contrast(logo_img)
    logo_contrast = enhancer.enhance(1.5)

    # Augmenter la luminosité
    enhancer_bright = ImageEnhance.Brightness(logo_contrast)
    logo_bright = enhancer_bright.enhance(1.2)

    return logo_bright


def create_mockup_improved(background_color, logo_path, output_path, label):
    """Crée un mockup amélioré avec meilleur contraste"""
    # Créer l'image de fond
    img = Image.new("RGB", MOCKUP_SIZE, background_color)
    draw = ImageDraw.Draw(img)

    # Charger le logo
    try:
        logo = Image.open(logo_path)
        # Améliorer le contraste du logo
        logo = enhance_logo_contrast(logo)

        # Redimensionner le logo
        logo.thumbnail(LOGO_SIZE, Image.Resampling.LANCZOS)

        # Centrer le logo
        x = (MOCKUP_SIZE[0] - logo.size[0]) // 2
        y = (MOCKUP_SIZE[1] - logo.size[1]) // 2

        # Coller le logo (avec transparence si PNG)
        if logo.mode == "RGBA":
            img.paste(logo, (x, y), logo)
        else:
            img.paste(logo, (x, y))

        # Ajouter un label en bas
        try:
            font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 20)
        except (OSError, IOError):
            font = ImageFont.load_default()

        text = f"Test amélioré: {label} ({background_color})"
        bbox = draw.textbbox((0, 0), text, font=font)
        text_width = bbox[2] - bbox[0]
        text_x = (MOCKUP_SIZE[0] - text_width) // 2
        text_y = MOCKUP_SIZE[1] - 40

        # Couleur du texte selon le fond
        if background_color == "#FFFFFF":
            text_color = "#000000"
        else:
            text_color = "#FFFFFF"

        draw.text((text_x, text_y), text, fill=text_color, font=font)

    except Exception as e:
        print(f"⚠️  Erreur lors du chargement du logo {logo_path}: {e}")
        # Dessiner un rectangle pour indiquer l'emplacement du logo
        x = (MOCKUP_SIZE[0] - LOGO_SIZE[0]) // 2
        y = (MOCKUP_SIZE[1] - LOGO_SIZE[1]) // 2
        draw.rectangle(
            [x, y, x + LOGO_SIZE[0], y + LOGO_SIZE[1]], outline="#CCCCCC", width=2
        )
        draw.text(
            (x + 10, y + 10),
            f"Logo non trouvé:\n{logo_path}",
            fill="#CCCCCC",
            font=ImageFont.load_default(),
        )

    # Sauvegarder
    img.save(output_path, "PNG")
    print(f"✅ Mockup amélioré créé: {output_path}")


def main():
    """Fonction principale"""
    print("=" * 70)
    print("🔧 AMÉLIORATION DES MOCKUPS - CONTRASTE")
    print("=" * 70)

    # Dossier de sortie
    output_dir = Path("tests_visuels_improved")
    if not output_dir.exists():
        output_dir.mkdir()
        print(f"📁 Dossier créé: {output_dir}/")

    # Logo à tester (Mark Only pour les tests)
    # Essayer d'abord le PNG, puis le SVG
    logo_path = Path("bbia_mark_only_v2.png")

    if not logo_path.exists():
        print(f"⚠️  PNG non trouvé: {logo_path}")
        # Utiliser EXACTEMENT le fichier SOURCE
        svg_path = Path("bbia_mark_only_v2_SOURCE.svg")
        if svg_path.exists():
            print(f"   ✅ Utilisation du SVG: {svg_path}")
            # Convertir SVG en PNG temporaire
            try:
                import cairosvg

                logo_path = Path("bbia_mark_only_v2_temp.png")
                with open(svg_path, "rb") as f:
                    svg_data = f.read()
                cairosvg.svg2png(bytestring=svg_data, write_to=str(logo_path))
                print("   ✅ SVG converti en PNG temporaire")
            except ImportError:
                print("   ❌ cairosvg non installé, impossible de convertir SVG")
                return
            except Exception as e:
                print(f"   ❌ Erreur conversion SVG: {e}")
                return
        else:
            print(f"❌ Logo non trouvé (ni PNG ni SVG): {logo_path}")
            return

    print(f"\n📸 Logo utilisé: {logo_path}")
    print(f"📁 Dossier de sortie: {output_dir}/\n")

    # Créer tous les mockups améliorés
    for name, color in COLORS.items():
        output_file = output_dir / f"mockup_{name}_improved.png"
        create_mockup_improved(
            color, logo_path, output_file, name.replace("_", " ").title()
        )

    print(f"\n✅ {len(COLORS)} mockups améliorés créés dans {output_dir}/")
    print("\n📋 Fichiers créés:")
    for name in COLORS.keys():
        print(f"   • mockup_{name}_improved.png")

    print("\n💡 Instructions:")
    print("   1. Comparer les mockups améliorés avec les originaux")
    print("   2. Vérifier si le contraste est meilleur")
    print("   3. Utiliser les versions améliorées si meilleures")


if __name__ == "__main__":
    main()
