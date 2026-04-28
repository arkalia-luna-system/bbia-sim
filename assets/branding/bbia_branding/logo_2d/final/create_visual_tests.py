#!/usr/bin/env python3
"""
Script pour créer des mockups de test visuel du logo BBIA
Génère des images de test sur différents fonds pour vérifier la lisibilité
"""
import os
from PIL import Image, ImageDraw, ImageFont

# Couleurs de test (selon COULEURS_OFFICIELLES.md)
COLORS = {
    "fond_clair": "#FFFFFF",  # Blanc
    "fond_sombre": "#020202",  # Noir foncé BBIA (détails)
    "fond_bleu": "#008181",  # Bleu BBIA (fond logo)
    "fond_noir": "#000000",  # Noir pur BBIA (texte)
}

# Taille des mockups
MOCKUP_SIZE = (800, 600)
LOGO_SIZE = (400, 400)  # Taille du logo dans le mockup


def create_mockup(background_color, logo_path, output_path, label):
    """Crée un mockup avec un fond de couleur et le logo centré"""
    # Créer l'image de fond
    img = Image.new("RGB", MOCKUP_SIZE, background_color)
    draw = ImageDraw.Draw(img)

    # Charger le logo
    try:
        logo = Image.open(logo_path)
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

        text = f"Test: {label} ({background_color})"
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
    print(f"✅ Mockup créé: {output_path}")


def main():
    """Fonction principale"""
    print("=" * 70)
    print("🎨 CRÉATION DES MOCKUPS DE TEST VISUEL")
    print("=" * 70)

    # Dossier de sortie
    output_dir = "tests_visuels"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"📁 Dossier créé: {output_dir}/")

    # Logo à tester - UTILISER UNIQUEMENT LES FICHIERS _SOURCE.svg
    # Priorité : PNG existant > SVG SOURCE (converti avec Inkscape) > favicon
    logo_path = None
    svg_source = "bbia_mark_only_v2_SOURCE.svg"

    # Essayer d'abord le PNG existant
    if os.path.exists("bbia_mark_only_v2.png"):
        logo_path = "bbia_mark_only_v2.png"
        print(f"   ✅ Utilisation du PNG existant: {logo_path}")
    # Sinon utiliser le SVG SOURCE avec Inkscape
    elif os.path.exists(svg_source):
        print(f"   ✅ Utilisation du SVG SOURCE: {svg_source}")
        # Convertir avec Inkscape
        import subprocess

        temp_png = "bbia_mark_only_v2_temp.png"

        # Chercher Inkscape
        inkscape_paths = [
            "inkscape",
            "/opt/homebrew/bin/inkscape",
            "/Volumes/T7/Applications/Graphics/Inkscape/Inkscape.app/Contents/MacOS/inkscape",
            "/Applications/Inkscape.app/Contents/MacOS/inkscape",
        ]

        inkscape_cmd = None
        for path in inkscape_paths:
            try:
                if path == "inkscape":
                    result = subprocess.run(
                        [path, "--version"], capture_output=True, text=True, timeout=5
                    )
                else:
                    if os.path.exists(path):
                        result = subprocess.run(
                            [path, "--version"],
                            capture_output=True,
                            text=True,
                            timeout=5,
                        )
                    else:
                        continue
                if result.returncode == 0:
                    inkscape_cmd = path
                    break
            except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
                continue

        if inkscape_cmd:
            try:
                cmd = [
                    inkscape_cmd,
                    svg_source,
                    "--export-type=png",
                    f"--export-filename={temp_png}",
                    "--export-background-opacity=0",
                    "--export-dpi=96",
                    "--export-area-page",
                    "--export-width=512",
                ]
                subprocess.run(cmd, capture_output=True, text=True, check=True)
                logo_path = temp_png
                print(f"   ✅ SVG converti en PNG avec Inkscape: {temp_png}")
            except Exception as e:
                print(f"   ❌ Erreur conversion Inkscape: {e}")
                logo_path = None
        else:
            print("   ❌ Inkscape non trouvé, impossible de convertir SVG")
            logo_path = None
    else:
        print(f"   ⚠️  {svg_source} non trouvé !")
        # Essayer le favicon en dernier recours
        if os.path.exists("bbia_favicon_32x32.png"):
            logo_path = "bbia_favicon_32x32.png"
            print(f"   ✅ Utilisation du favicon: {logo_path}")
        else:
            print("❌ Aucun logo trouvé (PNG, SVG SOURCE, favicon)")
            return

    print(f"\n📸 Logo utilisé: {logo_path}")
    print(f"📁 Dossier de sortie: {output_dir}/\n")

    # Créer tous les mockups
    for name, color in COLORS.items():
        output_file = os.path.join(output_dir, f"mockup_{name}.png")
        create_mockup(color, logo_path, output_file, name.replace("_", " ").title())

    print(f"\n✅ {len(COLORS)} mockups créés dans {output_dir}/")
    print("\n📋 Fichiers créés:")
    for name in COLORS.keys():
        print(f"   • mockup_{name}.png")

    print("\n💡 Instructions:")
    print("   1. Ouvrir les mockups dans un visualiseur d'images")
    print("   2. Vérifier la lisibilité du logo sur chaque fond")
    print("   3. Noter les problèmes éventuels de contraste")
    print("   4. Documenter les résultats dans VALIDATION_FINALE.md")


if __name__ == "__main__":
    main()
