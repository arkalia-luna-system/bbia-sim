#!/usr/bin/env python3
"""
Script amélioré pour créer des mockups professionnels du logo BBIA
Génère des mockups avec différentes formes, tailles et présentations
"""
import os
import subprocess
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

# Couleurs officielles BBIA (selon COULEURS_OFFICIELLES.md)
COLORS = {
    "fond_clair": "#FFFFFF",  # Blanc
    "fond_sombre": "#020202",  # Noir foncé BBIA
    "fond_bleu": "#008181",  # Bleu BBIA
    "fond_noir": "#000000",  # Noir pur
    "fond_gris": "#CCCCCC",  # Gris clair (yeux)
}

# Tailles et formats de mockups
MOCKUP_CONFIGS = {
    "card": {"size": (400, 300), "logo_size": (200, 200), "shape": "rounded"},
    "banner": {"size": (800, 200), "logo_size": (150, 150), "shape": "rect"},
    "square": {"size": (600, 600), "logo_size": (400, 400), "shape": "square"},
    "header": {"size": (1200, 400), "logo_size": (300, 300), "shape": "rect"},
    "badge": {"size": (300, 300), "logo_size": (200, 200), "shape": "circle"},
}


def find_inkscape():
    """Trouve le chemin d'Inkscape"""
    inkscape_paths = [
        "inkscape",
        "/opt/homebrew/bin/inkscape",
        "/Volumes/T7/Applications/Graphics/Inkscape/Inkscape.app/Contents/MacOS/inkscape",
        "/Applications/Inkscape.app/Contents/MacOS/inkscape",
    ]

    for path in inkscape_paths:
        try:
            if path == "inkscape":
                result = subprocess.run(
                    [path, "--version"], capture_output=True, text=True, timeout=5
                )
            else:
                if os.path.exists(path):
                    result = subprocess.run(
                        [path, "--version"], capture_output=True, text=True, timeout=5
                    )
                else:
                    continue
            if result.returncode == 0:
                return path
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
            continue
    return None


def get_logo_path():
    """Récupère le chemin du logo (PNG existant ou conversion depuis _SOURCE.svg)"""
    # Priorité 1: PNG existant
    if os.path.exists("bbia_mark_only_v2.png"):
        return "bbia_mark_only_v2.png", False

    # Priorité 2: SVG SOURCE avec Inkscape
    svg_source = "bbia_mark_only_v2_SOURCE.svg"
    if os.path.exists(svg_source):
        inkscape_cmd = find_inkscape()
        if inkscape_cmd:
            temp_png = "bbia_mark_only_v2_temp.png"
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
                return temp_png, True
            except Exception:
                pass

    # Priorité 3: Favicon
    if os.path.exists("bbia_favicon_32x32.png"):
        return "bbia_favicon_32x32.png", False

    return None, False


def create_rounded_rectangle(draw, bbox, radius, fill=None, outline=None, width=1):
    """Dessine un rectangle aux coins arrondis"""
    x1, y1, x2, y2 = bbox
    draw.rectangle(
        [x1 + radius, y1, x2 - radius, y2], fill=fill, outline=outline, width=width
    )
    draw.rectangle(
        [x1, y1 + radius, x2, y2 - radius], fill=fill, outline=outline, width=width
    )
    draw.ellipse(
        [x1, y1, x1 + 2 * radius, y1 + 2 * radius],
        fill=fill,
        outline=outline,
        width=width,
    )
    draw.ellipse(
        [x2 - 2 * radius, y1, x2, y1 + 2 * radius],
        fill=fill,
        outline=outline,
        width=width,
    )
    draw.ellipse(
        [x1, y2 - 2 * radius, x1 + 2 * radius, y2],
        fill=fill,
        outline=outline,
        width=width,
    )
    draw.ellipse(
        [x2 - 2 * radius, y2 - 2 * radius, x2, y2],
        fill=fill,
        outline=outline,
        width=width,
    )


def create_enhanced_mockup(
    background_color, logo_path, output_path, config_name, config, label
):
    """Crée un mockup amélioré avec forme et présentation professionnelle"""
    size = config["size"]
    logo_size = config["logo_size"]
    shape = config["shape"]

    # Créer l'image de fond
    img = Image.new("RGB", size, background_color)
    draw = ImageDraw.Draw(img)

    # Ajouter un effet de profondeur (ombre subtile)
    if shape == "rounded":
        # Fond avec coins arrondis
        margin = 20
        rounded_rect = [margin, margin, size[0] - margin, size[1] - margin]
        create_rounded_rectangle(
            draw, rounded_rect, 15, fill=background_color, outline="#E0E0E0", width=2
        )
    elif shape == "circle":
        # Cercle centré
        center_x, center_y = size[0] // 2, size[1] // 2
        radius = min(size) // 2 - 20
        draw.ellipse(
            [
                center_x - radius,
                center_y - radius,
                center_x + radius,
                center_y + radius,
            ],
            fill=background_color,
            outline="#E0E0E0",
            width=2,
        )

    # Charger et redimensionner le logo
    try:
        logo = Image.open(logo_path).convert("RGBA")

        # Redimensionner en gardant le ratio
        logo.thumbnail(logo_size, Image.Resampling.LANCZOS)

        # Centrer le logo
        x = (size[0] - logo.size[0]) // 2
        y = (size[1] - logo.size[1]) // 2

        # Coller le logo avec transparence
        if logo.mode == "RGBA":
            img.paste(logo, (x, y), logo)
        else:
            img.paste(logo, (x, y))

        # Ajouter un label stylisé en bas
        try:
            font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 18)
        except (OSError, IOError):
            font = ImageFont.load_default()

        # Texte avec fond semi-transparent
        text = f"{label} • {background_color}"
        bbox = draw.textbbox((0, 0), text, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]
        text_x = (size[0] - text_width) // 2
        text_y = size[1] - text_height - 15

        # Fond pour le texte (améliore la lisibilité)
        padding = 8
        text_bg = [
            text_x - padding,
            text_y - padding // 2,
            text_x + text_width + padding,
            text_y + text_height + padding // 2,
        ]
        draw.rectangle(
            text_bg,
            fill=(
                (0, 0, 0, 180)
                if background_color == "#FFFFFF"
                else (255, 255, 255, 180)
            ),
        )

        # Couleur du texte selon le fond
        text_color = "#000000" if background_color == "#FFFFFF" else "#FFFFFF"
        draw.text((text_x, text_y), text, fill=text_color, font=font)

    except Exception as e:
        print(f"⚠️  Erreur lors du chargement du logo {logo_path}: {e}")

    # Sauvegarder
    img.save(output_path, "PNG", optimize=True)
    print(f"✅ Mockup créé: {output_path}")


def main():
    """Fonction principale"""
    print("=" * 70)
    print("🎨 CRÉATION DE MOCKUPS AMÉLIORÉS")
    print("=" * 70)

    # Récupérer le logo
    logo_path, is_temp = get_logo_path()
    if not logo_path:
        print("❌ Aucun logo trouvé (PNG, SVG SOURCE, favicon)")
        return

    print(f"\n📸 Logo utilisé: {logo_path}")

    # Dossier de sortie
    output_dir = "tests_visuels"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"📁 Dossier créé: {output_dir}/")

    print(f"\n📁 Dossier de sortie: {output_dir}/\n")

    # Créer tous les mockups améliorés
    total_created = 0
    for color_name, color in COLORS.items():
        for config_name, config in MOCKUP_CONFIGS.items():
            label = f"{color_name.replace('_', ' ').title()} • {config_name.title()}"
            output_file = os.path.join(
                output_dir, f"mockup_{color_name}_{config_name}.png"
            )
            create_enhanced_mockup(
                color, logo_path, output_file, config_name, config, label
            )
            total_created += 1

    print(f"\n✅ {total_created} mockups améliorés créés dans {output_dir}/")

    # Nettoyer le fichier temporaire si créé
    if is_temp and os.path.exists(logo_path):
        os.remove(logo_path)
        print(f"🧹 Fichier temporaire supprimé: {logo_path}")

    print("\n💡 Les mockups incluent:")
    print("   • Différentes formes (carré, rectangle, cercle, arrondi)")
    print("   • Différentes tailles (card, banner, square, header, badge)")
    print("   • Tous les fonds officiels BBIA")
    print("   • Labels stylisés avec informations")


if __name__ == "__main__":
    main()
