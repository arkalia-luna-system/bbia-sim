#!/usr/bin/env python3
"""
Script pour créer des screenshots/mockups pour GitHub
Crée des mockups visuels du logo sur différents fonds
"""
import os
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

# Couleurs de fond (selon COULEURS_OFFICIELLES.md)
COLORS = {
    "fond_clair": "#FFFFFF",  # Blanc
    "fond_sombre": "#020202",  # Noir foncé BBIA (détails)
    "fond_bleu": "#008181",  # Bleu BBIA (fond logo)
    "fond_noir": "#000000",  # Noir pur BBIA (texte)
}

# Tailles des mockups
MOCKUP_SIZES = {
    "github_header": (1200, 400),  # Header GitHub
    "readme_preview": (800, 600),  # Preview README
    "favicon_browser": (400, 300),  # Favicon dans navigateur
}


def create_mockup(
    background_color, logo_path, output_path, size, label, logo_size=None, is_svg=False
):
    """Crée un mockup avec fond et logo"""
    img = Image.new("RGB", size, background_color)
    draw = ImageDraw.Draw(img)

    try:
        # Si c'est un SVG, utiliser cairosvg pour le convertir
        if is_svg or str(logo_path).endswith(".svg"):
            try:
                import cairosvg

                # Convertir SVG en PNG en mémoire
                with open(logo_path, "rb") as f:
                    svg_data = f.read()
                import io

                png_data = cairosvg.svg2png(bytestring=svg_data)
                logo = Image.open(io.BytesIO(png_data)).convert("RGBA")
            except ImportError:
                print(f"   ⚠️  cairosvg non installé, impossible de traiter {logo_path}")
                return False
            except Exception as e:
                print(f"   ⚠️  Erreur conversion SVG: {e}")
                return False
        else:
            logo = Image.open(logo_path).convert("RGBA")

        # Redimensionner le logo si nécessaire
        if logo_size:
            # Gérer les tuples avec None
            if isinstance(logo_size, tuple):
                if logo_size[0] is not None and logo_size[1] is not None:
                    logo.thumbnail(logo_size, Image.Resampling.LANCZOS)
                elif logo_size[0] is not None:
                    # Redimensionner en gardant le ratio
                    ratio = logo_size[0] / logo.size[0]
                    new_height = int(logo.size[1] * ratio)
                    logo = logo.resize(
                        (logo_size[0], new_height), Image.Resampling.LANCZOS
                    )
                elif logo_size[1] is not None:
                    # Redimensionner en gardant le ratio
                    ratio = logo_size[1] / logo.size[1]
                    new_width = int(logo.size[0] * ratio)
                    logo = logo.resize(
                        (new_width, logo_size[1]), Image.Resampling.LANCZOS
                    )
            else:
                logo.thumbnail(logo_size, Image.Resampling.LANCZOS)

        # Centrer le logo
        x = (size[0] - logo.size[0]) // 2
        y = (size[1] - logo.size[1]) // 2

        # Coller le logo avec transparence
        if logo.mode == "RGBA":
            img.paste(logo, (x, y), logo)
        else:
            img.paste(logo, (x, y))

        # Ajouter un label si nécessaire
        if label:
            try:
                font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 16)
            except (OSError, IOError):
                font = ImageFont.load_default()

            text = label
            bbox = draw.textbbox((0, 0), text, font=font)
            text_x = (size[0] - (bbox[2] - bbox[0])) // 2
            text_y = size[1] - 30
            draw.text(
                (text_x, text_y),
                text,
                font=font,
                fill=(0, 0, 0) if background_color == "#FFFFFF" else (255, 255, 255),
            )

        img.save(output_path)
        print(f"   ✅ {os.path.basename(output_path)}")
        return True
    except Exception as e:
        print(f"   ❌ Erreur: {e}")
        return False


def create_favicon_mockup(favicon_path, output_path):
    """Crée un mockup du favicon dans un navigateur"""
    size = MOCKUP_SIZES["favicon_browser"]
    img = Image.new("RGB", size, "#F5F5F5")
    draw = ImageDraw.Draw(img)

    try:
        # Barre de navigation du navigateur
        draw.rectangle([(0, 0), (size[0], 40)], fill="#2D2D2D")
        draw.rectangle([(10, 5), (30, 35)], fill="#FFFFFF")

        # Favicon
        favicon = Image.open(favicon_path).convert("RGBA")
        favicon.thumbnail((24, 24), Image.Resampling.LANCZOS)
        img.paste(favicon, (13, 8), favicon)

        # Texte "BBIA Branding"
        try:
            font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 12)
        except (OSError, IOError):
            font = ImageFont.load_default()
        draw.text((40, 14), "BBIA Branding", font=font, fill="#FFFFFF")

        # Zone de contenu avec fonds différents
        y_offset = 60
        for i, (label, color) in enumerate(COLORS.items()):
            y = y_offset + (i * 60)
            draw.rectangle([(20, y), (size[0] - 20, y + 50)], fill=color)

            # Logo centré
            logo = Image.open(favicon_path).convert("RGBA")
            logo.thumbnail((32, 32), Image.Resampling.LANCZOS)
            logo_x = (size[0] - logo.size[0]) // 2
            logo_y = y + 9
            img.paste(logo, (logo_x, logo_y), logo)

        img.save(output_path)
        print(f"   ✅ {os.path.basename(output_path)}")
        return True
    except Exception as e:
        print(f"   ❌ Erreur favicon mockup: {e}")
        return False


def main():
    """Fonction principale"""
    print("=" * 70)
    print("📸 CRÉATION DES SCREENSHOTS POUR GITHUB")
    print("=" * 70)

    current_dir = Path(".")
    screenshots_dir = current_dir.parent.parent / "docs" / "screenshots"
    screenshots_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n📁 Dossier de sortie: {screenshots_dir}\n")

    # Mockups logo sur différents fonds
    # Utiliser les PNG finaux (générés depuis les _SOURCE) ou les SVG SOURCE directement
    logo_files = {
        "bbia_mark_only_512x512.png": {"name": "mark_only", "size": (256, 256)},
        # Essayer d'abord le PNG, sinon utiliser le SVG SOURCE
        "bbia_logo_vertical_v2.png": {
            "name": "logo_vertical",
            "size": (300, None),  # 300px largeur
        },
        "bbia_logo_vertical_v2_SOURCE.svg": {
            "name": "logo_vertical",
            "size": (300, None),
            "is_svg": True,
            "is_fallback": True,  # Utilisé seulement si PNG n'existe pas
        },
        # bbia_logo_horizontal.png peut ne pas exister, utiliser EXACTEMENT le SVG SOURCE
        "bbia_logo_horizontal_SOURCE.svg": {
            "name": "logo_horizontal",
            "size": (400, None),  # 400px largeur
            "is_svg": True,
        },
    }

    success_count = 0
    total_count = 0

    # Créer mockups pour chaque logo
    # Éviter les doublons : si PNG existe, ignorer le fallback SVG
    processed_names = set()

    for logo_file, config in logo_files.items():
        # Ignorer les fallbacks si le fichier principal existe déjà
        if config.get("is_fallback"):
            main_file = logo_file.replace("_SOURCE.svg", ".png")
            if (current_dir / main_file).exists():
                continue  # PNG existe, ignorer le fallback SVG

        logo_path = current_dir / logo_file
        if not logo_path.exists():
            print(f"⚠️  Logo non trouvé: {logo_file}")
            # Pour le logo horizontal, essayer le PNG si le SVG SOURCE n'existe pas
            if logo_file == "bbia_logo_horizontal_SOURCE.svg":
                alt_path = current_dir / "bbia_logo_horizontal.png"
                if alt_path.exists():
                    logo_path = alt_path
                    print(f"   ✅ Utilisation alternative: {alt_path.name}")
                else:
                    continue
            # Pour le logo vertical, essayer le SVG SOURCE si le PNG n'existe pas
            elif logo_file == "bbia_logo_vertical_v2.png":
                alt_path = current_dir / "bbia_logo_vertical_v2_SOURCE.svg"
                if alt_path.exists():
                    logo_path = alt_path
                    config["is_svg"] = True
                    print(
                        f"   ✅ Utilisation alternative (SVG SOURCE): {alt_path.name}"
                    )
                else:
                    continue
            else:
                continue

        # Éviter de traiter deux fois le même logo (PNG + fallback SVG)
        if config["name"] in processed_names:
            continue
        processed_names.add(config["name"])

        print(f"📄 {logo_file}:")

        # Mockups sur différents fonds
        for label, color in COLORS.items():
            total_count += 1
            output_file = screenshots_dir / f"{config['name']}_{label}.png"
            is_svg = config.get("is_svg", False) or str(logo_path).endswith(".svg")
            if create_mockup(
                color,
                logo_path,
                output_file,
                MOCKUP_SIZES["readme_preview"],
                f"Logo sur {label}",
                config["size"],
                is_svg=is_svg,
            ):
                success_count += 1

        print()

    # Mockup favicon dans navigateur
    favicon_path = current_dir / "bbia_favicon_32x32.png"
    if favicon_path.exists():
        total_count += 1
        output_file = screenshots_dir / "favicon_navigateur.png"
        if create_favicon_mockup(favicon_path, output_file):
            success_count += 1

    print("✅ Résumé:")
    print(f"   • {success_count}/{total_count} screenshots créés")
    print(f"   • Dossier: {screenshots_dir}")

    if success_count == total_count:
        print("\n💡 Tous les screenshots ont été créés !")
        print("   Utilise-les dans le README pour améliorer l'affichage.")


if __name__ == "__main__":
    main()
