#!/usr/bin/env python3
"""
Script pour extraire les couleurs réelles des logos SVG _SOURCE
Utilise uniquement les fichiers _SOURCE.svg pour garantir les bonnes couleurs
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import Counter


def extract_colors_from_svg(svg_file):
    """Extrait toutes les couleurs d'un fichier SVG"""
    colors = set()

    try:
        tree = ET.parse(svg_file)
        root = tree.getroot()

        # Chercher tous les attributs fill et stroke
        for elem in root.iter():
            # Attribut fill
            fill = elem.get("fill")
            if fill and fill.startswith("#"):
                # Normaliser (enlever les espaces, convertir en majuscules)
                fill = fill.strip().upper()
                if len(fill) == 7:  # #RRGGBB
                    colors.add(fill)
                elif len(fill) == 9:  # #RRGGBBAA
                    colors.add(fill[:7])  # Prendre sans alpha pour comparaison
                    colors.add(fill)  # Garder aussi avec alpha

            # Attribut stroke
            stroke = elem.get("stroke")
            if stroke and stroke.startswith("#"):
                stroke = stroke.strip().upper()
                if len(stroke) == 7:
                    colors.add(stroke)
                elif len(stroke) == 9:
                    colors.add(stroke[:7])
                    colors.add(stroke)

            # Chercher dans le style
            style = elem.get("style")
            if style:
                # Extraire fill et stroke du style
                fill_match = re.search(r"fill:\s*#([0-9A-Fa-f]{6,8})", style)
                if fill_match:
                    hex_color = "#" + fill_match.group(1).upper()
                    if len(hex_color) == 7:
                        colors.add(hex_color)
                    elif len(hex_color) == 9:
                        colors.add(hex_color[:7])
                        colors.add(hex_color)

                stroke_match = re.search(r"stroke:\s*#([0-9A-Fa-f]{6,8})", style)
                if stroke_match:
                    hex_color = "#" + stroke_match.group(1).upper()
                    if len(hex_color) == 7:
                        colors.add(hex_color)
                    elif len(hex_color) == 9:
                        colors.add(hex_color[:7])
                        colors.add(hex_color)

        # Chercher aussi dans le contenu textuel (pour les gradients, etc.)
        svg_content = Path(svg_file).read_text(encoding="utf-8")
        hex_colors = re.findall(r"#([0-9A-Fa-f]{6,8})", svg_content, re.IGNORECASE)
        for hex_color in hex_colors:
            hex_color = "#" + hex_color.upper()
            if len(hex_color) == 7:
                colors.add(hex_color)
            elif len(hex_color) == 9:
                colors.add(hex_color[:7])
                colors.add(hex_color)

    except Exception as e:
        print(f"❌ Erreur lors de l'analyse de {svg_file}: {e}")
        return set()

    return colors


def hex_to_rgb(hex_color):
    """Convertit une couleur hex en RGB"""
    hex_color = hex_color.lstrip("#")
    if len(hex_color) == 6:
        return tuple(int(hex_color[i : i + 2], 16) for i in (0, 2, 4))
    elif len(hex_color) == 8:
        return tuple(int(hex_color[i : i + 2], 16) for i in (0, 2, 4, 6))
    return None


def main():
    """Fonction principale"""
    print("=" * 70)
    print("🎨 EXTRACTION DES COULEURS DES LOGOS _SOURCE")
    print("=" * 70)

    current_dir = Path(".")

    # FICHIERS SVG SOURCES - UNIQUEMENT CES FICHIERS
    source_files = [
        "bbia_mark_only_v2_SOURCE.svg",
        "bbia_logo_vertical_v2_SOURCE.svg",
        "bbia_logo_horizontal_SOURCE.svg",
    ]

    all_colors = {}

    for svg_file in source_files:
        svg_path = current_dir / svg_file

        if not svg_path.exists():
            print(f"⚠️  Fichier non trouvé: {svg_file}")
            continue

        print(f"\n📄 Analyse de {svg_file}...")
        colors = extract_colors_from_svg(svg_path)

        if colors:
            all_colors[svg_file] = sorted(colors)
            print(f"   ✅ {len(colors)} couleurs trouvées:")
            for color in sorted(colors):
                rgb = hex_to_rgb(color)
                if rgb:
                    if len(rgb) == 3:
                        print(f"      • {color} → RGB({rgb[0]}, {rgb[1]}, {rgb[2]})")
                    else:
                        print(
                            f"      • {color} → RGBA({rgb[0]}, {rgb[1]}, {rgb[2]}, {rgb[3]})"
                        )
        else:
            print("   ⚠️  Aucune couleur trouvée")

    # Couleurs communes à tous les fichiers
    if len(all_colors) > 1:
        common_colors = set.intersection(
            *[set(colors) for colors in all_colors.values()]
        )
        if common_colors:
            print("\n" + "=" * 70)
            print("🎯 COULEURS COMMUNES À TOUS LES LOGOS:")
            print("=" * 70)
            for color in sorted(common_colors):
                rgb = hex_to_rgb(color)
                if rgb:
                    if len(rgb) == 3:
                        print(f"   • {color} → RGB({rgb[0]}, {rgb[1]}, {rgb[2]})")
                    else:
                        print(
                            f"   • {color} → RGBA({rgb[0]}, {rgb[1]}, {rgb[2]}, {rgb[3]})"
                        )

    print("\n✅ Extraction terminée")
    print("\n💡 Utilisez ces couleurs pour créer le fichier COULEURS_OFFICIELLES.md")


if __name__ == "__main__":
    main()
