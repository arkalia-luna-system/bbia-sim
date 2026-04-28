#!/usr/bin/env python3
"""
Script pour créer le logo horizontal depuis le logo vertical
Reconnaît tous les calques et permet de les repositionner
"""
import xml.etree.ElementTree as ET
import re
import os
import sys


def get_all_layers(svg_file):
    """Liste tous les calques/éléments du SVG"""
    tree = ET.parse(svg_file)
    root = tree.getroot()

    layers = []

    # Parcourir tous les éléments
    for elem in root.iter():
        tag = elem.tag.split("}")[-1] if "}" in elem.tag else elem.tag

        # Chercher les paths, groupes, etc.
        if tag in ["path", "g", "text", "rect", "circle", "ellipse"]:
            layer_id = elem.get("id", "")
            label = elem.get("{http://www.inkscape.org/namespaces/inkscape}label", "")
            inkscape_groupmode = elem.get(
                "{http://www.inkscape.org/namespaces/inkscape}groupmode", ""
            )

            # Extraire les coordonnées du path si disponible
            d_attr = elem.get("d", "")
            bounds = None
            if d_attr:
                bounds = extract_path_bounds(d_attr)

            if layer_id or label or tag == "g":
                layers.append(
                    {
                        "element": elem,
                        "tag": tag,
                        "id": layer_id,
                        "label": label,
                        "groupmode": inkscape_groupmode,
                        "bounds": bounds,
                    }
                )

    return layers, tree, root


def extract_path_bounds(d_path):
    """Extrait les coordonnées min/max d'un path SVG"""
    # Extraire tous les nombres (coordonnées)
    numbers = re.findall(r"([-]?\d+\.?\d*)", d_path)
    if not numbers:
        return None

    # Convertir en float
    coords = []
    for n in numbers:
        try:
            coords.append(float(n))
        except ValueError:
            pass

    if len(coords) < 2:
        return None

    # Séparer X et Y (approximation : pairs = X, impairs = Y)
    x_coords = []
    y_coords = []

    # Parser les commandes SVG (M, L, C, etc.)
    # Simplification : prendre tous les nombres
    for i, coord in enumerate(coords):
        if i % 2 == 0:
            x_coords.append(coord)
        else:
            y_coords.append(coord)

    if not x_coords or not y_coords:
        return None

    return {
        "min_x": min(x_coords),
        "max_x": max(x_coords),
        "min_y": min(y_coords),
        "max_y": max(y_coords),
        "width": max(x_coords) - min(x_coords),
        "height": max(y_coords) - min(y_coords),
        "center_x": (min(x_coords) + max(x_coords)) / 2,
        "center_y": (min(y_coords) + max(y_coords)) / 2,
    }


def list_layers(svg_file):
    """Affiche tous les calques trouvés"""
    layers, _, _ = get_all_layers(svg_file)

    print("\n" + "=" * 70)
    print("📋 CALQUES TROUVÉS DANS LE SVG")
    print("=" * 70)

    for i, layer in enumerate(layers, 1):
        print(f"\n{i}. {layer['tag'].upper()}")
        if layer["id"]:
            print(f"   ID: {layer['id']}")
        if layer["label"]:
            print(f"   Label: {layer['label']}")
        if layer["groupmode"]:
            print(f"   Groupe: {layer['groupmode']}")
        if layer["bounds"]:
            b = layer["bounds"]
            print(
                f"   Position: x={b['min_x']:.1f}-{b['max_x']:.1f}, y={b['min_y']:.1f}-{b['max_y']:.1f}"
            )
            print(f"   Taille: {b['width']:.1f}x{b['height']:.1f}px")
            print(f"   Centre: ({b['center_x']:.1f}, {b['center_y']:.1f})")

    print("\n" + "=" * 70)

    return layers


def create_horizontal_logo(
    svg_file="bbia_logo_vertical_v2_SOURCE.svg",
    output_file="bbia_logo_horizontal_SOURCE.svg",
):
    """Crée le logo horizontal"""

    if not os.path.exists(svg_file):
        print(f"❌ Fichier {svg_file} introuvable")
        return False

    # Lister tous les calques
    print("🔍 Analyse du SVG...")
    layers, tree, root = get_all_layers(svg_file)

    # Trouver le symbole (robot) et le texte (BBIA)
    symbole_layer = None
    texte_layer = None

    for layer in layers:
        label = layer["label"].lower() if layer["label"] else ""

        # Le texte a "ecriture" dans le label
        if label == "ecriture" or "ecriture" in label and "sans" not in label:
            texte_layer = layer
            print(f"✅ Texte trouvé : {layer['id']} (label: {layer['label']})")

        # Le symbole a "contoure" et "sans ecriture" dans le label
        elif "contoure" in label and "sans ecriture" in label:
            symbole_layer = layer
            print(f"✅ Symbole trouvé : {layer['id']} (label: {layer['label']})")

    if not texte_layer or not symbole_layer:
        print("\n❌ Impossible de trouver le texte ou le symbole")
        print("\n📋 Liste de tous les calques disponibles :")
        list_layers(svg_file)
        return False

    # Obtenir les bounding boxes
    symbole_bounds = symbole_layer["bounds"]
    texte_bounds = texte_layer["bounds"]

    if not symbole_bounds or not texte_bounds:
        print("❌ Impossible d'extraire les dimensions")
        return False

    print("\n📐 Dimensions calculées :")
    print(
        f"   Symbole : {symbole_bounds['width']:.1f}x{symbole_bounds['height']:.1f}px"
    )
    print(f"   Texte   : {texte_bounds['width']:.1f}x{texte_bounds['height']:.1f}px")

    # Calculer les nouvelles positions pour le logo horizontal
    # Le symbole reste à gauche, le texte va à droite

    # Position X : après le symbole + espacement
    espacement = 40  # 40px d'espacement
    nouveau_x_texte = symbole_bounds["max_x"] + espacement

    # Position Y : aligner verticalement (centrer sur le symbole)
    nouveau_y_texte = symbole_bounds["center_y"] - texte_bounds["height"] / 2

    # Calculer la translation nécessaire
    translate_x = nouveau_x_texte - texte_bounds["min_x"]
    translate_y = nouveau_y_texte - texte_bounds["min_y"]

    print("\n📐 Transformations calculées :")
    print(f"   Texte : translate({translate_x:.1f}, {translate_y:.1f})")

    # Appliquer la transformation au texte
    texte_element = texte_layer["element"]
    current_transform = texte_element.get("transform", "")

    if current_transform:
        # Ajouter la nouvelle transformation
        new_transform = (
            f"{current_transform} translate({translate_x:.2f}, {translate_y:.2f})"
        )
    else:
        new_transform = f"translate({translate_x:.2f}, {translate_y:.2f})"

    texte_element.set("transform", new_transform)

    # Ajuster le viewBox pour accommoder le logo horizontal
    nouvelle_largeur = (
        nouveau_x_texte + texte_bounds["width"] + 50
    )  # 50px de marge droite
    nouvelle_hauteur = (
        max(symbole_bounds["height"], texte_bounds["height"]) + 100
    )  # 100px de marge

    # Mettre à jour les dimensions du SVG
    root.set("width", str(int(nouvelle_largeur)))
    root.set("height", str(int(nouvelle_hauteur)))

    # Mettre à jour le viewBox
    root.set("viewBox", f"0 0 {int(nouvelle_largeur)} {int(nouvelle_hauteur)}")

    # Sauvegarder
    tree.write(output_file, encoding="utf-8", xml_declaration=True)

    print(f"\n✅ Logo horizontal créé : {output_file}")
    print(f"   Dimensions : {int(nouvelle_largeur)}x{int(nouvelle_hauteur)}px")

    # Essayer d'exporter en PNG avec cairosvg
    try:
        import cairosvg

        png_output = output_file.replace(".svg", ".png")
        width = 1024
        height = int(1024 * nouvelle_hauteur / nouvelle_largeur)

        print("\n📤 Export PNG en cours...")
        cairosvg.svg2png(
            url=output_file,
            write_to=png_output,
            output_width=width,
            output_height=height,
        )
        print(f"✅ PNG exporté : {png_output} ({width}x{height}px)")
    except ImportError:
        print("\n💡 Pour exporter en PNG, installer cairosvg : pip3 install cairosvg")
        print("   Ou exporter manuellement depuis Inkscape")
    except Exception as e:
        print(f"\n⚠️  Erreur export PNG : {e}")
        print("   Exporter manuellement depuis Inkscape")

    return True


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Créer logo horizontal ou lister les calques"
    )
    parser.add_argument("--list", action="store_true", help="Lister tous les calques")
    parser.add_argument(
        "--input",
        default="bbia_logo_vertical_v2_SOURCE.svg",
        help="Fichier SVG d'entrée",
    )
    parser.add_argument(
        "--output",
        default="bbia_logo_horizontal_SOURCE.svg",
        help="Fichier SVG de sortie",
    )

    args = parser.parse_args()

    if args.list:
        list_layers(args.input)
    else:
        success = create_horizontal_logo(args.input, args.output)
        sys.exit(0 if success else 1)
