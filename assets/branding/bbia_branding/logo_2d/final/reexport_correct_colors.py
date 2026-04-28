#!/usr/bin/env python3
"""
Script pour réexporter les logos avec les bonnes couleurs depuis Inkscape
Utilise Inkscape en ligne de commande pour garantir les couleurs correctes
"""
import os
import subprocess
from pathlib import Path


def reexport_with_inkscape(svg_file, output_png, width=None, height=None):
    """Réexporte un SVG en PNG avec Inkscape pour préserver les couleurs"""
    svg_path = Path(svg_file).absolute()
    png_path = Path(output_png).absolute()

    if not svg_path.exists():
        print(f"❌ SVG non trouvé: {svg_path}")
        return False

    # Commande Inkscape pour exporter en PNG avec les BONS paramètres
    # Utiliser le chemin Inkscape trouvé (peut être dans T7)
    inkscape_exec = getattr(reexport_with_inkscape, "inkscape_path", "inkscape")
    cmd = [
        inkscape_exec,
        str(svg_path),
        "--export-type=png",
        f"--export-filename={png_path}",
        "--export-background-opacity=0",  # Transparence
        "--export-dpi=96",  # DPI standard
        "--export-area-page",  # IMPORTANT : exporter la page entière (pas le dessin)
    ]

    # Ajouter les dimensions si spécifiées
    if width and height:
        cmd.append(f"--export-width={width}")
        cmd.append(f"--export-height={height}")
    elif width:
        cmd.append(f"--export-width={width}")
    # Si aucune dimension, Inkscape utilise la taille de la page

    try:
        print(f"📸 Export Inkscape: {svg_path.name} → {png_path.name}")
        if width:
            print(f"   Taille: {width}x{height if height else 'auto'}")
        subprocess.run(cmd, capture_output=True, text=True, check=True)
        print("   ✅ Export réussi")
        return True
    except subprocess.CalledProcessError as e:
        print(f"   ❌ Erreur: {e.stderr}")
        return False
    except FileNotFoundError:
        print("   ❌ Inkscape non trouvé dans le PATH")
        print("   ⚠️  Inkscape est OBLIGATOIRE pour générer les bons logos")
        print("   💡 Installez Inkscape ou exportez manuellement depuis Inkscape")
        return False


# Fonction cairosvg supprimée - Inkscape est OBLIGATOIRE pour les bons logos


def main():
    """Fonction principale"""
    print("=" * 70)
    print("🎨 RÉEXPORT AVEC BONNES COULEURS")
    print("=" * 70)

    current_dir = Path(".")

    # FICHIERS SVG SOURCES EXACTS - UTILISER UNIQUEMENT CES FICHIERS
    svg_files = {
        "bbia_mark_only_v2_SOURCE.svg": [
            ("bbia_mark_only_v2.png", None, None),  # Taille originale
            ("bbia_mark_only_512x512.png", 512, 512),
            ("bbia_favicon_32x32.png", 32, 32),
        ],
        "bbia_logo_vertical_v2_SOURCE.svg": [
            ("bbia_logo_vertical_v2.png", None, None),
        ],
        "bbia_logo_horizontal_SOURCE.svg": [
            ("bbia_logo_horizontal.png", 1024, None),  # Largeur 1024px
        ],
    }

    print("   ✅ Mark Only: bbia_mark_only_v2_SOURCE.svg")
    print("   ✅ Vertical: bbia_logo_vertical_v2_SOURCE.svg")
    print("   ✅ Horizontal: bbia_logo_horizontal_SOURCE.svg")

    # Vérifier si Inkscape est disponible (OBLIGATOIRE)
    # Chercher Inkscape dans plusieurs emplacements
    inkscape_paths = [
        "inkscape",  # Dans le PATH
        "/opt/homebrew/bin/inkscape",  # Homebrew
        "/Volumes/T7/Applications/Graphics/Inkscape/Inkscape.app/Contents/MacOS/inkscape",  # T7
        "/Applications/Inkscape.app/Contents/MacOS/inkscape",  # Applications standard
    ]

    inkscape_cmd = None
    inkscape_available = False

    for path in inkscape_paths:
        try:
            if path == "inkscape":
                # Vérifier dans le PATH
                result = subprocess.run(
                    [path, "--version"], capture_output=True, text=True, timeout=5
                )
            else:
                # Vérifier le chemin absolu
                if os.path.exists(path):
                    result = subprocess.run(
                        [path, "--version"], capture_output=True, text=True, timeout=5
                    )
                else:
                    continue

            if result.returncode == 0:
                inkscape_cmd = path
                inkscape_available = True
                print(f"✅ Inkscape trouvé: {result.stdout.strip()}")
                print(f"   Chemin: {path}")
                break
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
            continue

    if not inkscape_available:
        print("❌ Inkscape non trouvé")
        print("⚠️  Inkscape est OBLIGATOIRE pour générer les bons logos")
        print("💡 Emplacements vérifiés:")
        for p in inkscape_paths:
            print(f"   - {p}")
        print("\n💡 Installez Inkscape ou exportez manuellement depuis Inkscape")
        return

    if not inkscape_available:
        print("\n❌ Impossible de continuer sans Inkscape")
        return

    print("\n📸 Réexport des fichiers avec Inkscape...\n")

    success_count = 0
    total_count = 0

    for svg_file, outputs in svg_files.items():
        svg_path = current_dir / svg_file

        if not svg_path.exists():
            print(f"❌ SVG non trouvé: {svg_file}")
            continue

        print(f"📄 {svg_file}:")

        for png_file, width, height in outputs:
            total_count += 1
            png_path = current_dir / png_file

            # UTILISER UNIQUEMENT INKSCAPE (pas cairosvg)
            # Passer le chemin Inkscape trouvé à la fonction
            reexport_with_inkscape.inkscape_path = inkscape_cmd
            if reexport_with_inkscape(svg_path, png_path, width, height):
                success_count += 1

        print()

    print("✅ Résumé:")
    print(f"   • {success_count}/{total_count} fichiers exportés avec succès")

    if success_count == total_count:
        print("\n💡 Tous les fichiers ont été réexportés avec les bonnes couleurs !")
        print("   Vérifiez maintenant que les couleurs sont correctes.")
    else:
        print("\n⚠️  Certains fichiers n'ont pas pu être exportés")
        print("   Vérifiez les erreurs ci-dessus")


if __name__ == "__main__":
    main()
