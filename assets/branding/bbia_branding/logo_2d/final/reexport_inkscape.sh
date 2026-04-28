#!/bin/bash
# Script pour réexporter les logos avec Inkscape en ligne de commande
# Garantit les bonnes couleurs sans artefacts

echo "======================================================================"
echo "🎨 RÉEXPORT AVEC INKSCAPE (BONNES COULEURS)"
echo "======================================================================"
echo ""

cd "$(dirname "$0")"

# Fonction pour exporter avec Inkscape
export_inkscape() {
    local svg_file=$1
    local png_file=$2
    local width=$3
    local height=$4
    
    if [ ! -f "$svg_file" ]; then
        echo "❌ SVG non trouvé: $svg_file"
        return 1
    fi
    
    echo "📸 Export: $svg_file → $png_file"
    
    if [ -n "$width" ] && [ -n "$height" ]; then
        inkscape "$svg_file" \
            --export-type=png \
            --export-filename="$png_file" \
            --export-width="$width" \
            --export-height="$height" \
            --export-background-opacity=0 \
            --export-dpi=96 \
            --export-area-page
    elif [ -n "$width" ]; then
        inkscape "$svg_file" \
            --export-type=png \
            --export-filename="$png_file" \
            --export-width="$width" \
            --export-background-opacity=0 \
            --export-dpi=96 \
            --export-area-page
    else
        inkscape "$svg_file" \
            --export-type=png \
            --export-filename="$png_file" \
            --export-background-opacity=0 \
            --export-dpi=96 \
            --export-area-page
    fi
    
    if [ $? -eq 0 ]; then
        echo "   ✅ Export réussi"
        return 0
    else
        echo "   ❌ Erreur lors de l'export"
        return 1
    fi
}

# Exports - UTILISER UNIQUEMENT LES FICHIERS _SOURCE
echo "📄 bbia_mark_only_v2_SOURCE.svg:"
export_inkscape "bbia_mark_only_v2_SOURCE.svg" "bbia_mark_only_v2.png" "" ""
export_inkscape "bbia_mark_only_v2_SOURCE.svg" "bbia_mark_only_512x512.png" "512" "512"
export_inkscape "bbia_mark_only_v2_SOURCE.svg" "bbia_favicon_32x32.png" "32" "32"
echo ""

echo "📄 bbia_logo_vertical_v2_SOURCE.svg:"
export_inkscape "bbia_logo_vertical_v2_SOURCE.svg" "bbia_logo_vertical_v2.png" "" ""
echo ""

echo "📄 bbia_logo_horizontal_SOURCE.svg:"
export_inkscape "bbia_logo_horizontal_SOURCE.svg" "bbia_logo_horizontal.png" "1024" ""
echo ""

echo "✅ Tous les fichiers ont été réexportés avec Inkscape !"
echo "💡 Vérifiez maintenant que les couleurs sont correctes (bleu #008181, blanc, etc.)"

