#!/bin/bash
"""
record_demo.sh - Script pour enregistrer une démo MuJoCo
Génère une vidéo 15-20s + données pour graphe qpos
"""

set -e  # Arrêt en cas d'erreur

# Configuration
DEMO_NAME=${1:-"happy"}
DURATION=${2:-15}
OUTPUT_DIR="assets/videos"
PLOT_DIR="assets/plots"

# Créer les répertoires
mkdir -p "$OUTPUT_DIR"
mkdir -p "$PLOT_DIR"

echo "🎬 Enregistrement démo: $DEMO_NAME (${DURATION}s)"
echo "📁 Sortie: $OUTPUT_DIR"

# Vérifier que mjpython est disponible
if ! command -v mjpython &> /dev/null; then
    echo "❌ mjpython non trouvé. Installez MuJoCo d'abord."
    exit 1
fi

# Vérifier que le script de démo existe
DEMO_SCRIPT="examples/demo_emotion_fixed.py"
if [ ! -f "$DEMO_SCRIPT" ]; then
    echo "❌ Script de démo non trouvé: $DEMO_SCRIPT"
    exit 1
fi

# Nom des fichiers de sortie
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
VIDEO_FILE="$OUTPUT_DIR/demo_${DEMO_NAME}_${TIMESTAMP}.mp4"
TRACE_FILE="$OUTPUT_DIR/demo_${DEMO_NAME}_${TIMESTAMP}.jsonl"

echo "📹 Vidéo: $VIDEO_FILE"
echo "📊 Trace: $TRACE_FILE"

# Enregistrer la trace (pour le graphe)
echo "📊 Enregistrement trace qpos..."
python scripts/record_trace.py \
    --emotion "$DEMO_NAME" \
    --duration "$DURATION" \
    --out "$TRACE_FILE" \
    --backend mujoco

if [ $? -eq 0 ]; then
    echo "✅ Trace enregistrée: $TRACE_FILE"
else
    echo "❌ Erreur enregistrement trace"
    exit 1
fi

# Note: Pour la vidéo, on utilise une approche différente
# car MuJoCo ne permet pas facilement l'enregistrement vidéo programmatique
echo "📝 Note: Pour la vidéo, utilisez un enregistreur d'écran"
echo "   - Lancez: mjpython examples/demo_emotion_fixed.py --emotion $DEMO_NAME --duration $DURATION"
echo "   - Enregistrez l'écran pendant $DURATION secondes"
echo "   - Sauvegardez comme: $VIDEO_FILE"

# Générer le graphe qpos
echo "📈 Génération graphe qpos..."
python scripts/plot_trace.py \
    --input "$TRACE_FILE" \
    --output "$PLOT_DIR/demo_${DEMO_NAME}_${TIMESTAMP}.png" \
    --title "Démo $DEMO_NAME - Mouvements joints"

if [ $? -eq 0 ]; then
    echo "✅ Graphe généré: $PLOT_DIR/demo_${DEMO_NAME}_${TIMESTAMP}.png"
else
    echo "❌ Erreur génération graphe"
    exit 1
fi

echo ""
echo "🎉 Démo $DEMO_NAME enregistrée avec succès !"
echo "📁 Fichiers générés:"
echo "   - Trace: $TRACE_FILE"
echo "   - Graphe: $PLOT_DIR/demo_${DEMO_NAME}_${TIMESTAMP}.png"
echo "   - Vidéo: $VIDEO_FILE (à enregistrer manuellement)"
echo ""
echo "🚀 Pour voir la démo:"
echo "   mjpython examples/demo_emotion_fixed.py --emotion $DEMO_NAME --duration $DURATION"
