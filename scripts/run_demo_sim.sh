#!/bin/bash

# run_demo_sim.sh - Script one-click pour démo simulation
# Lance une démo MuJoCo avec viewer 3D et génère les traces

set -e

# Configuration
EMOTION=${1:-"happy"}
DURATION=${2:-15}
OUTPUT_DIR="assets/demos/sim_$(date +%Y%m%d_%H%M%S)"

echo "🎬 Démo Simulation BBIA"
echo "========================"
echo "🎭 Émotion: $EMOTION"
echo "⏱️ Durée: ${DURATION}s"
echo "📁 Sortie: $OUTPUT_DIR"

# Créer répertoire de sortie
mkdir -p "$OUTPUT_DIR"

# 1. Enregistrer trace de référence
echo "📊 Enregistrement trace de référence..."
python scripts/record_trace.py \
    --backend mujoco \
    --emotion "$EMOTION" \
    --duration "$DURATION" \
    --joint yaw_body \
    --seed 42 \
    --out "$OUTPUT_DIR/trace_reference.jsonl"

# 2. Générer rapport d'analyse
echo "📈 Génération rapport d'analyse..."
python scripts/plot_trace.py \
    --input "$OUTPUT_DIR/trace_reference.jsonl" \
    --output "$OUTPUT_DIR/rapport.txt"

# 3. Lancement démo MuJoCo (avec viewer)
echo "🎥 Lancement démo MuJoCo..."
echo "💡 Instructions:"
echo "   - La fenêtre 3D va s'ouvrir"
echo "   - Le robot va exécuter l'émotion '$EMOTION' pendant ${DURATION}s"
echo "   - Fermez la fenêtre quand terminé"
echo ""

# Lancer la démo avec viewer
mjpython examples/demo_emotion_fixed.py \
    --emotion "$EMOTION" \
    --duration "$DURATION" \
    --intensity 0.8

echo ""
echo "✅ Démo simulation terminée!"
echo "📁 Fichiers générés dans: $OUTPUT_DIR"
echo "   - Trace: $OUTPUT_DIR/trace_reference.jsonl"
echo "   - Rapport: $OUTPUT_DIR/rapport.txt"
echo ""
echo "🎯 Pour voir le rapport:"
echo "   cat $OUTPUT_DIR/rapport.txt"
