#!/bin/bash

# run_demo_real.sh - Script one-click pour démo robot réel
# Lance une démo via RobotAPI reachy (headless) et génère les traces

set -e

# Configuration
EMOTION=${1:-"happy"}
DURATION=${2:-15}
OUTPUT_DIR="assets/demos/real_$(date +%Y%m%d_%H%M%S)"

echo "🤖 Démo Robot Réel BBIA"
echo "======================="
echo "🎭 Émotion: $EMOTION"
echo "⏱️ Durée: ${DURATION}s"
echo "📁 Sortie: $OUTPUT_DIR"
echo "🔌 Backend: reachy"

# Créer répertoire de sortie
mkdir -p "$OUTPUT_DIR"

# 1. Test hardware dry run
echo "🔧 Test hardware dry run..."
python scripts/hardware_dry_run.py \
    --duration 5 \
    --backend reachy \
    --output "$OUTPUT_DIR/hardware_test"

# 2. Enregistrer trace robot réel (mode lent)
echo "📊 Enregistrement trace robot réel..."
python scripts/record_trace.py \
    --backend reachy \
    --emotion "$EMOTION" \
    --duration "$DURATION" \
    --joint yaw_body \
    --seed 42 \
    --slow \
    --out "$OUTPUT_DIR/trace_real.jsonl"

# 3. Générer rapport d'analyse
echo "📈 Génération rapport d'analyse..."
python scripts/plot_trace.py \
    --input "$OUTPUT_DIR/trace_real.jsonl" \
    --output "$OUTPUT_DIR/rapport_real.txt"

# 4. Comparaison avec référence (si disponible)
REFERENCE_TRACE="artifacts/golden/${EMOTION}_mujoco.jsonl"
if [ -f "$REFERENCE_TRACE" ]; then
    echo "🔍 Comparaison avec trace de référence..."
    python scripts/validate_trace.py \
        --ref "$REFERENCE_TRACE" \
        --cur "$OUTPUT_DIR/trace_real.jsonl" \
        --tol-q 0.25 \
        --tol-rate 0.20 > "$OUTPUT_DIR/comparison.txt" 2>&1 || true
    echo "📊 Comparaison sauvegardée: $OUTPUT_DIR/comparison.txt"
fi

echo ""
echo "✅ Démo robot réel terminée!"
echo "📁 Fichiers générés dans: $OUTPUT_DIR"
echo "   - Trace: $OUTPUT_DIR/trace_real.jsonl"
echo "   - Rapport: $OUTPUT_DIR/rapport_real.txt"
echo "   - Test hardware: $OUTPUT_DIR/hardware_test/"
if [ -f "$OUTPUT_DIR/comparison.txt" ]; then
    echo "   - Comparaison: $OUTPUT_DIR/comparison.txt"
fi
echo ""
echo "🎯 Pour voir le rapport:"
echo "   cat $OUTPUT_DIR/rapport_real.txt"
echo ""
echo "⚠️ Note: Ce script utilise le backend 'reachy' (mock)"
echo "   Pour le vrai robot, configurez la connexion dans RobotAPI"
