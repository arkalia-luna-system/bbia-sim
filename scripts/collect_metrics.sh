#!/bin/bash
# Script de collecte de métriques BBIA-SIM avec arkalia-metrics-collector
# Collecte uniquement les fichiers pertinents (src/ et tests/)

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

echo "📊 Collecte des métriques BBIA-SIM avec arkalia-metrics-collector..."

# Créer un répertoire temporaire avec seulement les fichiers pertinents
TEMP_DIR=$(mktemp -d)
trap "rm -rf $TEMP_DIR" EXIT

echo "📁 Copie des fichiers pertinents..."
# Copier uniquement src/ et tests/
cp -r src "$TEMP_DIR/"
cp -r tests "$TEMP_DIR/"
cp pyproject.toml "$TEMP_DIR/" 2>/dev/null || true
cp README.md "$TEMP_DIR/" 2>/dev/null || true

# Créer le répertoire de sortie
mkdir -p metrics

echo "🔍 Collecte des métriques..."
# Collecter les métriques depuis le répertoire temporaire
arkalia-metrics collect "$TEMP_DIR" \
  --output metrics \
  --format all \
  --validate

# Si coverage.xml existe, l'utiliser pour les métriques de coverage
if [ -f coverage.xml ]; then
  echo "📊 Utilisation de coverage.xml pour les métriques de coverage..."
  # arkalia-metrics peut utiliser coverage.xml automatiquement
fi

echo "✅ Métriques collectées dans metrics/"
echo "📄 Fichiers générés:"
ls -lh metrics/*.json metrics/*.md 2>/dev/null | grep -v "^\._" || true

