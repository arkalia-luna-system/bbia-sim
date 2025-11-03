#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VENV_PY="$REPO_ROOT/venv/bin/python"

OUT_DIR="$REPO_ROOT/log"
OUT_FILE="$OUT_DIR/bandit_report_run_round2.json"

mkdir -p "$OUT_DIR"

# ⚠️ NETTOYAGE AUTOMATIQUE : Supprimer les fichiers macOS cachés avant Bandit
# Ces fichiers causent des erreurs de parsing et bloquent l'exécution
echo "🧹 Nettoyage fichiers macOS cachés..."
find "$REPO_ROOT/src" -name "._*" -type f -delete 2>/dev/null || true
find "$REPO_ROOT/tests" -name "._*" -type f -delete 2>/dev/null || true
echo "✅ Nettoyage terminé"

# Bandit en mode silencieux: sortie JSON, avertissements sur stderr supprimés
# Exclure les fichiers macOS (._*) et utiliser la config .bandit
"$VENV_PY" -m bandit \
  -r "$REPO_ROOT/src" \
  -x "$REPO_ROOT/tests" \
  -c "$REPO_ROOT/.bandit" \
  -x ".*|._*|*.pyc" \
  -q \
  -f json \
  -o "$OUT_FILE" \
  2>/dev/null || true

echo "Bandit terminé. Rapport: $OUT_FILE"


