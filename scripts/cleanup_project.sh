#!/bin/bash

# 🧹 Script de nettoyage et organisation du projet BBIA
# Maintient le projet propre et organisé

echo "🧹 Nettoyage du projet BBIA..."
echo "================================"

# 1. Supprimer les fichiers système macOS
echo "🗑️ Suppression des fichiers système macOS..."
find . -name "._*" -type f -delete 2>/dev/null || true

# 2. Supprimer les fichiers temporaires
echo "🗑️ Suppression des fichiers temporaires..."
rm -f reachy_commands.txt reachy_response.txt 2>/dev/null || true
rm -f *.tmp *.log 2>/dev/null || true

# 3. Nettoyer les caches Python
echo "🐍 Nettoyage des caches Python..."
find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find . -name "*.pyc" -delete 2>/dev/null || true
find . -name "*.pyo" -delete 2>/dev/null || true

# 4. Nettoyer les caches mypy
echo "🔍 Nettoyage des caches mypy..."
rm -rf .mypy_cache 2>/dev/null || true

# 5. Vérifier la structure de documentation
echo "📚 Vérification de la structure de documentation..."
if [ ! -d "docs/semaines" ]; then
    echo "   📁 Création du dossier docs/semaines..."
    mkdir -p docs/semaines
fi

if [ ! -d "docs/rapports" ]; then
    echo "   📁 Création du dossier docs/rapports..."
    mkdir -p docs/rapports
fi

if [ ! -d "docs/archives" ]; then
    echo "   📁 Création du dossier docs/archives..."
    mkdir -p docs/archives
fi

# 6. Déplacer les fichiers de documentation mal placés
echo "📋 Organisation de la documentation..."
if [ -f "📋_SEMAINE_*_*.md" ]; then
    echo "   📁 Déplacement des documents de semaine..."
    mv 📋_SEMAINE_*_*.md docs/semaines/ 2>/dev/null || true
fi

if [ -f "📋_BILAN_*.md" ] || [ -f "📋_ARRET_*.md" ] || [ -f "📋_REORGANISATION_*.md" ]; then
    echo "   📁 Déplacement des rapports..."
    mv 📋_BILAN_*.md docs/rapports/ 2>/dev/null || true
    mv 📋_ARRET_*.md docs/rapports/ 2>/dev/null || true
    mv 📋_REORGANISATION_*.md docs/rapports/ 2>/dev/null || true
fi

# 7. Afficher la structure finale
echo "📊 Structure finale du projet :"
echo "   📁 Racine :"
ls -la | grep -E "^(README|requirements|\.git)" || echo "   (fichiers de base)"
echo "   📁 docs/ :"
ls -la docs/ | grep -v "^total" || echo "   (dossier documentation)"
echo "   📁 src/ :"
ls -la src/bbia_sim/ | grep -v "^total" || echo "   (modules BBIA)"
echo "   📁 tests/ :"
ls -la tests/ | grep -v "^total" || echo "   (tests)"

echo ""
echo "✅ Nettoyage terminé !"
echo "🎯 Projet BBIA organisé et propre"
echo ""
echo "📚 Documentation : docs/README.md"
echo "🚀 Tests : python3 tests/test_vision_advanced.py" 