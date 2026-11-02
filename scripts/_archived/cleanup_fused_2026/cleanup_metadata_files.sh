#!/bin/bash
# 🧹 Script pour supprimer tous les fichiers de métadonnées macOS
# Ces fichiers ._* sont créés automatiquement par macOS et ne doivent pas être commités

echo "🧹 Nettoyage des fichiers de métadonnées macOS..."
echo "=================================================="

# Compter fichiers avant suppression
count_before=$(find . -name "._*" -type f ! -path "./venv/*" ! -path "./venv-*/*" ! -path "./dist/*" ! -path "./build/*" 2>/dev/null | wc -l | tr -d ' ')

if [ "$count_before" -eq 0 ]; then
    echo "✅ Aucun fichier de métadonnées trouvé"
    exit 0
fi

echo "📊 Fichiers trouvés: $count_before"

# Supprimer fichiers ._* (métadonnées macOS standards)
find . -name "._*" -type f ! -path "./venv/*" ! -path "./venv-*/*" ! -path "./dist/*" ! -path "./build/*" -delete 2>/dev/null

# Supprimer aussi fichiers .!*!._* (métadonnées macOS sur disque réseau/externe)
find . -name ".!*!._*" -type f ! -path "./venv/*" ! -path "./venv-*/*" ! -path "./dist/*" ! -path "./build/*" -delete 2>/dev/null

# Supprimer aussi .DS_Store
find . -name ".DS_Store" -type f -delete 2>/dev/null

count_after=$(find . -name "._*" -type f ! -path "./venv/*" ! -path "./venv-*/*" ! -path "./dist/*" ! -path "./build/*" 2>/dev/null | wc -l | tr -d ' ')

echo "✅ Fichiers supprimés: $((count_before - count_after))"
echo "📝 Fichiers restants (dans venv/build uniquement): $count_after"

# Afficher avertissement si fichiers trackés dans git
tracked=$(git ls-files | grep "^\._" 2>/dev/null | wc -l | tr -d ' ')
if [ "$tracked" -gt 0 ]; then
    echo ""
    echo "⚠️  ATTENTION: $tracked fichiers ._* sont trackés dans git!"
    echo "   Exécutez: git rm --cached '**/._*' pour les retirer"
fi

