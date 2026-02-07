#!/bin/bash
# Nettoie le cache / état Cursor (state.vscdb) pour réduire la lenteur.
# À lancer APRÈS avoir fermé Cursor complètement (Cmd+Q).

set -e
CURSOR_GLOBAL="$HOME/Library/Application Support/Cursor/User/globalStorage"

if pgrep -x "Cursor" >/dev/null 2>&1 || pgrep -f "Cursor.app" >/dev/null 2>&1; then
  echo "❌ Cursor est encore ouvert. Ferme Cursor (Cmd+Q) puis relance ce script."
  exit 1
fi

if [[ ! -f "$CURSOR_GLOBAL/state.vscdb" ]]; then
  echo "⚠️  Fichier state.vscdb introuvable (déjà nettoyé ou chemin différent)."
  exit 0
fi

echo "📁 Dossier: $CURSOR_GLOBAL"
echo "📦 Renommage de state.vscdb et state.vscdb.backup..."

mv "$CURSOR_GLOBAL/state.vscdb" "$CURSOR_GLOBAL/state.vscdb.old" 2>/dev/null && echo "   ✓ state.vscdb → state.vscdb.old" || echo "   ⚠️ state.vscdb: échec (fichier verrouillé?)"
mv "$CURSOR_GLOBAL/state.vscdb.backup" "$CURSOR_GLOBAL/state.vscdb.backup.old" 2>/dev/null && echo "   ✓ state.vscdb.backup → state.vscdb.backup.old" || echo "   ⚠️ state.vscdb.backup: échec (fichier verrouillé?)"

# Fichiers SQLite associés (optionnel)
[[ -f "$CURSOR_GLOBAL/state.vscdb-shm" ]] && mv "$CURSOR_GLOBAL/state.vscdb-shm" "$CURSOR_GLOBAL/state.vscdb-shm.old" 2>/dev/null && echo "   ✓ state.vscdb-shm → .old"
[[ -f "$CURSOR_GLOBAL/state.vscdb-wal" ]] && mv "$CURSOR_GLOBAL/state.vscdb-wal" "$CURSOR_GLOBAL/state.vscdb-wal.old" 2>/dev/null && echo "   ✓ state.vscdb-wal → .old"

echo ""
echo "✅ Nettoyage terminé. Tu peux rouvrir Cursor."
echo "   Si tout va bien, tu pourras supprimer les fichiers .old plus tard pour libérer de l’espace."
echo "   (Dans le Finder: Aller au dossier → coller le chemin ci-dessus)"
