#!/usr/bin/env bash
set -euo pipefail

# Exporte docs/presentation/PORTFOLIO_ONEPAGER.md en PDF si pypandoc/pandoc est dispo, sinon en HTML.

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
INPUT_MD="$ROOT_DIR/docs/presentation/PORTFOLIO_ONEPAGER.md"
OUT_DIR="$ROOT_DIR/artifacts"
PDF_OUT="$OUT_DIR/PORTFOLIO_ONEPAGER.pdf"
HTML_OUT="$OUT_DIR/PORTFOLIO_ONEPAGER.html"

mkdir -p "$OUT_DIR"

# 1) Essayer via pypandoc (dans venv)
if "$ROOT_DIR/venv/bin/python" -c "import sys; import importlib; sys.exit(0 if importlib.util.find_spec('pypandoc') else 1)" 2>/dev/null; then
  echo "[export] Génération PDF via pypandoc…"
  if "$ROOT_DIR/venv/bin/python" - <<'PY'
import sys
try:
    import pypandoc
    pypandoc.convert_file(sys.argv[1], 'pdf', outputfile=sys.argv[2])
    print('[export] PDF OK via pypandoc')
except Exception as e:
    print('[export] Échec pypandoc:', e)
    sys.exit(1)
PY
  "$INPUT_MD" "$PDF_OUT"; then
    echo "[export] Terminé. Voir $PDF_OUT"
    exit 0
  else
    echo "[export] pypandoc a échoué, tentative pandoc CLI…"
  fi
fi

# 2) Essayer via pandoc CLI
if command -v pandoc >/dev/null 2>&1; then
  echo "[export] Génération PDF via pandoc…"
  pandoc "$INPUT_MD" -o "$PDF_OUT" --from markdown --pdf-engine=xelatex || {
    echo "[export] Échec PDF CLI, génération HTML fallback…"
    pandoc "$INPUT_MD" -o "$HTML_OUT" --from markdown --standalone
  }
else
  echo "[export] pandoc introuvable. Génération HTML…"
  cat > "$HTML_OUT" <<'HTML'
<!DOCTYPE html>
<html lang="fr"><meta charset="utf-8"><title>PORTFOLIO_ONEPAGER</title><body>
<pre>
HTML
  sed 's/&/\&/g; s/</\&lt;/g; s/>/\&gt;/g' "$INPUT_MD" >> "$HTML_OUT"
  echo -e "\n</pre>\n</body></html>" >> "$HTML_OUT"
fi

echo "[export] Terminé. Voir $OUT_DIR"
