#!/usr/bin/env bash
set -euo pipefail

# Exporte docs/presentation/PORTFOLIO_ONEPAGER.md en PDF si pandoc est dispo, sinon en HTML.

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
INPUT_MD="$ROOT_DIR/docs/presentation/PORTFOLIO_ONEPAGER.md"
OUT_DIR="$ROOT_DIR/artifacts"
PDF_OUT="$OUT_DIR/PORTFOLIO_ONEPAGER.pdf"
HTML_OUT="$OUT_DIR/PORTFOLIO_ONEPAGER.html"

mkdir -p "$OUT_DIR"

if command -v pandoc >/dev/null 2>&1; then
  echo "[export] Génération PDF via pandoc…"
  pandoc "$INPUT_MD" -o "$PDF_OUT" --from markdown --pdf-engine=xelatex || {
    echo "[export] Échec PDF, génération HTML fallback…"
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
