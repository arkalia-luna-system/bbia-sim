#!/usr/bin/env bash
set -euo pipefail

# Usage: scripts/docs/export_md_dark.sh INPUT_MD [OUTPUT_HTML]
# Exporte un Markdown vers HTML avec thème sombre (fallback sans pandoc/pypandoc).

if [ $# -lt 1 ]; then
  echo "Usage: $0 INPUT_MD [OUTPUT_HTML]" >&2
  exit 2
fi

INPUT_MD="$1"
if [ ! -f "$INPUT_MD" ]; then
  echo "Fichier introuvable: $INPUT_MD" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
OUT_DIR="$ROOT_DIR/artifacts"
BASENAME="$(basename "$INPUT_MD" .md)"
OUT_HTML="${2:-$OUT_DIR/${BASENAME}.html}"

mkdir -p "$OUT_DIR"

# Essayer pypandoc (venv), puis pandoc CLI, sinon fallback HTML sombre
if "$ROOT_DIR/venv/bin/python" -c "import sys,importlib; sys.exit(0 if importlib.util.find_spec('pypandoc') else 1)" 2>/dev/null; then
  echo "[export] pypandoc détecté — tentative conversion HTML…"
  if "$ROOT_DIR/venv/bin/python" - <<'PY'
import sys
try:
    import pypandoc
    inp, outp = sys.argv[1], sys.argv[2]
    html = pypandoc.convert_file(inp, 'html', extra_args=['--standalone'])
    with open(outp, 'w', encoding='utf-8') as f:
        f.write(html)
    print('[export] HTML via pypandoc OK')
except Exception as e:
    print('[export] Échec pypandoc:', e)
    sys.exit(1)
PY
  "$INPUT_MD" "$OUT_HTML"; then
    echo "[export] Terminé: $OUT_HTML"
    exit 0
  else
    echo "[export] pypandoc a échoué, fallback HTML sombre…"
  fi
elif command -v pandoc >/dev/null 2>&1; then
  echo "[export] pandoc détecté — tentative conversion HTML…"
  if pandoc "$INPUT_MD" -o "$OUT_HTML" --from markdown --standalone; then
    echo "[export] Terminé: $OUT_HTML"
    exit 0
  else
    echo "[export] pandoc a échoué, fallback HTML sombre…"
  fi
fi

# Fallback HTML sombre
cat > "$OUT_HTML" <<'HTML'
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<title>BBIA DOC</title>
<style>
  :root { color-scheme: dark; }
  html, body { height: 100%; margin: 0; }
  body { background-color: #0b0b0b; color: #f5f5f5; font: 16px/1.5 -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Oxygen, Ubuntu, Cantarell, Helvetica Neue, Arial, "Noto Sans"; }
  a { color: #9cdcfe; }
  code, pre { background: #111; color: #eee; }
  pre { white-space: pre-wrap; padding: 16px; border-radius: 8px; overflow-x: auto; }
  .container { max-width: 960px; margin: 24px auto; padding: 0 16px; }
  h1, h2, h3 { color: #ffffff; }
  hr { border: none; height: 1px; background: #222; }
</style>
</head>
<body>
<div class="container">
<pre>
HTML
sed 's/&/\&/g; s/</\&lt;/g; s/>/\&gt;/g' "$INPUT_MD" >> "$OUT_HTML"
cat >> "$OUT_HTML" <<'HTML'
</pre>
</div>
</body>
</html>
HTML

echo "[export] Terminé (fallback sombre): $OUT_HTML"
