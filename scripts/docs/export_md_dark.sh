#!/usr/bin/env bash
set -euo pipefail

# Usage: scripts/docs/export_md_dark.sh INPUT_MD [OUTPUT_HTML]
# Exporte un Markdown vers HTML sombre et rend les blocs mermaid (client-side via fichiers locaux, 100% offline).

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
JS_DIR="$OUT_DIR/js"

# Télécharger les JS si manquants (100% offline)
mkdir -p "$JS_DIR"
[ ! -f "$JS_DIR/marked.min.js" ] && curl -sL https://cdn.jsdelivr.net/npm/marked/marked.min.js -o "$JS_DIR/marked.min.js" || true
[ ! -f "$JS_DIR/mermaid.min.js" ] && curl -sL https://cdn.jsdelivr.net/npm/mermaid/dist/mermaid.min.js -o "$JS_DIR/mermaid.min.js" || true

# Essayer pypandoc (venv), puis pandoc CLI, sinon fallback HTML sombre dynamique
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
    echo "[export] pypandoc a échoué, fallback HTML sombre dynamique…"
  fi
elif command -v pandoc >/dev/null 2>&1; then
  echo "[export] pandoc détecté — tentative conversion HTML…"
  if pandoc "$INPUT_MD" -o "$OUT_HTML" --from markdown --standalone; then
    echo "[export] Terminé: $OUT_HTML"
    exit 0
  else
    echo "[export] pandoc a échoué, fallback HTML sombre dynamique…"
  fi
fi

# Fallback HTML sombre dynamique (Markdown + Mermaid rendus côté client)
TMP_JSON=$(mktemp)
INPUT_MD_PATH="$INPUT_MD" python - <<'PY' > "$TMP_JSON"
import os, json
p = os.environ['INPUT_MD_PATH']
print(json.dumps(open(p,'r',encoding='utf-8').read()))
PY
MD_CONTENT_ESCAPED=$(cat "$TMP_JSON")
rm -f "$TMP_JSON"

# Chemins relatifs depuis OUT_HTML vers JS_DIR
TMP_REL=$(mktemp)
JS_DIR_VAR="$JS_DIR" OUT_DIR_VAR="$(dirname "$OUT_HTML")" python - <<'PY' > "$TMP_REL"
import os
jdir = os.environ['JS_DIR_VAR']
odir = os.environ['OUT_DIR_VAR']
print(os.path.relpath(jdir, odir))
PY
REL_JS_DIR=$(cat "$TMP_REL")
rm -f "$TMP_REL"

cat > "$OUT_HTML" <<HTML
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>${BASENAME}</title>
<style>
  :root { color-scheme: dark; }
  html, body { height: 100%; margin: 0; }
  body { background-color: #0b0b0b; color: #f5f5f5; font: 16px/1.55 -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Oxygen, Ubuntu, Cantarell, Helvetica Neue, Arial, "Noto Sans"; }
  a { color: #9cdcfe; }
  code, pre { background: #111; color: #eee; }
  pre { white-space: pre-wrap; padding: 14px; border-radius: 8px; overflow-x: auto; }
  .container { max-width: 980px; margin: 28px auto; padding: 0 16px; }
  h1, h2, h3 { color: #ffffff; }
  hr { border: none; height: 1px; background: #222; }
</style>
</head>
<body>
<div class="container" id="app">Chargement…</div>
<script>
  window.__RAW_MD__ = ${MD_CONTENT_ESCAPED};
</script>
<script src="${REL_JS_DIR}/marked.min.js"></script>
<script src="${REL_JS_DIR}/mermaid.min.js"></script>
<script>
  mermaid.initialize({ startOnLoad: false, theme: 'dark' });
  // Personnaliser le rendu pour les blocs mermaid
  const renderer = new marked.Renderer();
  const origCode = renderer.code.bind(renderer);
  renderer.code = (code, infostring, escaped) => {
    const lang = (infostring || '').trim().toLowerCase();
    if (lang === 'mermaid') {
      return `<div class=\"mermaid\">${code}</div>`;
    }
    return origCode(code, infostring, escaped);
  };
  const html = marked.parse(window.__RAW_MD__, { renderer });
  document.getElementById('app').innerHTML = html;
  // Rendre mermaid après insertion DOM
  mermaid.run({ querySelector: '.mermaid' });
</script>
</body>
</html>
HTML

echo "[export] Terminé (fallback sombre + mermaid, 100% offline): $OUT_HTML"
