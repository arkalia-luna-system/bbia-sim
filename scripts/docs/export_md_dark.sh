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
  :root {
    color-scheme: dark;
    --bg-dark: #0a0a0a;
    --bg-card: #151515;
    --text-primary: #f5f5f5;
    --text-secondary: #b3b3b3;
    --accent: #5cb3ff;
    --accent-hover: #6fc0ff;
    --border: #2a2a2a;
    --border-light: #1f1f1f;
    --shadow: rgba(0, 0, 0, 0.3);
  }
  * { box-sizing: border-box; }
  html, body {
    height: 100%;
    margin: 0;
    padding: 0;
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Inter', Oxygen, Ubuntu, Cantarell, 'Helvetica Neue', Arial, sans-serif;
    background: linear-gradient(180deg, var(--bg-dark) 0%, #0d0d0d 100%);
    color: var(--text-primary);
    -webkit-font-smoothing: antialiased;
    -moz-osx-font-smoothing: grayscale;
  }
  .container {
    max-width: 1024px;
    margin: 60px auto;
    padding: 0 32px;
    line-height: 1.75;
  }
  h1, h2, h3, h4, h5, h6 {
    color: var(--text-primary);
    margin-top: 32px;
    margin-bottom: 16px;
    font-weight: 600;
    letter-spacing: -0.01em;
  }
  h1 {
    font-size: 2.8em;
    font-weight: 700;
    border-bottom: 1px solid var(--border-light);
    padding-bottom: 20px;
    margin-top: 0;
    margin-bottom: 32px;
    letter-spacing: -0.02em;
  }
  h2 {
    font-size: 2em;
    border-bottom: 1px solid var(--border-light);
    padding-bottom: 12px;
    margin-top: 56px;
    margin-bottom: 20px;
  }
  h3 { font-size: 1.5em; margin-top: 40px; margin-bottom: 16px; }
  p {
    color: var(--text-primary);
    margin: 20px 0;
    line-height: 1.8;
  }
  a {
    color: var(--accent);
    text-decoration: none;
    border-bottom: 1px solid transparent;
    transition: all 0.2s ease;
  }
  a:hover {
    color: var(--accent-hover);
    border-bottom-color: var(--accent);
  }
  code {
    background: rgba(255, 255, 255, 0.08);
    padding: 3px 8px;
    border-radius: 4px;
    font-size: 0.9em;
    color: var(--accent);
    font-family: 'SF Mono', 'Monaco', 'Menlo', 'Consolas', monospace;
  }
  pre {
    background: var(--bg-card);
    padding: 20px;
    border-radius: 12px;
    overflow-x: auto;
    border: 1px solid var(--border-light);
    box-shadow: inset 0 1px 3px rgba(0, 0, 0, 0.3);
    white-space: pre-wrap;
  }
  pre code {
    background: none;
    padding: 0;
    color: var(--text-primary);
  }
  hr {
    border: none;
    height: 1px;
    background: var(--border-light);
    margin: 32px 0;
  }
  li, td, th, span, div:not(.mermaid) {
    color: var(--text-primary);
  }
  table {
    border-collapse: collapse;
    width: 100%;
    margin: 24px 0;
    border-radius: 8px;
    overflow: hidden;
    border: 1px solid var(--border-light);
  }
  table th, table td {
    border: 1px solid var(--border-light);
    padding: 12px 16px;
    color: var(--text-primary);
  }
  table th {
    background: var(--bg-card);
    font-weight: 600;
    text-transform: uppercase;
    font-size: 0.85em;
    letter-spacing: 0.05em;
  }
  table tr:hover {
    background: rgba(255, 255, 255, 0.02);
  }
  .mermaid {
    margin: 32px 0;
    display: flex;
    justify-content: center;
    padding: 24px;
    background: rgba(255, 255, 255, 0.02);
    border-radius: 12px;
    border: 1px solid var(--border-light);
  }
  blockquote {
    border-left: 4px solid var(--accent);
    padding-left: 20px;
    margin: 24px 0;
    color: var(--text-secondary);
    background: rgba(255, 255, 255, 0.02);
    padding: 16px 20px;
    border-radius: 4px;
  }
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
      return \`<div class=\"mermaid\">\${code}</div>\`;
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
