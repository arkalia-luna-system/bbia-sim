#!/usr/bin/env bash
set -euo pipefail

# Exporte docs/presentation/PORTFOLIO_ONEPAGER.md en PDF si pypandoc/pandoc est dispo, sinon en HTML sombre rendu (marked+mermaid).

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
    echo "[export] Échec PDF CLI, génération HTML rendu (marked+mermaid)…"
  }
  if [ -f "$PDF_OUT" ]; then
    echo "[export] Terminé. Voir $PDF_OUT"
    exit 0
  fi
fi

# 3) Fallback HTML sombre rendu (marked + mermaid)
TMP_JSON=$(mktemp)
INPUT_MD_PATH="$INPUT_MD" python - <<'PY' > "$TMP_JSON"
import os, json
p = os.environ['INPUT_MD_PATH']
print(json.dumps(open(p,'r',encoding='utf-8').read()))
PY
MD_CONTENT_ESCAPED=$(cat "$TMP_JSON")
rm -f "$TMP_JSON"

cat > "$HTML_OUT" <<HTML
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>PORTFOLIO_ONEPAGER</title>
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
<script src="https://cdn.jsdelivr.net/npm/marked/marked.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/mermaid/dist/mermaid.min.js"></script>
<script>
  mermaid.initialize({ startOnLoad: false, theme: 'dark' });
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
  mermaid.run({ querySelector: '.mermaid' });
</script>
</body>
</html>
HTML

echo "[export] Terminé. Voir $HTML_OUT"
