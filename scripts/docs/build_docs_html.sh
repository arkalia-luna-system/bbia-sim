#!/usr/bin/env bash
set -euo pipefail

# Script pour générer une documentation HTML professionnelle complète
# Usage: scripts/docs/build_docs_html.sh [OUTPUT_DIR]

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
DOCS_DIR="$ROOT_DIR/docs"
OUT_DIR="${1:-$ROOT_DIR/artifacts/docs_html}"
INDEX_MD="$DOCS_DIR/INDEX_FINAL.md"
MAIN_MD="$DOCS_DIR/README.md"

mkdir -p "$OUT_DIR"
JS_DIR="$OUT_DIR/js"

# Télécharger les JS si manquants (100% offline)
mkdir -p "$JS_DIR"
[ ! -f "$JS_DIR/marked.min.js" ] && curl -sL https://cdn.jsdelivr.net/npm/marked/marked.min.js -o "$JS_DIR/marked.min.js" || true
[ ! -f "$JS_DIR/mermaid.min.js" ] && curl -sL https://cdn.jsdelivr.net/npm/mermaid/dist/mermaid.min.js -o "$JS_DIR/mermaid.min.js" || true

# Fonction pour convertir un MD en HTML avec Mermaid
convert_md_to_html() {
    local md_file="$1"
    local html_file="$2"
    local basename="$(basename "$md_file" .md)"
    
    if [ ! -f "$md_file" ]; then
        echo "[build] ⚠️  Fichier introuvable: $md_file" >&2
        return 1
    fi
    
    # Calculer le chemin relatif vers styles.css, index.html et js/
    local html_dir=$(dirname "$html_file")
    local html_dir_rel="${html_dir#$OUT_DIR/}"
    local depth=""
    if [ "$html_dir_rel" != "" ] && [ "$html_dir_rel" != "." ]; then
        # Compter le nombre de niveaux de profondeur
        local levels=$(echo "$html_dir_rel" | tr '/' '\n' | wc -l | tr -d ' ')
        depth=$(printf '../%.0s' $(seq 1 "$levels"))
    fi
    # Calculer chemin relatif vers js/
    TMP_REL_JS=$(mktemp)
    JS_DIR_VAR="$JS_DIR" OUT_DIR_VAR="$html_dir" python3 - <<'PY' > "$TMP_REL_JS"
import os
jdir = os.environ['JS_DIR_VAR']
odir = os.environ['OUT_DIR_VAR']
print(os.path.relpath(jdir, odir))
PY
    local js_rel_path=$(cat "$TMP_REL_JS")
    rm -f "$TMP_REL_JS"
    
    local md_content
    md_content=$(python3 -c "import json, sys; print(json.dumps(open(sys.argv[1], 'r', encoding='utf-8').read()))" "$md_file")
    
    cat > "$html_file" <<HTML
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1" />
<meta name="description" content="Documentation BBIA-SIM - Robot Reachy Mini avec intelligence artificielle" />
<meta name="theme-color" content="#000000" />
<title>${basename} - BBIA-SIM Documentation</title>
<style>
/* Style inline pour forcer fond noir */
html, body {
  background-color: #000000 !important;
  background: #000000 !important;
  color: #ffffff !important;
}
body {
  margin: 0;
  padding: 0;
}
</style>
<link rel="stylesheet" href="${depth}styles.css">
</head>
<body style="background-color: #000000 !important; color: #ffffff !important;">
<div class="docs-container" style="background-color: #000000 !important;">
<div class="sidebar">
<nav>
<h2>📚 Navigation</h2>
<ul>
<li><a href="${depth}index.html">🏠 Accueil</a></li>
<li><a href="${depth}README.html">📖 README</a></li>
<li><a href="${depth}guides/guide_debutant.html">🚀 Guide Débutant</a></li>
<li><a href="${depth}architecture/architecture_overview.html">🏗️ Architecture</a></li>
<li><a href="${depth}guides_techniques/integration_guide.html">🌐 Intégration</a></li>
<li><a href="${depth}guides_techniques/testing_guide.html">🧪 Tests</a></li>
</ul>
<h3>Ressources</h3>
<ul>
<li><a href="https://github.com/arkalia-luna-system/bbia-sim">GitHub</a></li>
<li><a href="http://localhost:8000/docs">API Swagger</a></li>
</ul>
</nav>
</div>
<div class="content" id="app">
<div class="markdown-body" style="color: #ffffff; background-color: #000000;">
  <div style="text-align: center; padding: 40px; color: #ffffff;">
    <p>Chargement de la documentation…</p>
  </div>
</div>
</div>
</div>
<script>
  window.__RAW_MD__ = ${md_content};
</script>
<script src="${js_rel_path}/marked.min.js"></script>
<script src="${js_rel_path}/mermaid.min.js"></script>
<script>
  mermaid.initialize({ 
    startOnLoad: false, 
    theme: 'dark',
    themeVariables: {
      primaryColor: '#4a9eff',
      primaryTextColor: '#f5f5f5',
      primaryBorderColor: '#6bb6ff'
    }
  });
  const renderer = new marked.Renderer();
  const origCode = renderer.code.bind(renderer);
  renderer.code = (code, infostring, escaped) => {
    const lang = (infostring || '').trim().toLowerCase();
    if (lang === 'mermaid') {
      return '<div class="mermaid">' + code + '</div>';
    }
    return origCode(code, infostring, escaped);
  };
  const html = marked.parse(window.__RAW_MD__, { renderer, gfm: true });
  const markdownBody = document.querySelector('.markdown-body');
  markdownBody.innerHTML = html;
  markdownBody.style.backgroundColor = '#000000';
  markdownBody.style.color = '#ffffff';
  
  // Forcer fond noir et texte blanc sur tous les éléments générés
  document.querySelectorAll('.markdown-body *').forEach(el => {
    if (!el.classList.contains('mermaid') && !el.querySelector('.mermaid')) {
      const tag = el.tagName.toLowerCase();
      if (['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'td', 'th', 'span', 'div', 'a'].includes(tag)) {
        el.style.color = '#ffffff';
      }
    }
  });
  
  mermaid.run({ querySelector: '.mermaid' });
  
  // Lien automatique des ancres
  document.querySelectorAll('h1, h2, h3, h4').forEach(h => {
    if (!h.id) {
      h.id = h.textContent.toLowerCase().replace(/[^\w]+/g, '-');
    }
    const a = document.createElement('a');
    a.href = '#' + h.id;
    a.className = 'anchor';
    a.textContent = '#';
    a.style.opacity = '0';
    a.style.marginLeft = '8px';
    a.style.color = '#ffffff';
    h.appendChild(a);
    h.addEventListener('mouseenter', () => { a.style.opacity = '1'; });
    h.addEventListener('mouseleave', () => { a.style.opacity = '0'; });
  });
  
  // Améliorer les liens dans le contenu pour qu'ils soient visibles
  document.querySelectorAll('.markdown-body a').forEach(a => {
    if (!a.style.color) {
      a.style.color = '#4a9eff';
    }
  });
</script>
</body>
</html>
HTML
}

# Générer le CSS principal
cat > "$OUT_DIR/styles.css" <<CSS
:root {
  --bg-dark: #0a0a0a;
  --bg-card: #151515;
  --bg-sidebar: #121212;
  --text-primary: #f5f5f5;
  --text-secondary: #b3b3b3;
  --text-tertiary: #888888;
  --accent: #5cb3ff;
  --accent-hover: #6fc0ff;
  --border: #2a2a2a;
  --border-light: #1f1f1f;
  --shadow: rgba(0, 0, 0, 0.3);
  color-scheme: dark;
}

* {
  box-sizing: border-box;
}

html, body {
  height: 100% !important;
  margin: 0 !important;
  padding: 0 !important;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Inter', Oxygen, Ubuntu, Cantarell, 'Helvetica Neue', Arial, sans-serif;
  background-color: var(--bg-dark) !important;
  background: linear-gradient(180deg, var(--bg-dark) 0%, #0d0d0d 100%) !important;
  color: var(--text-primary) !important;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
}

body {
  background-color: var(--bg-dark) !important;
  background: linear-gradient(180deg, var(--bg-dark) 0%, #0d0d0d 100%) !important;
  color: var(--text-primary) !important;
}

.docs-container {
  display: flex;
  min-height: 100vh;
}

.sidebar {
  width: 280px;
  background: var(--bg-sidebar);
  border-right: 1px solid var(--border-light);
  padding: 32px 24px;
  position: fixed;
  height: 100vh;
  overflow-y: auto;
  backdrop-filter: blur(10px);
  box-shadow: 2px 0 20px var(--shadow);
}

.sidebar::-webkit-scrollbar {
  width: 6px;
}

.sidebar::-webkit-scrollbar-track {
  background: transparent;
}

.sidebar::-webkit-scrollbar-thumb {
  background: var(--border);
  border-radius: 3px;
}

.sidebar::-webkit-scrollbar-thumb:hover {
  background: var(--border-light);
}

.sidebar nav h2, .sidebar nav h3 {
  color: var(--text-primary);
  font-size: 0.85em;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  margin: 24px 0 12px 0;
  padding: 0 12px;
  opacity: 0.7;
}

.sidebar nav h2:first-child {
  margin-top: 0;
}

.sidebar nav ul {
  list-style: none;
  padding: 0;
  margin: 0 0 24px 0;
}

.sidebar nav ul li {
  margin: 2px 0;
}

.sidebar nav a {
  color: var(--text-secondary);
  text-decoration: none;
  display: block;
  padding: 10px 12px;
  border-radius: 8px;
  transition: all 0.2s ease;
  font-size: 0.95em;
  border-left: 2px solid transparent;
}

.sidebar nav a:hover {
  background: rgba(255, 255, 255, 0.03);
  color: var(--text-primary);
  border-left-color: var(--accent);
  padding-left: 14px;
  transform: translateX(2px);
}

.content {
  margin-left: 280px;
  flex: 1;
  padding: 60px 64px;
  max-width: 1024px;
  background-color: transparent !important;
  color: var(--text-primary) !important;
}

.docs-container {
  background-color: transparent !important;
}

.markdown-body {
  line-height: 1.75;
  background-color: transparent !important;
  color: var(--text-primary) !important;
  font-size: 16px;
}

.markdown-body h1,
.markdown-body h1 * {
  font-size: 2.8em;
  font-weight: 700;
  border-bottom: 1px solid var(--border);
  padding-bottom: 20px;
  margin-top: 0;
  margin-bottom: 32px;
  color: var(--text-primary) !important;
  letter-spacing: -0.02em;
}

.markdown-body h2,
.markdown-body h2 * {
  font-size: 2em;
  font-weight: 600;
  margin-top: 56px;
  margin-bottom: 20px;
  border-bottom: 1px solid var(--border-light);
  padding-bottom: 12px;
  color: var(--text-primary) !important;
  letter-spacing: -0.01em;
}

.markdown-body h3,
.markdown-body h3 *,
.markdown-body h4,
.markdown-body h4 *,
.markdown-body h5,
.markdown-body h5 *,
.markdown-body h6,
.markdown-body h6 * {
  color: var(--text-primary) !important;
  font-weight: 600;
}

.markdown-body h3 {
  font-size: 1.5em;
  margin-top: 40px;
  margin-bottom: 16px;
  color: var(--text-primary) !important;
}

.markdown-body code {
  background: rgba(255, 255, 255, 0.08);
  padding: 3px 8px;
  border-radius: 4px;
  font-size: 0.9em;
  color: var(--accent);
  font-family: 'SF Mono', 'Monaco', 'Menlo', 'Consolas', monospace;
}

.markdown-body pre {
  background: var(--bg-card);
  padding: 20px;
  border-radius: 12px;
  overflow-x: auto;
  border: 1px solid var(--border-light);
  box-shadow: inset 0 1px 3px rgba(0, 0, 0, 0.3);
}

.markdown-body pre code {
  background: none;
  padding: 0;
}

.markdown-body a {
  color: var(--accent);
  text-decoration: none;
  border-bottom: 1px solid transparent;
  transition: all 0.2s ease;
}

.markdown-body a:hover {
  color: var(--accent-hover);
  border-bottom-color: var(--accent);
}

.markdown-body table {
  border-collapse: collapse;
  width: 100%;
  margin: 24px 0;
  border-radius: 8px;
  overflow: hidden;
  border: 1px solid var(--border-light);
}

.markdown-body table th,
.markdown-body table td {
  border: 1px solid var(--border-light);
  padding: 12px 16px;
  text-align: left;
  color: var(--text-primary) !important;
}

.markdown-body table th {
  background: var(--bg-card);
  font-weight: 600;
  color: var(--text-primary) !important;
  text-transform: uppercase;
  font-size: 0.85em;
  letter-spacing: 0.05em;
}

.markdown-body table tr:hover {
  background: rgba(255, 255, 255, 0.02);
}

.markdown-body .mermaid {
  margin: 32px 0;
  display: flex;
  justify-content: center;
  padding: 24px;
  background: rgba(255, 255, 255, 0.02);
  border-radius: 12px;
  border: 1px solid var(--border-light);
}

.markdown-body .anchor {
  text-decoration: none;
  transition: opacity 0.2s ease;
  color: var(--text-tertiary) !important;
  font-weight: normal;
}

.markdown-body .anchor:hover {
  color: var(--accent) !important;
}

/* Styles délicats pour les éléments texte */
.markdown-body p {
  color: var(--text-primary) !important;
  margin: 20px 0;
  line-height: 1.8;
}

.markdown-body li {
  color: var(--text-primary) !important;
  margin: 8px 0;
  line-height: 1.75;
}

.markdown-body ul, .markdown-body ol {
  margin: 20px 0;
  padding-left: 28px;
}

.markdown-body ul {
  list-style-type: disc;
}

.markdown-body ol {
  list-style-type: decimal;
}

.markdown-body li::marker {
  color: var(--text-tertiary);
}

.markdown-body blockquote {
  border-left: 3px solid var(--accent);
  padding-left: 20px;
  margin: 24px 0;
  background: rgba(255, 255, 255, 0.02);
  padding: 16px 20px;
  border-radius: 4px;
  color: var(--text-secondary) !important;
  font-style: italic;
}

.markdown-body blockquote p {
  margin: 0;
  color: var(--text-secondary) !important;
}

.markdown-body hr {
  border: none;
  height: 1px;
  background: linear-gradient(90deg, transparent, var(--border), transparent);
  margin: 40px 0;
}

/* Améliorations délicates pour mobile */
@media (max-width: 768px) {
  .sidebar {
    transform: translateX(-100%);
    transition: transform 0.3s ease;
    width: 100%;
    max-width: 320px;
    z-index: 1000;
    box-shadow: 4px 0 24px rgba(0, 0, 0, 0.5);
  }
  
  .content {
    margin-left: 0;
    padding: 32px 24px;
  }
  
  .markdown-body {
    font-size: 15px;
  }
  
  .markdown-body h1 {
    font-size: 2.2em;
  }
  
  .markdown-body h2 {
    font-size: 1.8em;
  }
  
  .markdown-body h3 {
    font-size: 1.4em;
  }
}

/* Animations subtiles au scroll */
@keyframes fadeIn {
  from {
    opacity: 0;
    transform: translateY(10px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.markdown-body > * {
  animation: fadeIn 0.4s ease-out;
}

/* Sélection de texte élégante */
::selection {
  background: rgba(92, 179, 255, 0.2);
  color: var(--text-primary);
}

::-moz-selection {
  background: rgba(92, 179, 255, 0.2);
  color: var(--text-primary);
}
CSS

echo "[build] ✅ CSS généré: $OUT_DIR/styles.css"

# Convertir les fichiers principaux
echo "[build] 🔨 Génération des pages HTML..."

# Index principal
convert_md_to_html "$INDEX_MD" "$OUT_DIR/index.html"
echo "[build] ✅ index.html"

# README docs
convert_md_to_html "$MAIN_MD" "$OUT_DIR/README.html"
echo "[build] ✅ README.html"

# Guides principaux
mkdir -p "$OUT_DIR/guides"
if [ -f "$DOCS_DIR/guides/GUIDE_DEBUTANT.md" ]; then
    convert_md_to_html "$DOCS_DIR/guides/GUIDE_DEBUTANT.md" "$OUT_DIR/guides/guide_debutant.html"
    echo "[build] ✅ guides/guide_debutant.html"
fi

# Architecture
mkdir -p "$OUT_DIR/architecture"
if [ -f "$DOCS_DIR/architecture/ARCHITECTURE_OVERVIEW.md" ]; then
    convert_md_to_html "$DOCS_DIR/architecture/ARCHITECTURE_OVERVIEW.md" "$OUT_DIR/architecture/architecture_overview.html"
    echo "[build] ✅ architecture/architecture_overview.html"
fi

# Guides techniques
mkdir -p "$OUT_DIR/guides_techniques"
if [ -f "$DOCS_DIR/guides_techniques/INTEGRATION_GUIDE.md" ]; then
    convert_md_to_html "$DOCS_DIR/guides_techniques/INTEGRATION_GUIDE.md" "$OUT_DIR/guides_techniques/integration_guide.html"
    echo "[build] ✅ guides_techniques/integration_guide.html"
fi

if [ -f "$DOCS_DIR/guides_techniques/TESTING_GUIDE.md" ]; then
    convert_md_to_html "$DOCS_DIR/guides_techniques/TESTING_GUIDE.md" "$OUT_DIR/guides_techniques/testing_guide.html"
    echo "[build] ✅ guides_techniques/testing_guide.html"
fi

echo ""
echo "[build] 🎉 Documentation HTML générée dans: $OUT_DIR"
echo "[build] 📂 Ouvrir: file://$OUT_DIR/index.html"
