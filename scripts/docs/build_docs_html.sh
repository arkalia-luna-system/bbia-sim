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
    
    # Calculer le chemin relatif vers styles.css et index.html
    local html_dir=$(dirname "$html_file")
    local html_dir_rel="${html_dir#$OUT_DIR/}"
    local depth=""
    if [ "$html_dir_rel" != "" ] && [ "$html_dir_rel" != "." ]; then
        # Compter le nombre de niveaux de profondeur
        local levels=$(echo "$html_dir_rel" | tr '/' '\n' | wc -l | tr -d ' ')
        depth=$(printf '../%.0s' $(seq 1 "$levels"))
    fi
    
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
<script src="https://cdn.jsdelivr.net/npm/marked/marked.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
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
  --bg-dark: #000000;
  --bg-card: #1a1a1a;
  --text-primary: #ffffff;
  --text-secondary: #cccccc;
  --accent: #4a9eff;
  --border: #333333;
  color-scheme: dark;
}

* {
  box-sizing: border-box;
}

html, body {
  height: 100% !important;
  margin: 0 !important;
  padding: 0 !important;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Helvetica Neue', Arial, sans-serif;
  background-color: #000000 !important;
  background: #000000 !important;
  color: #ffffff !important;
}

body {
  background-color: #000000 !important;
  background: #000000 !important;
  color: #ffffff !important;
}

.docs-container {
  display: flex;
  min-height: 100vh;
}

.sidebar {
  width: 260px;
  background: var(--bg-card);
  border-right: 1px solid var(--border);
  padding: 24px;
  position: fixed;
  height: 100vh;
  overflow-y: auto;
}

.sidebar nav ul {
  list-style: none;
  padding: 0;
  margin: 12px 0;
}

.sidebar nav ul li {
  margin: 6px 0;
}

.sidebar nav a {
  color: var(--text-secondary);
  text-decoration: none;
  display: block;
  padding: 6px 12px;
  border-radius: 6px;
  transition: all 0.2s;
}

.sidebar nav a:hover {
  background: var(--bg-dark);
  color: var(--text-primary);
}

.content {
  margin-left: 260px;
  flex: 1;
  padding: 48px;
  max-width: 980px;
  background-color: #000000 !important;
  background: #000000 !important;
  color: #ffffff !important;
}

.docs-container {
  background-color: #000000 !important;
  background: #000000 !important;
}

.markdown-body {
  line-height: 1.6;
  background-color: #000000 !important;
  background: #000000 !important;
  color: #ffffff !important;
}

.markdown-body h1,
.markdown-body h1 * {
  font-size: 2.5em;
  border-bottom: 2px solid var(--border);
  padding-bottom: 12px;
  margin-top: 0;
  color: #ffffff !important;
}

.markdown-body h2,
.markdown-body h2 * {
  font-size: 2em;
  margin-top: 48px;
  border-bottom: 1px solid var(--border);
  padding-bottom: 8px;
  color: #ffffff !important;
}

.markdown-body h3,
.markdown-body h3 *,
.markdown-body h4,
.markdown-body h4 *,
.markdown-body h5,
.markdown-body h5 *,
.markdown-body h6,
.markdown-body h6 * {
  color: #ffffff !important;
}

.markdown-body h3 {
  font-size: 1.5em;
  margin-top: 32px;
}

.markdown-body code {
  background: rgba(110, 118, 129, 0.4);
  padding: 2px 6px;
  border-radius: 3px;
  font-size: 0.9em;
}

.markdown-body pre {
  background: var(--bg-card);
  padding: 16px;
  border-radius: 8px;
  overflow-x: auto;
  border: 1px solid var(--border);
}

.markdown-body pre code {
  background: none;
  padding: 0;
}

.markdown-body a {
  color: var(--accent);
  text-decoration: none;
}

.markdown-body a:hover {
  text-decoration: underline;
}

.markdown-body table {
  border-collapse: collapse;
  width: 100%;
  margin: 16px 0;
}

.markdown-body table th,
.markdown-body table td {
  border: 1px solid var(--border);
  padding: 8px 12px;
  text-align: left;
}

.markdown-body table th {
  background: var(--bg-card);
  font-weight: 600;
}

.markdown-body .mermaid {
  margin: 24px 0;
  display: flex;
  justify-content: center;
}

.markdown-body .anchor {
  text-decoration: none;
  transition: opacity 0.2s;
}

/* Forcer tous les éléments texte en blanc sur fond noir */
.markdown-body p,
.markdown-body p *,
.markdown-body li,
.markdown-body li *,
.markdown-body td,
.markdown-body td *,
.markdown-body th,
.markdown-body th *,
.markdown-body span,
.markdown-body span *,
.markdown-body div:not(.mermaid),
.markdown-body div:not(.mermaid) *,
.markdown-body ul,
.markdown-body ul *,
.markdown-body ol,
.markdown-body ol *,
.markdown-body blockquote,
.markdown-body blockquote * {
  color: #ffffff !important;
}

.markdown-body table th,
.markdown-body table td {
  color: #ffffff !important;
}

@media (max-width: 768px) {
  .sidebar {
    transform: translateX(-100%);
    transition: transform 0.3s;
  }
  .content {
    margin-left: 0;
  }
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
