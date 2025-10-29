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
  <header class="docs-header">
    <div class="header-content">
      <div class="logo-section">
        <div class="logo-circle"></div>
        <div class="logo-text">
          <span class="logo-title">BBIA-SIM</span>
          <span class="logo-subtitle">Documentation</span>
        </div>
      </div>
      <nav class="header-nav">
        <a href="${depth}index.html" class="nav-link">Accueil</a>
        <a href="${depth}guides/guide_debutant.html" class="nav-link">Guides</a>
        <a href="${depth}architecture/architecture_overview.html" class="nav-link">Architecture</a>
        <a href="https://github.com/arkalia-luna-system/bbia-sim" class="nav-link external" target="_blank">GitHub ↗</a>
      </nav>
    </div>
  </header>
<div class="sidebar">
<nav class="sidebar-nav">
<div class="nav-section">
  <div class="nav-title">Navigation</div>
  <ul class="nav-list">
    <li><a href="${depth}index.html" class="nav-item"><span class="nav-icon">🏠</span><span>Accueil</span></a></li>
    <li><a href="${depth}README.html" class="nav-item"><span class="nav-icon">📖</span><span>README</span></a></li>
    <li><a href="${depth}guides/guide_debutant.html" class="nav-item"><span class="nav-icon">🚀</span><span>Guide Débutant</span></a></li>
    <li><a href="${depth}architecture/architecture_overview.html" class="nav-item"><span class="nav-icon">🏗️</span><span>Architecture</span></a></li>
    <li><a href="${depth}guides_techniques/integration_guide.html" class="nav-item"><span class="nav-icon">🔗</span><span>Intégration</span></a></li>
    <li><a href="${depth}guides_techniques/testing_guide.html" class="nav-item"><span class="nav-icon">🧪</span><span>Tests</span></a></li>
  </ul>
</div>
<div class="nav-section">
  <div class="nav-title">Ressources</div>
  <ul class="nav-list">
    <li><a href="https://github.com/arkalia-luna-system/bbia-sim" class="nav-item external" target="_blank"><span class="nav-icon">💻</span><span>GitHub</span><span class="external-badge">↗</span></a></li>
    <li><a href="http://localhost:8000/docs" class="nav-item external" target="_blank"><span class="nav-icon">📡</span><span>API Swagger</span><span class="external-badge">↗</span></a></li>
  </ul>
</div>
</nav>
</div>
<div class="content-wrapper">
<div class="content" id="app">
<div class="markdown-body" style="color: #ffffff; background-color: #000000;">
  <div class="loading-state">
    <div class="loading-spinner"></div>
    <p>Chargement de la documentation…</p>
  </div>
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
  --bg-card: #161616;
  --bg-sidebar: #131313;
  --bg-header: rgba(19, 19, 19, 0.95);
  --text-primary: #fafafa;
  --text-secondary: #c4c4c4;
  --text-tertiary: #9a9a9a;
  --accent: #64b5f6;
  --accent-hover: #90caf9;
  --accent-glow: rgba(100, 181, 246, 0.15);
  --border: #2d2d2d;
  --border-light: #222222;
  --shadow: rgba(0, 0, 0, 0.4);
  --shadow-lg: rgba(0, 0, 0, 0.6);
  --gradient-1: linear-gradient(135deg, rgba(100, 181, 246, 0.1) 0%, rgba(156, 39, 176, 0.1) 100%);
  color-scheme: dark;
}

* {
  box-sizing: border-box;
}

html, body {
  height: 100% !important;
  margin: 0 !important;
  padding: 0 !important;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Inter', Roboto, Oxygen, Ubuntu, Cantarell, 'Helvetica Neue', Arial, sans-serif;
  background-color: var(--bg-dark) !important;
  background: 
    radial-gradient(circle at 20% 30%, rgba(100, 181, 246, 0.03) 0%, transparent 50%),
    radial-gradient(circle at 80% 70%, rgba(156, 39, 176, 0.03) 0%, transparent 50%),
    var(--bg-dark) !important;
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
  flex-direction: column;
  min-height: 100vh;
}

/* Header élégant */
.docs-header {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  height: 70px;
  background: var(--bg-header);
  backdrop-filter: blur(20px) saturate(180%);
  border-bottom: 1px solid var(--border-light);
  z-index: 1000;
  box-shadow: 0 2px 20px var(--shadow);
}

.header-content {
  max-width: 1440px;
  margin: 0 auto;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 40px;
}

.logo-section {
  display: flex;
  align-items: center;
  gap: 16px;
}

.logo-circle {
  width: 40px;
  height: 40px;
  border-radius: 10px;
  background: var(--gradient-1);
  border: 1px solid var(--border);
  box-shadow: 0 4px 12px rgba(100, 181, 246, 0.2);
  position: relative;
}

.logo-circle::before {
  content: '';
  position: absolute;
  inset: 2px;
  border-radius: 8px;
  background: var(--accent);
  opacity: 0.2;
}

.logo-text {
  display: flex;
  flex-direction: column;
}

.logo-title {
  font-size: 18px;
  font-weight: 700;
  color: var(--text-primary);
  letter-spacing: -0.02em;
}

.logo-subtitle {
  font-size: 11px;
  color: var(--text-tertiary);
  text-transform: uppercase;
  letter-spacing: 0.1em;
  font-weight: 500;
}

.header-nav {
  display: flex;
  gap: 8px;
}

.nav-link {
  padding: 8px 16px;
  border-radius: 8px;
  color: var(--text-secondary);
  text-decoration: none;
  font-size: 14px;
  font-weight: 500;
  transition: all 0.2s ease;
}

.nav-link:hover {
  background: rgba(255, 255, 255, 0.05);
  color: var(--text-primary);
}

.nav-link.external::after {
  content: ' ↗';
  opacity: 0.6;
  font-size: 12px;
}

.sidebar {
  width: 280px;
  background: var(--bg-sidebar);
  border-right: 1px solid var(--border-light);
  padding: 0;
  position: fixed;
  top: 70px;
  height: calc(100vh - 70px);
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

.sidebar-nav {
  padding: 24px 0;
}

.nav-section {
  margin-bottom: 32px;
}

.nav-title {
  color: var(--text-tertiary);
  font-size: 11px;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.12em;
  margin: 0 24px 12px 24px;
  padding-bottom: 8px;
  border-bottom: 1px solid var(--border-light);
}

.nav-list {
  list-style: none;
  padding: 0;
  margin: 0;
}

.nav-item {
  color: var(--text-secondary);
  text-decoration: none;
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 12px 24px;
  margin: 2px 16px;
  border-radius: 10px;
  transition: all 0.25s cubic-bezier(0.4, 0, 0.2, 1);
  font-size: 14px;
  font-weight: 500;
  position: relative;
  border-left: 3px solid transparent;
}

.nav-item:hover {
  background: rgba(255, 255, 255, 0.04);
  color: var(--text-primary);
  transform: translateX(4px);
  border-left-color: var(--accent);
  box-shadow: -4px 0 12px var(--accent-glow);
}

.nav-icon {
  font-size: 18px;
  width: 24px;
  text-align: center;
  opacity: 0.8;
  transition: transform 0.2s ease;
}

.nav-item:hover .nav-icon {
  transform: scale(1.1);
}

.nav-item span:not(.nav-icon):not(.external-badge) {
  flex: 1;
}

.external-badge {
  font-size: 12px;
  opacity: 0.5;
  margin-left: auto;
}

.content-wrapper {
  margin-left: 280px;
  margin-top: 70px;
  min-height: calc(100vh - 70px);
  display: flex;
  justify-content: center;
  padding: 0;
}

.content {
  flex: 1;
  padding: 80px 80px;
  max-width: 1100px;
  width: 100%;
  background-color: transparent !important;
  color: var(--text-primary) !important;
}

.loading-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 80px 40px;
  text-align: center;
  color: var(--text-secondary);
}

.loading-spinner {
  width: 40px;
  height: 40px;
  border: 3px solid var(--border);
  border-top-color: var(--accent);
  border-radius: 50%;
  animation: spin 0.8s linear infinite;
  margin-bottom: 24px;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

.docs-container {
  background-color: transparent !important;
}

.markdown-body {
  line-height: 1.75;
  background-color: transparent !important;
  color: var(--text-primary) !important;
  font-size: 16px;
  position: relative;
}

.markdown-body::before {
  content: '';
  position: absolute;
  top: 0;
  left: -80px;
  width: 2px;
  height: 100%;
  background: linear-gradient(180deg, var(--accent) 0%, transparent 100%);
  opacity: 0.1;
}

.markdown-body h1,
.markdown-body h1 * {
  font-size: 3em;
  font-weight: 800;
  border-bottom: none;
  padding-bottom: 0;
  margin-top: 0;
  margin-bottom: 16px;
  color: var(--text-primary) !important;
  letter-spacing: -0.03em;
  position: relative;
  padding-left: 20px;
}

.markdown-body h1::before {
  content: '';
  position: absolute;
  left: 0;
  top: 0.15em;
  width: 4px;
  height: 0.8em;
  background: linear-gradient(180deg, var(--accent), var(--accent-hover));
  border-radius: 2px;
  box-shadow: 0 0 12px var(--accent-glow);
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
  background: rgba(100, 181, 246, 0.12);
  padding: 4px 10px;
  border-radius: 6px;
  font-size: 0.88em;
  color: var(--accent-hover);
  font-family: 'SF Mono', 'Monaco', 'Menlo', 'Consolas', 'Courier New', monospace;
  border: 1px solid rgba(100, 181, 246, 0.2);
  font-weight: 500;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.2);
}

.markdown-body pre {
  background: var(--bg-card);
  padding: 24px;
  border-radius: 14px;
  overflow-x: auto;
  border: 1px solid var(--border);
  box-shadow: 
    inset 0 1px 3px rgba(0, 0, 0, 0.4),
    0 4px 20px rgba(0, 0, 0, 0.3);
  position: relative;
  margin: 24px 0;
}

.markdown-body pre::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  height: 1px;
  background: linear-gradient(90deg, transparent, var(--accent), transparent);
  opacity: 0.3;
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
  border-left: 4px solid var(--accent);
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
