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
<div class="docs-container">
  <aside class="sidebar">
    <div class="sidebar-header">
      <div class="logo">
        <div class="logo-icon">🤖</div>
        <div class="logo-text">
          <div class="logo-title">BBIA-SIM</div>
          <div class="logo-subtitle">Documentation</div>
        </div>
      </div>
    </div>
    
    <nav class="sidebar-nav">
      <div class="nav-section">
        <div class="nav-group-title">Commencer</div>
        <a href="${depth}index.html" class="nav-item">
          <span class="nav-icon">🏠</span>
          <span class="nav-label">Accueil</span>
        </a>
        <a href="${depth}README.html" class="nav-item">
          <span class="nav-icon">📖</span>
          <span class="nav-label">README</span>
        </a>
        <a href="${depth}guides/guide_debutant.html" class="nav-item">
          <span class="nav-icon">🚀</span>
          <span class="nav-label">Guide Débutant</span>
        </a>
      </div>
      
      <div class="nav-section">
        <div class="nav-group-title">Documentation</div>
        <a href="${depth}architecture/architecture_overview.html" class="nav-item">
          <span class="nav-icon">🏗️</span>
          <span class="nav-label">Architecture</span>
        </a>
        <a href="${depth}guides_techniques/integration_guide.html" class="nav-item">
          <span class="nav-icon">🔗</span>
          <span class="nav-label">Intégration</span>
        </a>
        <a href="${depth}guides_techniques/testing_guide.html" class="nav-item">
          <span class="nav-icon">🧪</span>
          <span class="nav-label">Tests</span>
        </a>
      </div>
      
      <div class="nav-section">
        <div class="nav-group-title">Ressources</div>
        <a href="https://github.com/arkalia-luna-system/bbia-sim" class="nav-item external" target="_blank">
          <span class="nav-icon">💻</span>
          <span class="nav-label">GitHub</span>
          <span class="nav-arrow">↗</span>
        </a>
        <a href="http://localhost:8000/docs" class="nav-item external" target="_blank">
          <span class="nav-icon">📡</span>
          <span class="nav-label">API Swagger</span>
          <span class="nav-arrow">↗</span>
        </a>
      </div>
    </nav>
  </aside>
  
  <main class="content-area">
    <div class="content" id="app">
      <div class="markdown-body">
        <div class="loading-state">
          <div class="loading-spinner"></div>
          <p>Chargement de la documentation…</p>
        </div>
      </div>
    </div>
  </main>
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
  /* Palette premium BBIA/Reachy - Sombre élégant */
  --bg-dark: #0f0f14;
  --bg-card: #1a1b26;
  --bg-sidebar: #161822;
  --bg-header: rgba(22, 24, 34, 0.85);
  --text-primary: #e4e4e7;
  --text-secondary: #a1a1aa;
  --text-tertiary: #71717a;
  /* Accent tech élégant (bleu/cyan robotique) */
  --accent: #00d9ff;
  --accent-hover: #33e3ff;
  --accent-secondary: #8b5cf6;
  --accent-glow: rgba(0, 217, 255, 0.2);
  --accent-glow-strong: rgba(0, 217, 255, 0.35);
  /* Bordures subtiles */
  --border: #27272a;
  --border-light: #1f1f23;
  --border-glow: rgba(0, 217, 255, 0.1);
  /* Ombres profondes */
  --shadow: rgba(0, 0, 0, 0.5);
  --shadow-lg: rgba(0, 0, 0, 0.7);
  --shadow-xl: rgba(0, 0, 0, 0.9);
  /* Gradients premium */
  --gradient-tech: linear-gradient(135deg, rgba(0, 217, 255, 0.08) 0%, rgba(139, 92, 246, 0.08) 100%);
  --gradient-accent: linear-gradient(135deg, var(--accent) 0%, var(--accent-secondary) 100%);
  --gradient-glow: radial-gradient(circle at 50% 50%, rgba(0, 217, 255, 0.1) 0%, transparent 70%);
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
  min-height: 100vh;
  background: var(--bg-dark);
}

/* Sidebar élégante à gauche */
.sidebar {
  width: 300px;
  background: var(--bg-sidebar);
  border-right: 1px solid var(--border-light);
  position: fixed;
  left: 0;
  top: 0;
  height: 100vh;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  backdrop-filter: blur(20px);
  box-shadow: 4px 0 30px rgba(0, 0, 0, 0.5);
  z-index: 100;
}

.sidebar::-webkit-scrollbar {
  width: 4px;
}

.sidebar::-webkit-scrollbar-track {
  background: transparent;
}

.sidebar::-webkit-scrollbar-thumb {
  background: var(--border);
  border-radius: 2px;
}

.sidebar::-webkit-scrollbar-thumb:hover {
  background: var(--border-light);
}

/* Header de la sidebar */
.sidebar-header {
  padding: 32px 24px;
  border-bottom: 1px solid var(--border-light);
}

.logo {
  display: flex;
  align-items: center;
  gap: 14px;
}

.logo-icon {
  font-size: 32px;
  width: 48px;
  height: 48px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: var(--gradient-1);
  border-radius: 12px;
  border: 1px solid var(--border);
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
  line-height: 1.2;
}

.logo-subtitle {
  font-size: 11px;
  color: var(--text-tertiary);
  text-transform: uppercase;
  letter-spacing: 0.08em;
  font-weight: 500;
  margin-top: 2px;
}

/* Navigation */
.sidebar-nav {
  padding: 24px 0;
  flex: 1;
}

.nav-section {
  margin-bottom: 32px;
}

.nav-group-title {
  color: var(--text-tertiary);
  font-size: 10px;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.15em;
  padding: 0 24px 10px 24px;
  margin-bottom: 8px;
}

.nav-item {
  color: var(--text-secondary);
  text-decoration: none;
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 10px 24px;
  margin: 0 12px 4px 12px;
  border-radius: 8px;
  transition: all 0.2s ease;
  font-size: 14px;
  font-weight: 500;
  position: relative;
}

.nav-item:hover {
  background: rgba(255, 255, 255, 0.03);
  color: var(--text-primary);
}

.nav-item.active {
  background: rgba(100, 181, 246, 0.1);
  color: var(--accent);
  border-left: 3px solid var(--accent);
  padding-left: 21px;
}

.nav-icon {
  font-size: 18px;
  width: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
  opacity: 0.7;
  transition: opacity 0.2s;
}

.nav-item:hover .nav-icon,
.nav-item.active .nav-icon {
  opacity: 1;
}

.nav-label {
  flex: 1;
}

.nav-arrow {
  font-size: 12px;
  opacity: 0.4;
  margin-left: auto;
}

.nav-item.external:hover .nav-arrow {
  opacity: 0.7;
}

.content-area {
  margin-left: 300px;
  min-height: 100vh;
  display: flex;
  justify-content: center;
  background: var(--bg-dark);
}

.content {
  width: 100%;
  max-width: 1000px;
  padding: 60px 80px;
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
