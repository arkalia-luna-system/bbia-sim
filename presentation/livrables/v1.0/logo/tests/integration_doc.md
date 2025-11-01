# Exemples d'Intégration Logo BBIA

> Exemples concrets d'intégration du logo dans différents contextes

---

## 📖 Documentation Markdown

### Exemple 1 : Header README

```markdown
# <img src="logo_bbia_horizontal.svg" alt="BBIA" width="200">

**BBIA** - Brain-Based Interactive Agent

Assistant IA robotique nouvelle génération
```

### Exemple 2 : Badge Logo

```markdown
![BBIA](mascotte_seule.svg)

## BBIA Logo Usage

Le logo BBIA représente notre assistant IA...
```

---

## 🌐 Intégration Web HTML

### Header avec Logo

```html
<header class="bbia-header">
    <div class="container">
        <a href="/" class="logo-link">
            <img src="logo_bbia_horizontal.svg" 
                 alt="BBIA Logo" 
                 class="bbia-logo"
                 width="200" 
                 height="67">
        </a>
        <nav>
            <!-- Navigation -->
        </nav>
    </div>
</header>
```

### Footer avec Logo

```html
<footer class="bbia-footer">
    <div class="container">
        <img src="logo_bbia_horizontal.svg" 
             alt="BBIA" 
             class="footer-logo"
             width="150">
        <p>© 2025 BBIA - Arkalia Luna System</p>
    </div>
</footer>
```

---

## 💻 Intégration Code

### Python (FastAPI/Flask)

```python
from pathlib import Path

LOGO_DIR = Path(__file__).parent / "static" / "logo"
LOGO_HORIZONTAL = LOGO_DIR / "logo_bbia_horizontal.svg"
LOGO_MASCOTTE = LOGO_DIR / "mascotte_seule.svg"

# Dans template
@app.route("/")
def index():
    return render_template(
        "index.html",
        logo_path=LOGO_HORIZONTAL.relative_to(Path("static"))
    )
```

### JavaScript/React

```jsx
import logoHorizontal from './assets/logo/logo_bbia_horizontal.svg';
import logoMascotte from './assets/logo/mascotte_seule.svg';

function Header() {
    return (
        <header>
            <img 
                src={logoHorizontal} 
                alt="BBIA Logo" 
                width="200"
                height="67"
            />
        </header>
    );
}
```

---

## 📧 Email Signature

```html
<table>
    <tr>
        <td>
            <img src="logo_bbia_horizontal.svg" 
                 alt="BBIA" 
                 width="150"
                 style="display: block;">
        </td>
        <td>
            <strong>BBIA</strong><br>
            Brain-Based Interactive Agent<br>
            <a href="https://github.com/arkalia-luna-system/bbia-sim">GitHub</a>
        </td>
    </tr>
</table>
```

---

## 📱 Favicon Configuration

### HTML Head

```html
<head>
    <link rel="icon" type="image/png" sizes="32x32" href="favicon_32x32.png">
    <link rel="icon" type="image/png" sizes="64x64" href="favicon_64x64.png">
    <link rel="icon" type="image/png" sizes="128x128" href="favicon_128x128.png">
    <link rel="apple-touch-icon" sizes="180x180" href="favicon_512x512.png">
    <meta name="theme-color" content="#87bcfa">
</head>
```

### Python Sphinx (Documentation)

```python
# conf.py
html_favicon = '_static/logo/favicon_128x128.png'
html_logo = '_static/logo/logo_bbia_horizontal.svg'
```

---

## 🎨 CSS Styles

```css
/* Logo Base Styles */
.bbia-logo {
    max-width: 200px;
    height: auto;
    display: block;
    transition: opacity 0.3s ease;
}

.bbia-logo:hover {
    opacity: 0.8;
}

/* Dark Mode Support */
@media (prefers-color-scheme: dark) {
    .bbia-logo {
        filter: brightness(1.1);
    }
}

/* Responsive Logo */
@media (max-width: 768px) {
    .bbia-logo {
        max-width: 150px;
    }
}

/* Header Logo */
.bbia-header .bbia-logo {
    max-width: 180px;
}

/* Footer Logo */
.bbia-footer .bbia-logo {
    max-width: 120px;
    opacity: 0.7;
}
```

---

## 📊 Présentations (PowerPoint/Keynote)

**Recommandations** :
- Utiliser PNG haute résolution (2048x2048px)
- Logo horizontal pour slides titre
- Mascotte seule pour slides de transition
- Version couleur pour slides claires
- Version monochrome pour slides sombres

---

## ✅ Checklist Intégration

- [x] Documentation markdown testée
- [x] HTML/CSS intégration testée
- [x] Favicon configuré
- [x] Email signature testée
- [x] Code Python exemple fourni
- [x] Code JavaScript/React exemple fourni
- [x] Styles CSS fournis
- [x] Responsive testé

---

*Exemples d'intégration - Logo BBIA v1.0*

