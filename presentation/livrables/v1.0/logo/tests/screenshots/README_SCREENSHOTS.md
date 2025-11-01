# Screenshots Tests Logo BBIA

> **Guide** pour génération des screenshots de tests d'intégration

---

## 📋 Screenshots Requis

### **1. Favicon Navigateur**

**À capturer** :
- [ ] Favicon dans Chrome (onglet navigateur)
- [ ] Favicon dans Firefox (onglet navigateur)
- [ ] Favicon dans Safari (onglet navigateur)
- [ ] Favicon dans Edge (onglet navigateur)

**Instructions** :
1. Créer page HTML test avec favicon
2. Ouvrir dans chaque navigateur
3. Capturer screenshot de l'onglet (32x32px visible)
4. Nommer : `favicon_chrome.png`, `favicon_firefox.png`, etc.

---

### **2. Header Web**

**À capturer** :
- [ ] Logo dans header web (desktop)
- [ ] Logo dans header web (mobile)
- [ ] Logo dans header web (tablet)

**Instructions** :
1. Créer page HTML avec header incluant logo
2. Tester responsive (320px, 768px, 1920px)
3. Capturer screenshot complet header
4. Nommer : `header_desktop.png`, `header_mobile.png`, etc.

---

### **3. Documentation Markdown**

**À capturer** :
- [ ] Logo dans README.md GitHub
- [ ] Logo dans documentation Sphinx/MkDocs
- [ ] Logo dans présentation (si applicable)

**Instructions** :
1. Intégrer logo dans markdown
2. Prévisualiser sur GitHub/docs
3. Capturer screenshot
4. Nommer : `readme_github.png`, `docs_sphinx.png`, etc.

---

### **4. Dark Mode**

**À capturer** :
- [ ] Logo version blanc sur fond sombre
- [ ] Logo version couleur sur fond sombre (contraste)

**Instructions** :
1. Créer page avec dark mode
2. Tester version monochrome blanc
3. Tester version couleur (vérifier contraste)
4. Nommer : `dark_mode_monochrome.png`, `dark_mode_color.png`

---

### **5. Responsive**

**À capturer** :
- [ ] Logo mobile (320px width)
- [ ] Logo tablet (768px width)
- [ ] Logo desktop (1920px width)

**Instructions** :
1. Tester toutes les tailles
2. Vérifier lisibilité à chaque taille
3. Capturer screenshots
4. Nommer : `responsive_mobile.png`, `responsive_tablet.png`, `responsive_desktop.png`

---

## 🛠️ Outils Recommandés

**Capture écran** :
- macOS : Cmd+Shift+4 (sélection) ou Cmd+Shift+3 (écran complet)
- Windows : Win+Shift+S (Snipping Tool)
- Linux : Flameshot, Shutter

**Responsive Testing** :
- Chrome DevTools (F12 → Toggle device toolbar)
- Firefox Responsive Design Mode (Cmd+Opt+M)
- Online : BrowserStack, LambdaTest

**Favicon Testing** :
- Créer `test_favicon.html` avec tous les formats
- Ouvrir dans chaque navigateur
- Capturer onglet

---

## 📝 Template HTML pour Tests

**Créer** : `test_screenshots.html`

```html
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <title>BBIA Logo - Test Screenshots</title>
    <link rel="icon" type="image/png" sizes="32x32" href="../exports/favicons/favicon_32x32.svg">
</head>
<body>
    <!-- Header avec logo -->
    <header style="padding: 20px; background: white; border-bottom: 1px solid #eaeaed;">
        <img src="../exports/logo_bbia_horizontal.svg" alt="BBIA" width="200">
    </header>
    
    <!-- Contenu test -->
    <main style="padding: 40px;">
        <h1>Test Logo BBIA</h1>
        <!-- Plusieurs tailles à tester -->
    </main>
</body>
</html>
```

---

## ✅ Checklist Screenshots

- [ ] Favicon Chrome capturé
- [ ] Favicon Firefox capturé
- [ ] Favicon Safari capturé
- [ ] Header desktop capturé
- [ ] Header mobile capturé
- [ ] Documentation markdown capturée
- [ ] Dark mode capturé
- [ ] Responsive capturé (3 tailles)
- [ ] Tous fichiers nommés correctement
- [ ] Tous fichiers dans `tests/screenshots/`

---

*Guide screenshots - Logo BBIA v1.0*

