# 🔄 Workflow Open Source - Branding BBIA

> **Méthodologie** : Versionnage, documentation, tests et relecture collaborative  
> **Principe** : Chaque asset doit être prêt pour intégration directe dans le code/doc

---

## 📋 Principes Fondamentaux

### **1. Versionnage Complet**

Chaque asset livré doit inclure :

- ✅ **Format source** (SVG, AI, PSD, Figma, etc.) - versionnable
- ✅ **Formats exportés** (PNG, JPG, PDF, etc.) - utilisables immédiatement
- ✅ **Métadonnées** (dimensions, poids, usage) - dans README associé

### **2. Documentation Synthétique**

Pour chaque asset :

- ✅ **README.md** ou fichier `.md` associé
- ✅ **Spécifications techniques** (dimensions, formats, couleurs)
- ✅ **Exemples d'usage** (intégration code, doc, UI)
- ✅ **Justification créative** (choix couleurs, typographie, variantes)

### **3. Tests en Conditions Réelles**

Chaque asset doit être testé :

- ✅ **Preview HTML/CSS** (aperçu visuel)
- ✅ **Intégration dans doc** (exemple markdown, site)
- ✅ **Favicon web** (test navigateur)
- ✅ **Rendu UI** (mockup ou screenshot)

### **4. Relecture Collaborative**

Workflow type "Pull Request" :

- ✅ **Relecture fonctionnelle** (conformité brief, spécifications)
- ✅ **Relecture visuelle** (cohérence, esprit BBIA)
- ✅ **Test d'intégration** (code, doc, UI)
- ✅ **Feedback structuré** (priorités, suggestions)

---

## 📁 Structure des Livrables

```text
presentation/
├── livrables/
│   ├── v1.0/                    # Version initiale
│   │   ├── logo/
│   │   │   ├── source/
│   │   │   │   └── logo_bbia.ai         # Fichier source
│   │   │   ├── exports/
│   │   │   │   ├── logo_bbia_complet.svg
│   │   │   │   ├── logo_bbia_complet.png
│   │   │   │   ├── mascotte_seule.svg
│   │   │   │   └── favicons/
│   │   │   │       ├── favicon_32x32.png
│   │   │   │       ├── favicon_64x64.png
│   │   │   │       └── favicon_128x128.png
│   │   │   ├── tests/
│   │   │   │   ├── preview_logo.html    # Aperçu visuel
│   │   │   │   └── integration_doc.md   # Exemple doc
│   │   │   └── README.md               # Documentation asset
│   │   │
│   │   ├── palette/
│   │   │   ├── source/
│   │   │   │   └── palette_bbia.ase     # Swatches Adobe
│   │   │   ├── exports/
│   │   │   │   ├── palette_bbia.json
│   │   │   │   ├── palette_bbia.css
│   │   │   │   └── palette_bbia.pdf
│   │   │   ├── tests/
│   │   │   │   └── preview_palette.html
│   │   │   └── README.md
│   │   │
│   │   └── banniere/
│   │       ├── source/
│   │       │   └── banniere_univers.ai
│   │       ├── exports/
│   │       │   ├── banniere_univers_1920x1080.png
│   │       │   ├── banniere_univers_2560x1440.png
│   │       │   └── banniere_univers_1200x675.png
│   │       ├── tests/
│   │       │   └── preview_banniere.html
│   │       └── README.md
│   │
│   └── v1.1/                    # Version corrigée/améliorée
│       └── ...
```

---

## 📝 Template README Asset

```markdown
# [Nom Asset] - Documentation

## 📋 Informations Générales

| Propriété | Valeur |
|-----------|--------|
| **Version** | 1.0 |
| **Date création** | YYYY-MM-DD |
| **Auteur** | Graphiste |
| **Statut** | ✅ Validé / 🔄 En relecture |

## 🎨 Spécifications Techniques

**Formats disponibles** :
- Source : `.ai` / `.svg` / `.psd`
- Export : `.png`, `.svg`, `.jpg`

**Dimensions** :
- Logo complet : 2048x2048px
- Favicon : 32x32px, 64x64px, 128x128px

**Couleurs utilisées** :
- Bleu principal : `#87bcfa`
- Violet accent : `#A680FF`
- [etc.]

## 💡 Justification Créative

**Choix de conception** :
- [Expliquer les choix de couleurs, typographie, style]
- [Lien avec l'ADN BBIA]

**Variantes proposées** :
- [Description des variantes et leur usage]

## 🧪 Tests & Intégration

**Preview** : `tests/preview_[asset].html`

**Intégration** :
```html

<!-- Exemple code HTML/CSS -->
<img src="logo_bbia_complet.svg" alt="BBIA Logo">

```text

**Rendu testé** :
- ✅ Documentation markdown
- ✅ Site web (header)
- ✅ Favicon navigateur
- ✅ Impression (PDF)

## 📦 Utilisation

[Instructions d'utilisation]

## 🔄 Historique Versions

- v1.0 (YYYY-MM-DD) : Version initiale
- v1.1 (YYYY-MM-DD) : [Corrections apportées]
```

---

## 🔍 Processus de Relecture

### **Étape 1 : Livraison**

Graphiste livre dans `livrables/v[X.X]/[type]/`

### **Étape 2 : Relecture Fonctionnelle**

- [ ] Conformité au brief
- [ ] Respect des spécifications techniques
- [ ] Formats livrés complets
- [ ] Documentation présente

### **Étape 3 : Test Intégration**

- [ ] Preview HTML fonctionne
- [ ] Intégration dans doc testée
- [ ] Favicon testé en navigateur
- [ ] Rendu UI vérifié

### **Étape 4 : Relecture Visuelle**

- [ ] Cohérence avec ADN BBIA
- [ ] Esprit "douceur tech" respecté
- [ ] Palette couleurs cohérente
- [ ] Qualité artistique

### **Étape 5 : Feedback**

Documenter dans `SUIVI_BRANDING.md` :

- Points validés ✅
- Ajustements demandés 🔄
- Suggestions 💡

### **Étape 6 : Corrections & Validation**

- Graphiste corrige selon feedback
- Nouvelle livraison en `v[X.X+1]/`
- Validation finale

---

## 📊 Checklist Livraison Complète

Pour chaque asset livré, vérifier :

### **Fichiers**

- [ ] Source (format éditable)
- [ ] Exports (formats utilisables)
- [ ] Documentation (README.md)
- [ ] Tests (preview HTML, exemples)

### **Documentation**

- [ ] Spécifications techniques
- [ ] Justification créative
- [ ] Exemples d'usage
- [ ] Historique versions

### **Tests**

- [ ] Preview HTML/CSS
- [ ] Intégration doc
- [ ] Intégration web
- [ ] Rendu final validé

### **Qualité**

- [ ] Conformité brief
- [ ] Qualité artistique
- [ ] Cohérence BBIA
- [ ] Prêt pour intégration

---

## 🔗 Liens Utiles

- **Suivi projet** : `SUIVI_BRANDING.md`
- **Brief** : `BRIEF_GRAPHISTE_DA_BBIA.md`
- **Spécifications** : `specifications_*.md`
- **GitHub Actions** : [workflow CI/CD](https://github.com/arkalia-luna-system/bbia-sim/actions)

---

*Workflow adapté aux projets open source - Version 1.0*
