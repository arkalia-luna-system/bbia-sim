# [NOM_ASSET] - Documentation

> **Template** pour documentation de chaque asset livré  
> **Usage** : Copier ce template et personnaliser pour chaque asset (logo, palette, bannière)

---

## 📋 Informations Générales

| Propriété | Valeur |
|-----------|--------|
| **Version** | 1.0 |
| **Date création** | YYYY-MM-DD |
| **Auteur** | [Nom graphiste] |
| **Statut** | ✅ Validé / 🔄 En relecture / ⏳ En attente |
| **Priorité** | 🔥 Haute / 🟡 Moyenne / 🟢 Basse |

---

## 🎨 Spécifications Techniques

### **Formats Disponibles**

**Source** (versionnable) :

- `.ai` (Adobe Illustrator)
- `.svg` (vectoriel)
- `.psd` (Photoshop)
- `.fig` (Figma)

**Exports** (utilisables) :

- `.png` (haute résolution, fond transparent)
- `.svg` (vectoriel)
- `.jpg` (optimisé web)
- `.pdf` (impression)

### **Dimensions**

| Format | Dimensions | Usage |
|--------|------------|-------|
| Logo complet | 2048x2048px | Header, bannière |
| Favicon | 32x32px, 64x64px, 128x128px | Navigateur |
| [Autre] | [dimensions] | [usage] |

### **Poids Fichiers**

| Format | Taille | Optimisation |
|--------|--------|--------------|
| PNG 2048x2048 | [X] MB | [Status] |
| SVG | [X] KB | [Status] |
| Favicon 32x32 | [X] KB | [Status] |

### **Couleurs Utilisées**

| Couleur | Code HEX | Usage | Justification |
|---------|----------|-------|---------------|
| Bleu principal | `#87bcfa` | [usage] | [lien ADN BBIA] |
| Violet accent | `#A680FF` | [usage] | [lien ADN BBIA] |
| [Autre] | `#[code]` | [usage] | [justification] |

### **Typographie** (si applicable)

- **Police** : [Nom police]
- **Source** : [Google Fonts / Personnalisée]
- **Justification** : [Pourquoi ce choix]

---

## 💡 Justification Créative

### **Choix de Conception**

[Expliquer en détail les choix créatifs :]

- Pourquoi ces couleurs ?
- Pourquoi ce style ?
- Comment ça s'inscrit dans l'ADN BBIA ?
- Lien avec Arkalia / futur lunaire ?

### **Variantes Proposées**

**Variante A** : [description]

- Usage : [contexte d'utilisation]
- Différences : [ce qui change]

**Variante B** : [description]

- Usage : [contexte d'utilisation]
- Différences : [ce qui change]

### **Alternatives Créatives**

[Si variantes supplémentaires proposées :]

- [Description des alternatives]
- [Justification si cohérent avec branding]

---

## 🧪 Tests & Intégration

### **Preview Visuel**

**Fichier** : `tests/preview_[asset].html`

**Contenu** :

- Aperçu de l'asset
- Différentes tailles
- Différents fonds (clair/sombre)
- Exemples d'utilisation

### **Intégration Code**

**HTML** :

```html
<!-- Exemple d'intégration -->
<img src="logo_bbia_complet.svg" alt="BBIA Logo" class="bbia-logo">
```

**CSS** :

```css 🎨
.bbia-logo {
    max-width: 200px;
    height: auto;
}
```

**Markdown** (Documentation) :

```markdown
![BBIA Logo](logo_bbia_complet.png)
```

### **Intégration Web**

**Favicon** :

```html
<link rel="icon" type="image/png" sizes="32x32" href="favicon_32x32.png">
<link rel="icon" type="image/png" sizes="64x64" href="favicon_64x64.png">
<link rel="icon" type="image/png" sizes="128x128" href="favicon_128x128.png">
```

### **Rendu Testé**

- [x] Documentation markdown ✅
- [x] Site web (header) ✅
- [x] Favicon navigateur ✅
- [x] Impression (PDF) ✅
- [x] Mobile responsive ✅
- [x] Dark mode ✅

**Screenshots** : Disponibles dans `tests/screenshots/`

---

## 📦 Utilisation

### **Pour Développeurs**

1. Copier les fichiers dans `assets/logo/` (ou équivalent)
2. Utiliser le SVG pour qualité vectorielle
3. Utiliser le PNG pour compatibilité maximale
4. Référencer dans code via chemin relatif

### **Pour Documentation**

1. Intégrer dans README.md :

```markdown
![BBIA Logo](../livrables/v1.0/logo/exports/logo_bbia_complet.png)
```

2. Utiliser favicon dans config site

### **Pour Communication**

1. Logo pour email signatures
2. Bannière pour réseaux sociaux
3. Formats adaptés aux plateformes

---

## 🔄 Historique Versions

| Version | Date | Modifications | Statut |
|---------|------|---------------|--------|
| 1.0 | YYYY-MM-DD | Version initiale | ✅ Validé |
| 1.1 | YYYY-MM-DD | [Corrections demandées] | 🔄 En cours |
| 1.2 | YYYY-MM-DD | [Ajustements finaux] | ✅ Validé |

**Détails modifications** :

- v1.1 : [Description détaillée des corrections]
- v1.2 : [Description détaillée des ajustements]

---

## ✅ Validation

### **Relecture Fonctionnelle**

- [x] Conformité brief
- [x] Spécifications techniques respectées
- [x] Formats complets

### **Relecture Visuelle**

- [x] Cohérence ADN BBIA
- [x] Esprit "douceur tech"
- [x] Qualité artistique

### **Tests Intégration**

- [x] Preview fonctionne
- [x] Code intégré testé
- [x] Documentation validée

---

## 🔗 Liens

- **Brief** : `../../BRIEF_GRAPHISTE_DA_BBIA.md`
- **Spécifications** : `../../specifications_[type].md`
- **Suivi** : `../../SUIVI_BRANDING.md`
- **Workflow** : `../../WORKFLOW_OPEN_SOURCE.md`

---

*Template de documentation asset - BBIA Branding*
