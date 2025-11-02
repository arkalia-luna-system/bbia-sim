# Exports Logo BBIA - Guide

> **Note** : Exports à générer après conversion du PNG client en SVG

---

## 📋 Workflow

**Une fois le logo client créé** (PNG Procreate) :

1. **Conversion** : PNG → SVG (automatique)
2. **Optimisation** : Nettoyage SVG
3. **Génération** : Toutes versions créées automatiquement

---

## 📦 Exports à Générer (après logo créé)

### **Logo Principal**
- `logo_bbia_complet.svg` - Logo complet (format carré ou selon design client)
- `logo_bbia_horizontal.svg` - Logo horizontal (avec typographie BBIA)

### **Versions**
- `logo_bbia_monochrome_noir.svg` - Version monochrome noir
- `logo_bbia_monochrome_blanc.svg` - Version monochrome blanc

### **Favicons**
- `favicons/favicon_512x512.svg` - Source favicon
- `favicons/favicon_128x128.svg` - Grand favicon
- `favicons/favicon_64x64.svg` - Favicon standard
- `favicons/favicon_32x32.svg` - Petit favicon

### **PNG Haute Résolution** (à générer depuis SVG)
- Logo complet : 2048x2048px
- Logo horizontal : 1500x500px (ou selon design client)
- Favicons : 32, 64, 128, 512px

---

## 🔄 Conversion SVG → PNG

**Une fois SVG créé**, génération PNG avec :

### **Commande Inkscape (CLI)**
```bash
# Logo complet
inkscape --export-filename=logo_bbia_complet.png --export-width=2048 --export-height=2048 logo_bbia_complet.svg

# Favicons
inkscape --export-filename=favicon_32x32.png --export-width=32 --export-height=32 favicons/favicon_512x512.svg
```

### **Commande ImageMagick**
```bash
convert -background none -density 300 logo_bbia_complet.svg logo_bbia_complet.png
```

---

## ✅ Checklist (après logo client créé)

- [ ] SVG généré depuis PNG client
- [ ] Logo complet SVG optimisé
- [ ] Logo horizontal SVG créé (avec typographie)
- [ ] Versions monochrome créées
- [ ] Favicons générés (toutes tailles)
- [ ] PNG haute résolution générés
- [ ] Tous fichiers avec fond transparent
- [ ] Optimisation poids fichiers effectuée

---

*Guide exports - Logo BBIA - À générer après logo client créé*
