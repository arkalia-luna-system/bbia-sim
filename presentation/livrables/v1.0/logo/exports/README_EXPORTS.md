# Exports Logo BBIA - Guide

> **Note importante** : Les fichiers SVG fournis sont des structures de base.  
> Pour les fichiers PNG haute résolution, utiliser un convertisseur SVG→PNG avec les dimensions spécifiées.

---

## 📋 Fichiers SVG Disponibles

### **Logo Complet**
- `logo_bbia_complet.svg` - 2048x2048px (format source complet)
- `logo_bbia_horizontal.svg` - 1500x500px (format header web)

### **Mascotte Seule**
- `mascotte_seule.svg` - 512x512px (avatar, favicon source)

### **Versions Monochrome**
- `logo_monochrome_noir.svg` - Noir (fond clair)
- `logo_monochrome_blanc.svg` - Blanc (fond sombre)

### **Favicons**
- `favicons/favicon_512x512.svg` - Source favicon
- `favicons/favicon_128x128.svg` - Grand favicon
- `favicons/favicon_64x64.svg` - Favicon standard
- `favicons/favicon_32x32.svg` - Petit favicon

---

## 🔄 Conversion SVG → PNG

### **Commande ImageMagick**
```bash
# Logo complet haute résolution
convert -background none -density 300 logo_bbia_complet.svg logo_bbia_complet.png

# Logo horizontal
convert -background none -density 300 logo_bbia_horizontal.svg logo_bbia_horizontal.png

# Favicons
convert -background none -resize 32x32 favicon_512x512.svg favicon_32x32.png
convert -background none -resize 64x64 favicon_512x512.svg favicon_64x64.png
convert -background none -resize 128x128 favicon_512x512.svg favicon_128x128.png
```

### **Commande Inkscape (CLI)**
```bash
# Logo complet
inkscape --export-filename=logo_bbia_complet.png --export-width=2048 --export-height=2048 logo_bbia_complet.svg

# Favicons
inkscape --export-filename=favicon_32x32.png --export-width=32 --export-height=32 favicons/favicon_512x512.svg
```

### **Via Interface Graphique**
- **Inkscape** : Fichier → Exporter PNG
- **Adobe Illustrator** : Exporter pour écrans
- **Figma** : Exporter en PNG

---

## 📊 Spécifications PNG Requises

| Fichier | Dimensions | Format | Fond | Usage |
|---------|------------|--------|------|-------|
| `logo_bbia_complet.png` | 2048x2048px | PNG-24 | Transparent | Haute résolution |
| `logo_bbia_horizontal.png` | 1500x500px | PNG-24 | Transparent | Header web |
| `mascotte_seule.png` | 512x512px | PNG-24 | Transparent | Avatar |
| `logo_monochrome_noir.png` | 2048x2048px | PNG-24 | Transparent | Impression |
| `logo_monochrome_blanc.png` | 2048x2048px | PNG-24 | Transparent | Dark mode |
| `favicon_32x32.png` | 32x32px | PNG-8 | Transparent | Navigateur |
| `favicon_64x64.png` | 64x64px | PNG-24 | Transparent | Navigateur |
| `favicon_128x128.png` | 128x128px | PNG-24 | Transparent | Navigateur |

---

## ✅ Checklist Conversion

- [ ] Logo complet PNG généré (2048x2048px)
- [ ] Logo horizontal PNG généré (1500x500px)
- [ ] Mascotte seule PNG généré (512x512px)
- [ ] Version monochrome noir PNG générée
- [ ] Version monochrome blanc PNG générée
- [ ] Favicons PNG générés (32, 64, 128px)
- [ ] Tous fichiers avec fond transparent
- [ ] Optimisation poids fichiers effectuée
- [ ] Qualité visuelle vérifiée

---

*Guide exports - Logo BBIA v1.0*

