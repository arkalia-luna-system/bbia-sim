# 🔗 Intégration Logos BBIA dans le Dashboard

> **Date** : 7 Décembre 2025  
> **Statut** : 📝 **Guide d'intégration**

---

## 📍 Emplacement des Logos

Les logos BBIA sont disponibles dans :
- **Logos** : `assets/logos/bbia/logos/`
- **Identity** : `assets/logos/bbia/identity/`

Pour les utiliser dans le dashboard, ils doivent être accessibles via `/static/`.

---

## 🔧 Option 1 : Lien Symbolique (Recommandé)

Créer un lien symbolique depuis `static/` vers les logos :

```bash
cd /Volumes/T7/bbia-reachy-sim/src/bbia_sim/daemon/app/dashboard/static
ln -s ../../../../../../assets/logos/bbia logos
```

**Avantages** :
- ✅ Pas de duplication
- ✅ Mise à jour automatique si les logos changent
- ✅ Structure propre

**Usage dans HTML** :
```html
<img src="/static/logos/logos/bbia-horizontal-clean-serenity-512.svg" alt="BBIA Logo">
```

---

## 🔧 Option 2 : Copie Directe

Copier les logos dans `static/` :

```bash
cp -r /Volumes/T7/bbia-reachy-sim/assets/logos/bbia/* \
      /Volumes/T7/bbia-reachy-sim/src/bbia_sim/daemon/app/dashboard/static/
```

**Avantages** :
- ✅ Fichiers directement accessibles
- ✅ Pas de dépendance vers assets/

**Inconvénients** :
- ⚠️ Duplication
- ⚠️ Nécessite recopie lors des mises à jour

---

## 🎨 Exemples d'Utilisation

### Header du Dashboard

```html
<!-- Dans base.html ou index.html -->
<header class="flex items-center justify-between p-4 bg-white shadow">
    <img src="/static/logos/logos/bbia-horizontal-clean-serenity-512.svg" 
         alt="BBIA Logo" 
         class="h-12">
    <nav>
        <!-- Navigation -->
    </nav>
</header>
```

### Favicon (Nécessite conversion PNG)

Le `manifest.json` nécessite des PNG. Pour utiliser le logo BBIA :

1. Convertir `bbia-app_icon-512.svg` en PNG :
   ```bash
   # Avec Inkscape
   inkscape --export-filename=icon-512.png --export-width=512 --export-height=512 \
            assets/logos/bbia/identity/bbia-app_icon-512.svg
   
   # Avec ImageMagick
   convert -background none -density 300 \
           assets/logos/bbia/identity/bbia-app_icon-512.svg \
           src/bbia_sim/daemon/app/dashboard/static/images/icon-512.png
   ```

2. Générer aussi la version 192x192 :
   ```bash
   convert -background none -resize 192x192 \
           assets/logos/bbia/identity/bbia-app_icon-512.svg \
           src/bbia_sim/daemon/app/dashboard/static/images/icon-192.png
   ```

3. Mettre à jour `manifest.json` (déjà fait si les fichiers sont au bon endroit)

### Logo Dynamique selon l'Émotion

```javascript
// Dans un fichier JS du dashboard
function updateLogo(emotion, style = 'clean') {
    const logoPath = `/static/logos/logos/bbia-horizontal-${style}-${emotion}-512.svg`;
    document.getElementById('bbia-logo').src = logoPath;
}

// Usage
updateLogo('power', 'wireframe');  // Logo énergique
updateLogo('serenity', 'clean');    // Logo calme
```

---

## 📝 Notes Importantes

1. **Format SVG** : Les logos sont en SVG, donc évolutifs sans perte de qualité
2. **Taille** : Tous les logos sont en 512×512px, mais peuvent être redimensionnés via CSS
3. **Style** : Utiliser "clean" pour fonds clairs, "wireframe" pour fonds sombres
4. **Émotion par défaut** : "serenity" est recommandée pour l'usage général

---

## 🔄 Mise à Jour

Si les logos sont mis à jour dans `assets/logos/bbia/` :

- **Option 1 (lien symbolique)** : Mise à jour automatique ✅
- **Option 2 (copie)** : Recopier les fichiers

---

**Dernière mise à jour** : 7 Décembre 2025

