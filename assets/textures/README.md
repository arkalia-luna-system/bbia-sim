# 🎨 Textures pour MuJoCo

> **Dossier pour vos images/textures Procreate** à utiliser dans les scènes MuJoCo

---

## 📋 Textures à Créer

### **Textures de Base**

1. **`mur.png`** - Texture murale
   - Taille : 2048x2048px (recommandé)
   - Usage : Murs de la pièce
   - Peut être répétée (tiling)

2. **`sol.png`** - Texture sol
   - Taille : 2048x2048px (recommandé)
   - Usage : Sol de la pièce
   - Peut être répétée (parquet, carrelage, moquette)

3. **`plafond.png`** - Texture plafond (optionnel)
   - Taille : 2048x2048px
   - Usage : Plafond de la pièce

### **Textures Décoratives** (optionnel)

4. **`tableau.png`** - Tableau/Poster mural
5. **`plante.png`** - Plante décorative
6. **`fenetre.png`** - Fenêtre/vue extérieure

---

## 🎨 Comment Créer dans Procreate

### **Pour Texture Murale**

1. Créez document **2048x2048px**
2. Dessinez votre texture :
   - Peinture murale
   - Papier peint
   - Brique, pierre
   - Dégradé couleur
3. Export PNG haute qualité
4. Placer ici : `assets/textures/mur.png`

### **Pour Texture Sol**

1. Créez document **2048x2048px**
2. Dessinez :
   - Parquet
   - Carrelage
   - Moquette
   - Béton ciré
3. Export PNG haute qualité
4. Placer ici : `assets/textures/sol.png`

---

## ✅ Format Requis

- **Format** : PNG (recommandé) ou JPG
- **Taille** : 1024x1024px minimum, 2048x2048px recommandé
- **Fond** : Selon usage (transparent pour décor, opaque pour murs/sol)

---

## 🔄 Utilisation dans MuJoCo

**Dans XML scène** :

```xml
<texture name="mur" type="2d" file="assets/textures/mur.png"/>
<material name="mat_mur" texture="mur"/>
<geom type="box" material="mat_mur"/>
```

**Voir** : `docs/simulations/GUIDE_IMPORT_IMAGES_MUJOCO.md` pour guide complet.

---

*Dossier Textures - BBIA-SIM*
