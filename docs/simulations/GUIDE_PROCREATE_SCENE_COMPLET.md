# 🎨 Guide Complet - Créer Scène MuJoCo avec Procreate

**Date** : 21 Novembre 2025

> **Guide professionnel** pour créer des textures de pièce dans Procreate, optimisées pour MuJoCo

---

## 📋 Table des Matières

1. [Configuration Procreate](#configuration-procreate)
2. [Organisation des Calques](#organisation-des-calques)
3. [Créer les Textures](#créer-les-textures)
4. [Conseils Professionnels](#conseils-professionnels)
5. [Export et Optimisation](#export-et-optimisation)
6. [Intégration dans MuJoCo](#intégration-dans-mujoco)

---

## 🎨 Configuration Procreate

### **1. Créer le Document**

**Paramètres recommandés** :

- **Taille** : `4096 x 4096 pixels` (qualité maximale)
  - Alternative : `2048 x 2048 pixels` (qualité très bonne)
- **Résolution** : `300 DPI` (pour impression si besoin)
- **Format** : `PNG` (avec transparence si nécessaire)
- **Fond** : `Transparent` ou `Blanc` selon usage

**Pourquoi 4096x4096 ?**

- ✅ Qualité maximale (pixels nets même zoomé)
- ✅ MuJoCo peut redimensionner automatiquement
- ✅ Flexible pour tous usages futurs
- ⚠️ Fichier plus lourd (acceptable pour textures)

### **2. Espace de Couleur**

- **Format** : `RGB` (pas CMYK)
- **Couleur** : `sRGB` (standard web/MuJoCo)
- **Profondeur** : `8 bits` ou `16 bits` (8 bits suffit généralement)

---

## 🗂️ Organisation des Calques (Structure Recommandée)

### **Structure Idéale pour Texture de Mur** :

```text
📁 Calque "Mur Final" (groupe)
  ├── 📄 Calque "Base Couleur"
  ├── 📄 Calque "Texture/Motif"
  ├── 📄 Calque "Ombres/Lumières"
  ├── 📄 Calque "Détails"
  └── 📄 Calque "Filtres/Effets" (optionnel)

```

### **Structure Idéale pour Texture de Sol** :

```text
📁 Calque "Sol Final" (groupe)
  ├── 📄 Calque "Base"
  ├── 📄 Calque "Parquet/Carrelage"
  ├── 📄 Calque "Jointures/Interstices"
  ├── 📄 Calque "Usure/Patine"
  └── 📄 Calque "Reflets" (optionnel)

```

### **Nombre de Calques Optimal** :

**Pour une texture simple** : **3-5 calques**

- Base couleur
- Texture/motif
- Ombres/lumières

**Pour une texture complexe** : **7-10 calques**

- Base
- Texture principale
- Texture secondaire
- Ombres
- Lumières
- Détails fins
- Reflets
- Filtres

**Recommandation** : **5-7 calques** par texture (équilibre qualité/temps)

---

## 🖌️ Créer les Textures

### **A. Texture Murale**

#### **Étape 1 : Base de Couleur**

1. Créer calque "Base Couleur"
2. Utiliser outil **"Remplissage"** (`Paint Bucket`)
3. Couleur : Gris lunaire BBIA (`#EAEAED`) ou votre couleur
4. Remplir toute la toile

#### **Étape 2 : Texture/Motif**

1. Créer calque "Texture"
2. Utiliser pinceaux Procreate :
   - **"Texture"** → "Canvas" (pour effet papier)
   - **"Artistic"** → "Grunge" (pour effet usé)
   - **"Elements"** → "Rocks" (pour effet pierre)
3. Couleur : Légèrement plus sombre/clair que la base
4. Opacité : `30-50%` (subtile)
5. Mode de fusion : `Overlay` ou `Soft Light`

**Astuce** : Dupliquer le calque et déplacer légèrement pour plus de profondeur

#### **Étape 3 : Ombres et Lumières**

1. Créer calque "Ombres" (mode `Multiply`, opacité `20-30%`)
2. Utiliser **Pinceau "Airbrush"** → "Soft Brush"
3. Dessiner ombres aux bords/corners
4. Créer calque "Lumières" (mode `Screen`, opacité `15-25%`)
5. Ajouter lumières au centre

#### **Étape 4 : Détails Fins** (optionnel)

1. Créer calque "Détails"
2. Pinceaux fins : "Inking" → "Dry Ink"
3. Ajouter imperfections subtiles
4. Opacité : `10-20%`

---

### **B. Texture Sol**

#### **Étape 1 : Base**

1. Calque "Base"
2. Couleur : Gris moyen (`#D9D9DC`)
3. Remplir toute la toile

#### **Étape 2 : Motif Parquet/Carrelage**

**Option A : Parquet** :

1. Créer calque "Parquet"
2. Utiliser outil **"Formes"** → Rectangle
3. Créer planches : rectangles horizontaux
4. Espacement régulier
5. Couleur : Légèrement plus sombre/clair

**Option B : Carrelage** :

1. Créer calque "Carrelage"
2. Grille de carrés : Outil **"Formes"** → Rectangle (carré)
3. Répéter avec **"Sélection"** → **"Copier/Coller"**
4. Jointures : Lignes fines entre carrés

**Option C : Moquette** :

1. Créer calque "Moquette"
2. Pinceau "Texture" → "Hair"
3. Créer effet fibre/tissu
4. Direction uniforme

#### **Étape 3 : Jointures/Interstices**

1. Calque "Jointures"
2. Pinceau fin : "Inking" → "Technical Pen"
3. Lignes fines entre éléments (parquet/carrelage)
4. Couleur : Gris foncé (`#888888`)
5. Opacité : `60-80%`

#### **Étape 4 : Usure/Patine**

1. Calque "Usure"
2. Pinceau "Grunge" ou "Damaged"
3. Ajouter zones usées (zones de passage)
4. Opacité : `20-40%`
5. Mode : `Overlay`

---

### **C. Texture Plafond** (optionnel)

1. Calque "Base" : Blanc cassé (`#F0F0F5`)
2. Calque "Texture" : Légère texture (subtile)
3. Calque "Ombres" : Ombres très légères aux bords
4. **Astuce** : Plafond souvent simple (moins de détails que mur/sol)

---

## 💡 Conseils Professionnels

### **1. Répétabilité (Tiling)**

**Important** : Les textures doivent pouvoir se répéter sans couture visible !

**Technique** :

1. Créer motif sur une zone (ex: 512x512px)
2. **Copier/Coller** pour remplir 4096x4096px
3. **Fusionner** les calques de motif
4. **Masque** ou **Effacement** aux bords pour transition douce
5. **Test** : Vérifier que bords se connectent bien

**Outil Procreate** : **"Actions"** → **"Symétrie"** pour motifs symétriques

### **2. Cohérence des Couleurs**

**Palette BBIA recommandée** :

- Gris lunaire : `#EAEAED`
- Bleu céleste : `#87BCFA`
- Violet : `#A680FF`
- Turquoise : `#60E9E1`
- Rose pastel : `#FFDAEC`

**Astuce** : Créer palette Procreate avec ces couleurs pour cohérence

### **3. Pinceaux Procreate Recommandés**

**Pour Textures** :

- `Texture` → `Canvas` (fond texture)
- `Artistic` → `Grunge` (effet usé)
- `Elements` → `Rocks`, `Foliage` (détails naturels)

**Pour Ombres/Lumières** :

- `Airbrushing` → `Soft Brush` (dégradés doux)

**Pour Lignes/Détails** :

- `Inking` → `Technical Pen` (précision)
- `Inking` → `Dry Ink` (détails fins)

### **4. Organisation des Calques**

**Nommage clair** :

- ✅ `Base`, `Texture`, `Ombres`, `Lumières`, `Détails`
- ❌ `Calque 1`, `Calque 2`, `Calque copie`

**Groupes** :

- Créer **groupes** par texture (`Mur`, `Sol`, `Plafond`)
- Facilite la gestion et export sélectif

**Ordre des calques** (du bas vers le haut) :

1. Base
2. Texture principale
3. Texture secondaire
4. Ombres
5. Lumières
6. Détails
7. Filtres

### **5. Mode de Fusion des Calques**

**Modes utiles** :

- `Normal` : Base, détails
- `Overlay` : Textures, contrastes
- `Soft Light` : Lumières/ombres subtiles
- `Multiply` : Ombres
- `Screen` : Lumières
- `Color Burn` : Contraste fort

**Astuce** : Tester différents modes pour trouver le meilleur effet

### **6. Opacité Stratégique**

**Règles générales** :

- Base : `100%`
- Textures : `30-60%`
- Ombres : `20-40%`
- Lumières : `15-30%`
- Détails : `10-30%`

**Astuce** : Mieux vaut sous-exposer que sur-exposer (ajuster ensuite)

### **7. Pensez à la Perspective MuJoCo**

**Dans MuJoCo** :

- Les textures sont projetées sur des surfaces planes
- Pas de perspective 3D dans la texture elle-même
- **Astuce** : Créer textures "plates" (sans perspective), MuJoCo gère la 3D

---

## 📤 Export et Optimisation

### **1. Export PNG**

**Paramètres Procreate** :

1. **Actions** → **Partager** → **PNG**
2. **Qualité** : `Maximum`
3. **Fond transparent** : Activé (si besoin)
4. **Résolution** : `4096 x 4096 px` (ou 2048x2048)

**Nommage** :

- `mur.png` (pour mur)
- `sol.png` (pour sol)
- `plafond.png` (pour plafond)

### **2. Optimisation (optionnel)**

**Avant export** :

- Fusionner calques inutiles
- Vérifier poids fichier (cibler < 10 MB si possible)
- Tester texture dans MuJoCo avant finalisation

**Outil** : **"Actions"** → **"Aplatir"** pour fusionner tous calques (garder version avec calques séparés !)

---

## 🔄 Intégration dans MuJoCo

### **1. Placer Fichiers**

```bash
# Créer textures Procreate et placer ici :
assets/textures/mur.png
assets/textures/sol.png
assets/textures/plafond.png

```

### **2. Créer Scène XML**

**Fichier** : `src/bbia_sim/sim/scenes/ma_scene_procreate.xml`

```xml
<mujoco model="scene_procreate">
  <compiler angle="radian" meshdir="../../assets/reachy_official"/>

  <asset>
    <!-- Vos textures Procreate -->
    <texture name="texture_mur" type="2d" file="../../../assets/textures/mur.png"/>
    <texture name="texture_sol" type="2d" file="../../../assets/textures/sol.png"/>
    <texture name="texture_plafond" type="2d" file="../../../assets/textures/plafond.png"/>

    <!-- Matériaux -->
    <material name="mat_mur" texture="texture_mur"/>
    <material name="mat_sol" texture="texture_sol"/>
    <material name="mat_plafond" texture="texture_plafond"/>

    <!-- Skybox BBIA -->
    <texture name="skybox_bbia" type="skybox" builtin="gradient"
             rgb1="0.92 0.92 0.93" rgb2="0.53 0.74 0.98"/>
  </asset>

  <worldbody>
    <!-- Sol avec votre texture -->
    <geom name="sol" type="plane" size="5 5" material="mat_sol"/>

    <!-- Murs avec vos textures -->
    <body name="mur_fond" pos="0 4 1.5">
      <geom type="box" size="0.1 5 2" material="mat_mur"/>
    </body>

    <!-- Robot Reachy -->
    <include file="../models/reachy_mini_REAL_OFFICIAL.xml"/>
  </worldbody>
</mujoco>

```

### **3. Visualiser**

```bash
source venv/bin/activate
python examples/view_scene_piece.py src/bbia_sim/sim/scenes/ma_scene_procreate.xml

```

---

## ✅ Checklist Complète

### **Avant de Commencer** :

- [ ] Document créé : 4096x4096px, RGB, sRGB
- [ ] Palette BBIA créée dans Procreate
- [ ] Structure calques planifiée

### **Pendant la Création** :

- [ ] Base couleur créée
- [ ] Textures ajoutées (5-7 calques)
- [ ] Ombres/lumières équilibrées
- [ ] Détails fins ajoutés (optionnel)
- [ ] Calques bien nommés et organisés

### **Avant Export** :

- [ ] Texture répétable testée (tiling)
- [ ] Opacités vérifiées (pas trop fort)
- [ ] Cohérence couleurs vérifiée
- [ ] Calques groupés et organisés
- [ ] Version avec calques sauvegardée (pour modifications)

### **Après Export** :

- [ ] PNG haute résolution exporté
- [ ] Fichiers placés dans `assets/textures/`
- [ ] Scène XML créée avec références textures
- [ ] Testé dans MuJoCo
- [ ] Ajustements si nécessaire

---

## 🎯 Résumé : Structure Idéale

**Pour une texture de mur complète** :

1. **Calque "Base"** (100% opacité) : Couleur principale
2. **Calque "Texture"** (40% opacité, Overlay) : Motif/texture
3. **Calque "Ombres"** (25% opacité, Multiply) : Ombres aux bords
4. **Calque "Lumières"** (20% opacité, Screen) : Lumières au centre
5. **Calque "Détails"** (15% opacité, Normal) : Imperfections subtiles

**Total** : 5 calques par texture = **Équilibre optimal qualité/temps**

---

## 💬 Conseils Bonus

### **Astuce 1 : Utiliser Références**

- Importer photos réelles de murs/sols pour inspiration
- Ne pas copier directement (droits), utiliser comme référence

### **Astuce 2 : Variantes Rapides**

- Créer version avec calques séparés
- Dupliquer document
- Ajuster opacités/couleurs = nouvelle variante en 2 minutes

### **Astuce 3 : Masques et Sélections**

- Utiliser **masques** pour transitions douces
- **Sélections** pour zones précises
- **Transformations** pour ajustements

### **Astuce 4 : Pinceaux Personnalisés**

- Créer vos propres pinceaux pour style unique
- Sauvegarder dans Procreate pour réutilisation

### **Astuce 5 : Test dans MuJoCo**

- Exporter version temporaire
- Tester dans MuJoCo
- Ajuster opacités/contrastes selon rendu 3D
- Réitérer si besoin

---

## 📚 Ressources

- **Documentation Procreate** : https://procreate.com/handbook
- **Tutoriels Texture** : Rechercher "Procreate texture tutorial" sur YouTube
- **Guide MuJoCo** : `docs/simulations/GUIDE_IMPORT_IMAGES_MUJOCO.md`
- **Conseils Robot** : `docs/simulations/CONSEILS_PROCREATE_ROBOT.md`
- **Résumé Rapide** : `docs/simulations/RESUME_RAPIDE_PROCREATE.md`

---

## 🔗 Guides Complémentaires

- **`GUIDE_IMPORT_IMAGES_MUJOCO.md`** : Comment importer textures dans MuJoCo
- **`CONSEILS_PROCREATE_ROBOT.md`** : Spécifique pour image robot (si besoin)
- **`RESUME_RAPIDE_PROCREATE.md`** : Version condensée (2 minutes)

---

## 🎯 Navigation

**Retour à** : [README Documentation](../README.md)
**Voir aussi** : [Guide Import Images](GUIDE_IMPORT_IMAGES_MUJOCO.md) • [Conseils Robot](CONSEILS_PROCREATE_ROBOT.md) • [Index Thématique](../reference/INDEX_THEMATIQUE.md)

---

*Guide Procreate Complet - BBIA-SIM - 21 Novembre 2025*
