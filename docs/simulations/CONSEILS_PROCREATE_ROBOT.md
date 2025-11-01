# 🤖 Conseils Procreate - Image du Robot dans la Scène

> **Guide spécifique** : Comment intégrer visuellement le robot Reachy Mini dans vos textures

---

## 🎯 Approche : Texture OU Objet 3D ?

### **Option 1 : Robot en Texture (Image 2D)** 

**Quand utiliser** :
- Pour éléments décoratifs (poster robot sur mur)
- Pour détails visuels lointains
- Pour style artistique

**Limitations** :
- Pas d'interaction 3D
- Pas de mouvement
- Statique uniquement

### **Option 2 : Robot 3D MuJoCo (Recommandé)** ⭐

**Quand utiliser** :
- Robot principal de la scène
- Besoin d'interaction/mouvement
- Simulation réaliste

**Avantages** :
- ✅ 3D réel
- ✅ Mouvement/interaction
- ✅ Conforme au vrai robot

**Recommandation** : **Utiliser le robot 3D MuJoCo** (chargé via `<include>`) plutôt que texture 2D.

---

## 🎨 Si Vous Voulez Ajouter le Robot en Texture (Optionnel)

### **Technique : Dessiner Robot sur Mur (Poster/Tableau)**

**Étape 1 : Préparation**
1. Document Procreate : 1024x1024px (assez pour détail)
2. Fond transparent

**Étape 2 : Dessin du Robot**

**Calques recommandés** (5 calques) :
1. **Calque "Base"** : Forme silhouette robot (gris clair)
2. **Calque "Détails"** : Yeux, antennes, contours
3. **Calque "Ombres"** : Ombres pour profondeur
4. **Calque "Lumières"** : Reflets (optionnel)
5. **Calque "Fond/Cadre"** : Si c'est un tableau/poster

**Style** :
- Simplifié (pas besoin de détails ultra-réalistes)
- Silhouette reconnaissable
- Couleurs cohérentes BBIA

**Étape 3 : Export**
- PNG transparent
- Placer dans `assets/textures/poster_robot.png`

**Étape 4 : Intégration XML**
```xml
<texture name="poster_robot" type="2d" file="../../../assets/textures/poster_robot.png"/>
<material name="mat_poster" texture="poster_robot"/>
<body name="tableau_mur" pos="0 1.8 1.2">
  <geom type="box" size="0.01 0.5 0.5" material="mat_poster"/>
</body>
```

---

## 💡 Conseils pour Dessiner le Robot

### **1. Références Visuelles**

**Utilisez** :
- Photos du vrai Reachy Mini
- Schémas/mesures du projet (`MESURES_REACHY_MINI.md`)
- Modèle 3D MuJoCo comme référence

**Ne copiez pas** :
- Logos/illustrations copyright
- Images protégées

### **2. Style Simplifié**

**Caractéristiques essentielles** :
- ✅ Corps ovoïde volumineux
- ✅ Tête rectangulaire arrondie
- ✅ 2 grands yeux ronds
- ✅ Barre horizontale entre yeux
- ✅ 2 antennes fines

**Simplification** :
- Contours nets
- Couleurs aplaties
- Ombres stylisées (pas ultra-réalistes)

### **3. Palette Couleurs BBIA**

**Couleurs recommandées** :
- Corps : Gris lunaire `#EAEAED`
- Yeux : Noir `#1A1A1A`
- Accents : Bleu céleste `#87BCFA` ou Turquoise `#60E9E1`

### **4. Techniques Procreate**

**Outils utiles** :
- **Formes** : Pour corps/yeux (cercles, rectangles arrondis)
- **Pinceau "Technical Pen"** : Pour contours nets
- **Pinceau "Airbrush"** : Pour ombres douces
- **Symétrie** : Pour antennes identiques

**Astuce** : Activez **"Drawing Guide"** → **"Symmetry"** pour antennes parfaitement identiques

---

## 🎨 Intégration dans Scène : Recommandation

### **Scénario Idéal** :

**Robot principal** : Utiliser modèle 3D MuJoCo (`<include file="reachy_mini_REAL_OFFICIAL.xml"/>`)

**Décor robot** : Texture 2D optionnelle
- Poster robot sur mur (décoratif)
- Petites figurines robot (style)
- Éléments graphiques robotiques

**Pourquoi** :
- Robot 3D = Interaction, mouvement, réalisme
- Textures robot = Décor, style, ambiance

---

## ✅ Checklist Robot en Texture (Si Optionnel)

- [ ] Document 1024x1024px créé
- [ ] 5 calques organisés (Base, Détails, Ombres, Lumières, Fond)
- [ ] Silhouette reconnaissable (corps ovoïde, tête rectangulaire, yeux, antennes)
- [ ] Couleurs BBIA cohérentes
- [ ] Export PNG transparent
- [ ] Placé dans `assets/textures/poster_robot.png`
- [ ] Intégré dans XML comme géométrie plane

---

## 🎯 Résumé : Robot dans Scène

**Recommandation principale** :
- ✅ **Robot principal** : Modèle 3D MuJoCo (chargé via include)
- ✅ **Robot décor** : Texture 2D optionnelle (poster/tableau)

**Pas besoin de** :
- ❌ Dessiner robot complet en texture (3D mieux)
- ❌ Créer texture robot complexe (3D gère ça)

**Focus sur** :
- ✅ Textures **environnement** (mur, sol, plafond)
- ✅ Décor optionnel (poster robot, éléments style)

---

*Conseils Robot - BBIA-SIM - 2025-10-31*

