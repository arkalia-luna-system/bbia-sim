# 🎨 Guide - Importer Images dans MuJoCo

> **Question** : Peut-on importer des images créées (ex: pièce avec audit) dans MuJoCo pour les scènes 3D ?  
> **Réponse** : ✅ **OUI, C'EST POSSIBLE ET RELATIVEMENT SIMPLE !**

---

## ✅ Réponse Directe

**OUI**, vous pouvez importer vos images (textures) dans MuJoCo pour :
- ✅ **Murs de pièce** : Image de texture murale
- ✅ **Sol/Plafond** : Image de texture sol/plafond
- ✅ **Objets décoratifs** : Images sur surfaces planes
- ✅ **Skybox** : Image d'environnement 360° (déjà utilisé dans le projet)
- ✅ **Tout décor visuel** : Plus facile que coder en 3D

**Avantage** : **Beaucoup plus simple** que de tout modéliser en 3D !  
Vous dessinez vos textures → MuJoCo les applique automatiquement.

---

## 🎨 Comment MuJoCo Gère les Images

### **Types de Textures MuJoCo**

1. **Texture 2D** (image PNG/JPG) :
   - Pour murs, sol, objets
   - Format : PNG, JPG, BMP

2. **Skybox** (environnement 360°) :
   - Fond d'environnement
   - Déjà utilisé dans le projet (dégradé BBIA)

3. **Texture Cubemap** (6 faces cube) :
   - Pour environnement complet

---

## 📐 Exemple : Pièce avec Images

### **Structure de Base**

```xml
<mujoco model="piece_avec_images">
  <compiler angle="radian"/>
  
  <asset>
    <!-- VOS IMAGES (placez dans assets/textures/) -->
    <texture name="mur_texture" type="2d" file="assets/textures/mur_procreate.png"/>
    <texture name="sol_texture" type="2d" file="assets/textures/sol_procreate.png"/>
    <texture name="plafond_texture" type="2d" file="assets/textures/plafond_procreate.png"/>
    
    <!-- Matériaux utilisant vos textures -->
    <material name="mat_mur" texture="mur_texture"/>
    <material name="mat_sol" texture="sol_texture"/>
    <material name="mat_plafond" texture="plafond_texture"/>
  </asset>
  
  <worldbody>
    <!-- Sol avec votre texture -->
    <geom name="sol" type="plane" size="2 2 0.1" material="mat_sol"/>
    
    <!-- Murs avec vos textures -->
    <body name="mur_fond" pos="0 0 1">
      <geom type="box" size="0.1 2 1" material="mat_mur"/>
    </body>
    
    <!-- Robot Reachy au centre -->
    <include file="reachy_mini_REAL_OFFICIAL.xml"/>
  </worldbody>
</mujoco>
```

---

## 🖼️ Formats d'Images Supportés

### **Formats Acceptés**
- ✅ **PNG** (recommandé : transparence)
- ✅ **JPG/JPEG**
- ✅ **BMP**
- ✅ **TGA**

### **Tailles Recommandées**
- **Textures murs/sol** : 1024x1024px ou 2048x2048px
- **Détails fins** : 512x512px minimum
- **Skybox** : 2048x2048px (ou 4096x4096px pour qualité max)

**Note** : MuJoCo redimensionne automatiquement si nécessaire.

---

## 🔄 Workflow avec Procreate

### **Étape 1 : Créer vos Textures**

**Dans Procreate** :
1. Créez vos textures (mur, sol, plafond, décor)
2. Taille recommandée : **2048x2048px** (qualité pro)
3. Export PNG haute résolution

**Textures à créer** :
- `mur.png` - Texture murale (peut être répétée)
- `sol.png` - Texture sol
- `plafond.png` - Texture plafond (optionnel)
- `decor.png` - Éléments décoratifs

### **Étape 2 : Organiser les Fichiers**

```
bbia-reachy-sim/
├── assets/
│   └── textures/           # NOUVEAU : Dossier textures
│       ├── mur.png
│       ├── sol.png
│       ├── plafond.png
│       └── decor.png
└── src/bbia_sim/sim/
    ├── models/
    └── scenes/
        └── piece_bbia.xml  # NOUVEAU : Scène avec vos textures
```

### **Étape 3 : Créer le XML MuJoCo**

**Fichier** : `src/bbia_sim/sim/scenes/piece_bbia.xml`

```xml
<mujoco model="piece_bbia">
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
    
    <!-- Skybox BBIA (déjà existant) -->
    <texture name="skybox_bbia" type="skybox" builtin="gradient" 
             rgb1="0.92 0.92 0.93" rgb2="0.53 0.74 0.98"/>
  </asset>
  
  <worldbody>
    <!-- Sol avec votre texture -->
    <geom name="sol" type="plane" size="3 3 0.1" material="mat_sol" pos="0 0 0"/>
    
    <!-- Murs (4 murs de la pièce) -->
    <body name="mur_fond" pos="0 2 1">
      <geom type="box" size="0.1 3 1.5" material="mat_mur"/>
    </body>
    <body name="mur_gauche" pos="-2 0 1">
      <geom type="box" size="0.1 3 1.5" material="mat_mur"/>
    </body>
    <body name="mur_droite" pos="2 0 1">
      <geom type="box" size="0.1 3 1.5" material="mat_mur"/>
    </body>
    
    <!-- Plafond -->
    <body name="plafond" pos="0 0 2.5">
      <geom type="box" size="3 3 0.1" material="mat_plafond"/>
    </body>
    
    <!-- Bureau (existant) -->
    <body name="bureau_bbia" pos="0 0 0.16">
      <geom name="tabletop" type="box" size="0.4 0.3 0.01" pos="0 0 0.16" rgba="1.0 1.0 1.0 1.0"/>
    </body>
    
    <!-- Robot Reachy Mini -->
    <include file="../models/reachy_mini_REAL_OFFICIAL.xml"/>
  </worldbody>
</mujoco>
```

---

## 🎯 Exemples Concrets

### **Exemple 1 : Pièce avec Texture Murale Simple**

**Image Procreate** : `mur.png` (2048x2048px, texture murale)

**Dans XML** :
```xml
<texture name="mur" type="2d" file="assets/textures/mur.png"/>
<material name="mat_mur" texture="mur"/>
<geom type="box" size="0.1 3 2" material="mat_mur"/>
```

**Résultat** : Votre image appliquée sur le mur !

---

### **Exemple 2 : Sol avec Texture Personnalisée**

**Image Procreate** : `sol.png` (parquet, carrelage, moquette, etc.)

**Dans XML** :
```xml
<texture name="sol" type="2d" file="assets/textures/sol.png"/>
<material name="mat_sol" texture="sol"/>
<geom name="sol" type="plane" size="3 3" material="mat_sol"/>
```

**Résultat** : Votre sol personnalisé !

---

### **Exemple 3 : Objets Décoratifs**

**Image Procreate** : `tableau.png`, `plante.png`, etc.

**Dans XML** :
```xml
<texture name="tableau" type="2d" file="assets/textures/tableau.png"/>
<material name="mat_tableau" texture="tableau"/>
<body name="tableau_mur" pos="0 1.8 1.2">
  <geom type="box" size="0.01 0.5 0.5" material="mat_tableau"/>
</body>
```

---

## 💡 Avantages vs Coder en 3D

### **Avec Images (Procreate)**
- ✅ **Plus rapide** : Vous dessinez → MuJoCo applique
- ✅ **Plus créatif** : Liberté totale sur le design
- ✅ **Plus simple** : Pas besoin de modéliser en 3D
- ✅ **Plus réaliste** : Textures photographiques possibles
- ✅ **Plus flexible** : Changement d'image = changement d'ambiance

### **Sans Images (Code pur)**
- ⚠️ **Plus long** : Modéliser chaque détail en 3D
- ⚠️ **Plus technique** : Nécessite connaissance 3D avancée
- ⚠️ **Moins créatif** : Limité par les primitives géométriques

**Conclusion** : Utiliser vos images Procreate est **beaucoup plus simple** et créatif !

---

## 🔧 Configuration Technique

### **Chemins de Fichiers**

**Option 1 : Chemin relatif depuis XML**
```xml
<texture name="mur" type="2d" file="../../../assets/textures/mur.png"/>
```

**Option 2 : Chemin absolu** (si images ailleurs)
```xml
<texture name="mur" type="2d" file="/Volumes/T7/bbia-reachy-sim/assets/textures/mur.png"/>
```

**Recommandation** : Utiliser chemin relatif pour portabilité.

---

## 📋 Checklist pour Créer une Scène

1. [ ] **Créer textures Procreate** :
   - Mur (2048x2048px)
   - Sol (2048x2048px)
   - Plafond (optionnel, 2048x2048px)
   - Décor (tailles selon usage)

2. [ ] **Organiser fichiers** :
   - Créer `assets/textures/`
   - Placer images PNG

3. [ ] **Créer XML scène** :
   - Définir textures dans `<asset>`
   - Créer matériaux
   - Appliquer aux géométries (murs, sol, plafond)

4. [ ] **Tester** :
   - Charger scène dans MuJoCo
   - Vérifier textures affichées
   - Ajuster si nécessaire

---

## 🎨 Cas d'Usage : "Pièce avec Audit"

**Votre cas** : Pièce avec éléments auditifs (audio/son)

**Approche** :
1. **Textures visuelles** : Murs, sol, plafond avec Procreate
2. **Positionnement objets** : Haut-parleurs, microphones visibles dans scène
3. **Audio** : Géré séparément par le code Python (pas dans MuJoCo directement)

**MuJoCo gère** :
- ✅ Visuel (textures, objets 3D)
- ✅ Positionnement objets audio visuels

**Code Python gère** :
- ✅ Audio réel (son, microphone)
- ✅ Interaction audio-robot

**C'est complémentaire** : MuJoCo = visuel, Code = fonctionnel (audio, logique).

---

## 🚀 Démarrage Rapide

### **Étape 1 : Créer Dossier Textures**

```bash
mkdir -p /Volumes/T7/bbia-reachy-sim/assets/textures
```

### **Étape 2 : Dessiner Textures Procreate**

- Mur : 2048x2048px
- Sol : 2048x2048px
- Export PNG

### **Étape 3 : Créer XML Scène**

Copier exemple ci-dessus, adapter chemins fichiers.

### **Étape 4 : Charger Scène**

```python
from bbia_sim.sim.simulator import MuJoCoSimulator

sim = MuJoCoSimulator()
sim.load_scene("piece_bbia.xml")  # Votre scène avec textures
sim.launch_simulation()
```

---

## ✅ Réponse à Votre Question

**"Est-ce que je peux importer des images dans MuJoCo pour une pièce avec audit ?"**

**OUI, ABSOLUMENT** :

1. ✅ **Textures images** : MuJoCo supporte PNG/JPG directement
2. ✅ **Plus simple** : Dessiner avec Procreate vs coder en 3D
3. ✅ **Pièce complète** : Murs, sol, plafond avec vos images
4. ✅ **Audit/audio** : Positions visuelles dans MuJoCo, audio géré par code Python

**C'est réellement possible et plus facile que tout coder** ! 🎨

---

## 📚 Références

- **Documentation MuJoCo Textures** : https://mujoco.readthedocs.io/en/latest/modeling.html#texture
- **Exemple projet** : `reachy_mini_REAL_OFFICIAL.xml` (ligne 534 : skybox)

---

*Guide Import Images MuJoCo - BBIA-SIM - 2025-10-31*

