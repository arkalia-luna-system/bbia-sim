# 🤖 Assemblage Robot Reachy Mini 3D pour Logo BBIA

> **Date** : Décembre 2025  
> **Objectif** : Créer un modèle 3D complet et assemblé du Reachy Mini pour dessiner le logo dans Procreate  
> **Statut** : ✅ **Complété** - Scripts d'assemblage fonctionnels

---

## 🎯 Contexte

Pour créer le logo BBIA avec Procreate sur iPad Pro, il faut un modèle 3D de référence du robot Reachy Mini correctement assemblé avec toutes ses pièces.

**Problème initial** : Les fichiers STL individuels ne sont pas assemblés, et les positions doivent être exactes selon les spécifications officielles.

---

## 📋 Sources de Référence

### 1. **Fichier XML Officiel**
- **Chemin** : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`
- **Source** : Dépôt officiel Pollen Robotics
- **Contenu** : Positions exactes de toutes les pièces du robot (41 composants)

### 2. **PDF d'Assemblage Officiel**
- **URL** : https://www.pollen-robotics.com/wp-content/uploads/2025/10/Reachy_Mini_Assembly_BETA_v2_LOW-compresse.pdf
- **Contenu** : Guide d'assemblage étape par étape avec photos

### 3. **Dépôt GitHub Officiel**
- **URL** : https://github.com/pollen-robotics/reachy_mini
- **Contenu** : Code source, documentation, modèles 3D

---

## 🔧 Solution Implémentée

### **Scripts d'Assemblage Blender**

Création de scripts Python pour Blender qui :
1. Importent tous les fichiers STL
2. Positionnent chaque pièce selon les coordonnées exactes du XML
3. Ajoutent des UV maps (requis par Procreate)
4. Exportent en USDZ (format compatible Procreate)

### **Scripts Créés**

#### 1. `ASSEMBLER_AVEC_VUE.py` ⭐ **Principal**
- **Localisation** : `/Users/athalia/Desktop/logo bbia/scripts/`
- **Fonction** : Assemble les pièces principales avec positions exactes
- **Pièces incluses** :
  - Base (body_foot, body_turning)
  - Corps (body_down, body_top)
  - Tête (head_back, head_front, head_mic)
  - Antennes (gauche et droite avec tous leurs composants)

#### 2. `ASSEMBLER_PARFAIT_PDF.py`
- **Fonction** : Version complète avec toutes les pièces du PDF
- **Inclut** : Lenses, caméra, composants détaillés

#### 3. `ASSEMBLER_FINAL_COMPLET.py`
- **Fonction** : Version simplifiée pour tests rapides

---

## 📐 Calcul des Positions

### **Méthodologie**

1. **Extraction depuis XML** : Positions relatives de chaque pièce dans la hiérarchie
2. **Calcul absolu** : Conversion des positions relatives en coordonnées absolues
3. **Ajustement base** : Mise à z=0 du body_foot (base du robot)

### **Structure Hiérarchique**

```
body_foot (z=0.0) ← BASE
  └── body_down (z=0.23)
      └── body_top (z=0.23)
          └── xl_330 (z≈0.315) ← Tête
              ├── head parts (z=0.34)
              ├── antenna_right (z=0.3105)
              └── antenna_left (z=0.399)
```

### **Positions Clés**

| Pièce | Position Z | Notes |
|-------|-----------|-------|
| `body_foot_3dprint.stl` | 0.0 | Base du robot (en bas) |
| `body_down_3dprint.stl` | 0.23 | Corps inférieur |
| `body_top_3dprint.stl` | 0.23 | Corps supérieur |
| `head_back_3dprint.stl` | 0.34 | Tête (arrière) |
| `head_front_3dprint.stl` | 0.34 | Tête (avant) |
| `antenna_holder_r_3dprint.stl` | 0.3105 | Antenne droite |
| `antenna_holder_l_3dprint.stl` | 0.399 | Antenne gauche |

---

## 🐛 Problèmes Résolus

### **1. Body_foot apparaissait en haut**
- **Cause** : Le mesh body_foot est décalé de +0.2298m dans le XML
- **Solution** : Positionner body_foot à z=-0.2298 pour que le mesh soit à z=0

### **2. Antennes invisibles**
- **Cause** : Positions incorrectes ou pièces manquantes
- **Solution** : Extraction des positions exactes depuis le XML pour les deux antennes

### **3. Assemblage incorrect**
- **Cause** : Positions estimées au lieu d'utiliser le XML officiel
- **Solution** : Calcul précis depuis `reachy_mini_REAL_OFFICIAL.xml`

---

## 📁 Fichiers de Travail

### **Sur le Desktop (Temporaire)**
```
/Users/athalia/Desktop/logo bbia/
├── fichiers_originaux_stl/     # 86 fichiers STL du robot
├── scripts/
│   ├── ASSEMBLER_AVEC_VUE.py   # Script principal ⭐
│   ├── ASSEMBLER_PARFAIT_PDF.py
│   └── ASSEMBLER_FINAL_COMPLET.py
└── fichiers_finaux_usdz/       # (À générer)
```

### **Dans le Projet BBIA (Futur)**
```
presentation/livrables/v1.0/logo/
├── source/
│   └── robot_3d/               # Modèles 3D de référence
└── exports/
    └── robot_reachy_mini.usdz  # Modèle assemblé final
```

---

## ✅ Checklist Finale

- [x] Extraction positions depuis XML officiel
- [x] Création scripts d'assemblage Blender
- [x] Correction position body_foot (en bas)
- [x] Ajout positions antennes exactes
- [x] Ajout UV maps pour Procreate
- [x] Documentation complète
- [ ] Test final dans Blender
- [ ] Export USDZ final
- [ ] Import dans Procreate pour validation

---

## 🔗 Références

- **XML Officiel** : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`
- **PDF Assemblage** : https://www.pollen-robotics.com/wp-content/uploads/2025/10/Reachy_Mini_Assembly_BETA_v2_LOW-compresse.pdf
- **GitHub Officiel** : https://github.com/pollen-robotics/reachy_mini
- **Documentation Logo** : `presentation/livrables/v1.0/logo/README.md`

---

## 📝 Notes Techniques

### **Format USDZ**
- Format Apple pour AR/3D
- Compatible Procreate
- Requiert UV maps sur tous les meshes

### **Blender 4.5.4**
- Version utilisée pour l'assemblage
- Scripts Python avec `bpy` module
- Export via `bpy.ops.wm.usd_export()`

### **Procreate**
- Import direct USDZ
- Permet de dessiner par-dessus le modèle 3D
- Calques séparés pour chaque pièce

---

**Dernière mise à jour** : Décembre 2025

