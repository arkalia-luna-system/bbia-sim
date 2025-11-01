# 📚 Index - Guides Procreate pour Scènes MuJoCo

> **Navigation rapide** : Tous les guides disponibles

---

## 🎯 Guides par Besoin

### **🚀 Démarrage Rapide (2 minutes)**

**👉 `RESUME_RAPIDE_PROCREATE.md`**
- Configuration basique
- Structure calques (5 calques)
- Export rapide
- Checklist express

**Quand l'utiliser** : Pour un aperçu ultra-rapide

---

### **📖 Guide Complet (Référence)**

**👉 `GUIDE_PROCREATE_SCENE_COMPLET.md`**
- Configuration détaillée Procreate
- Organisation calques professionnelle
- Création textures (mur, sol, plafond)
- Conseils pro (tiling, couleurs, pinceaux)
- Export et optimisation
- Intégration MuJoCo complète

**Quand l'utiliser** : Pour créer vos textures avec toutes les techniques

---

### **🤖 Robot dans la Scène**

**👉 `CONSEILS_PROCREATE_ROBOT.md`**
- Robot en texture OU 3D ?
- Dessiner robot (si optionnel)
- Intégration dans scène
- Recommandations

**Quand l'utiliser** : Si vous voulez ajouter robot en texture (décor)

---

### **🔧 Importer dans MuJoCo**

**👉 `GUIDE_IMPORT_IMAGES_MUJOCO.md`**
- Types de textures MuJoCo
- Format XML complet
- Exemples code
- Workflow complet

**Quand l'utiliser** : Pour intégrer vos textures Procreate dans MuJoCo

---

## ✅ Parcours Recommandé

### **Pour Créer Votre Première Scène** :

1. **Lire** : `RESUME_RAPIDE_PROCREATE.md` (2 min)
2. **Suivre** : `GUIDE_PROCREATE_SCENE_COMPLET.md` (étape par étape)
3. **Créer** : Vos textures (mur.png, sol.png, plafond.png)
4. **Intégrer** : `GUIDE_IMPORT_IMAGES_MUJOCO.md` (créer XML)
5. **Visualiser** : `python examples/view_scene_piece.py votre_scene.xml`

---

## 📋 Résumé des Essentiels

### **Configuration Procreate** :
- Taille : **4096 x 4096 px**
- Format : **RGB, sRGB**
- Fond : **Transparent** ou **Blanc**

### **Structure Calques (Par Texture)** :
- **5 calques** = Équilibre parfait
  1. Base (100%)
  2. Texture (40%, Overlay)
  3. Ombres (25%, Multiply)
  4. Lumières (20%, Screen)
  5. Détails (15%, Normal)

### **Textures à Créer** :
- `mur.png` (4096x4096px)
- `sol.png` (4096x4096px)
- `plafond.png` (4096x4096px, optionnel)

### **Export** :
- PNG maximum qualité
- Placer dans `assets/textures/`

### **Robot** :
- ✅ Utiliser modèle 3D MuJoCo (recommandé)
- ✅ Texture robot optionnelle (pour décor)

---

## 🔗 Autres Ressources

- **Commandes Venv** : `/COMMANDES_VENV_SCENE.md`
- **README Scènes** : `src/bbia_sim/sim/scenes/README_SCENES.md`
- **Guide Simulation** : `MUJOCO_SIMULATION_GUIDE.md`

---

*Index Guides Procreate - BBIA-SIM - 2025-10-31*

