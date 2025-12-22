# 📚 Index - Guides Procreate pour Scènes MuJoCo

**Dernière mise à jour : 15 Décembre 2025

> **Navigation rapide** : Tous les guides disponibles

---

## 🎯 Guide Complet Fusionné

### **📖 Guide Complet Procreate**

**👉 `GUIDE_PROCREATE_COMPLET.md`**

Ce guide unique contient toutes les sections :

- ⚡ **Résumé Rapide** (2 minutes) - Configuration basique, structure calques, export rapide
- 🎨 **Configuration Procreate** - Paramètres détaillés, espace couleur
- 🗂️ **Organisation des Calques** - Structure recommandée, nombre optimal
- 🖌️ **Créer les Textures** - Mur, sol, plafond (étapes détaillées)
- 💡 **Conseils Professionnels** - Tiling, couleurs, pinceaux, modes de fusion
- 📤 **Export et Optimisation** - PNG, optimisation, nommage
- 🔄 **Intégration dans MuJoCo** - XML, visualisation
- 🤖 **Conseils Spécifiques Robot** - Robot 3D vs texture, dessin robot optionnel

**Quand l'utiliser** : Guide de référence complet pour créer vos textures Procreate

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

### **Pour Créer Votre Première Scène**

1. **Lire** : Section "Résumé Rapide" dans `GUIDE_PROCREATE_COMPLET.md` (2 min)
2. **Suivre** : `GUIDE_PROCREATE_COMPLET.md` (étape par étape)
3. **Créer** : Vos textures (mur.png, sol.png, plafond.png)
4. **Intégrer** : `GUIDE_IMPORT_IMAGES_MUJOCO.md` (créer XML)
5. **Visualiser** : `python examples/view_scene_piece.py votre_scene.xml`

---

## 📋 Résumé des Essentiels

### **Configuration Procreate**

- Taille : **4096 x 4096 px**
- Format : **RGB, sRGB**
- Fond : **Transparent** ou **Blanc**

### **Structure Calques (Par Texture)**

- **5 calques** = Équilibre optimal
  1. Base (100%)
  2. Texture (40%, Overlay)
  3. Ombres (25%, Multiply)
  4. Lumières (20%, Screen)
  5. Détails (15%, Normal)

### **Textures à Créer**

- `mur.png` (4096x4096px)
- `sol.png` (4096x4096px)
- `plafond.png` (4096x4096px, optionnel)

### **Export**

- PNG maximum qualité
- Placer dans `assets/textures/`

### **Robot**

- ✅ Utiliser modèle 3D MuJoCo (recommandé)
- ✅ Texture robot optionnelle (pour décor)
  - **Fichier existant** : `assets/textures/robot_reachy_mini.png` (944x712px, créé avec Procreate)

---

## 🔗 Autres Ressources

- **Commandes Venv** : `/docs/development/setup/COMMANDES_VENV_SCENE.md`
- **README Scènes** : `src/bbia_sim/sim/scenes/README_SCENES.md`
- **Guide Simulation** : `MUJOCO_SIMULATION_GUIDE.md`

---

## 🎯 Navigation

**Retour à** : [README Documentation](../README.md)  
**Voir aussi** : [Guide MuJoCo](MUJOCO_SIMULATION_GUIDE.md) • [Index Thématique](../reference/INDEX_THEMATIQUE.md)

---

**Index Guides Procreate - BBIA-SIM - 22 Décembre 2025**
