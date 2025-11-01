# 🎨 Scènes MuJoCo BBIA

> **Note** : Créez vos propres scènes avec vos textures Procreate !

---

## 📋 Comment Créer Votre Scène

### **1. Créer Textures avec Procreate**

Créez vos textures dans Procreate :
- `mur.png` - Texture murale (2048x2048px recommandé)
- `sol.png` - Texture sol
- `plafond.png` - Texture plafond (optionnel)

Placez dans : `assets/textures/`

---

### **2. Créer Scène XML**

Créez votre fichier XML dans : `src/bbia_sim/sim/scenes/votre_scene.xml`

**Exemple minimal** :
```xml
<mujoco model="ma_scene">
  <compiler angle="radian" meshdir="../../assets/reachy_official"/>
  
  <asset>
    <!-- Vos textures Procreate -->
    <texture name="mur" type="2d" file="../../../assets/textures/mur.png"/>
    <texture name="sol" type="2d" file="../../../assets/textures/sol.png"/>
    
    <material name="mat_mur" texture="mur"/>
    <material name="mat_sol" texture="sol"/>
  </asset>
  
  <worldbody>
    <!-- Sol -->
    <geom type="plane" size="5 5" material="mat_sol"/>
    
    <!-- Murs -->
    <body pos="0 4 1">
      <geom type="box" size="0.1 5 2" material="mat_mur"/>
    </body>
    
    <!-- Robot Reachy -->
    <include file="../models/reachy_mini_REAL_OFFICIAL.xml"/>
  </worldbody>
</mujoco>
```

---

### **3. Visualiser Votre Scène**

```bash
# Activer venv
source venv/bin/activate

# Lancer votre scène
python examples/view_scene_piece.py src/bbia_sim/sim/scenes/votre_scene.xml
```

---

## 📚 Documentation Complète

### **Guides Procreate** (Recommandé pour créer vos textures) :
- **🚀 Résumé Rapide** : `docs/simulations/RESUME_RAPIDE_PROCREATE.md` (2 minutes)
- **📖 Guide Complet** : `docs/simulations/GUIDE_PROCREATE_SCENE_COMPLET.md` (référence complète)
- **🤖 Conseils Robot** : `docs/simulations/CONSEILS_PROCREATE_ROBOT.md` (robot en texture)
- **📚 Index** : `docs/simulations/INDEX_GUIDES_PROCREATE.md` (navigation tous guides)

### **Guides MuJoCo** :
- **Guide Import Images** : `docs/simulations/GUIDE_IMPORT_IMAGES_MUJOCO.md`
- **Dossier Textures** : `assets/textures/README.md`

---

## 🎮 Configuration Caméra

Ajustez dans `examples/view_scene_piece.py` :
- `azimuth` : Angle horizontal (180° = face)
- `elevation` : Angle vertical (-15° = vue de haut)
- `distance` : Distance caméra (2.5 = distance moyenne)
- `lookat` : Point de visée [x, y, z]

---

*Documentation Scènes - BBIA-SIM - 2025-01-31*
