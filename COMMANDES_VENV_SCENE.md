# 🚀 Commandes Complètes - Venv + Visualiser Scène

## 📋 Activation du Venv Principal (MuJoCo/Simulation)

### **Sur macOS/Linux** :

```bash
# Aller dans le dossier du projet
cd /Volumes/T7/bbia-reachy-sim

# Activer le venv principal (pour MuJoCo)
source venv/bin/activate
```

### **Vérifier que c'est actif** :

Le prompt devrait afficher `(venv)` :
```bash
# Vous devriez voir : (venv) dans votre prompt
which python
# Devrait afficher : /Volumes/T7/bbia-reachy-sim/venv/bin/python
```

---

## 🎬 Visualiser une Scène MuJoCo

### **Méthode 1 : Script Python** (Recommandé)

```bash
# 1. Activer le venv
cd /Volumes/T7/bbia-reachy-sim
source venv/bin/activate

# 2. Lancer une scène (spécifiez le chemin)
python examples/view_scene_piece.py chemin/vers/votre_scene.xml

# Exemple : scène par défaut (minimal.xml)
python examples/view_scene_piece.py
```

**Sur macOS** (si viewer ne s'ouvre pas avec `python`) :
```bash
# Activer le venv
source venv/bin/activate

# Utiliser mjpython (viewer MuJoCo pour macOS)
mjpython examples/view_scene_piece.py chemin/vers/votre_scene.xml
```

---

## 📝 Commandes Complètes en Une Ligne

### **Pour Visualiser une Scène** :

```bash
# macOS/Linux - Scène par défaut
cd /Volumes/T7/bbia-reachy-sim && source venv/bin/activate && python examples/view_scene_piece.py

# macOS/Linux - Scène personnalisée
cd /Volumes/T7/bbia-reachy-sim && source venv/bin/activate && python examples/view_scene_piece.py src/bbia_sim/sim/scenes/votre_scene.xml

# macOS - Avec mjpython
cd /Volumes/T7/bbia-reachy-sim && source venv/bin/activate && mjpython examples/view_scene_piece.py chemin/vers/scene.xml
```

---

## 🔧 Vérifications Avant Lancement

### **1. Vérifier que MuJoCo est installé** :

```bash
# Activer venv
source venv/bin/activate

# Tester MuJoCo
python -c "import mujoco; print('✅ MuJoCo installé - Version:', mujoco.__version__)"
```

### **2. Vérifier que le script existe** :

```bash
# Activer venv
source venv/bin/activate

# Vérifier fichier
ls -lh examples/view_scene_piece.py
```

### **3. Créer votre scène XML** :

```bash
# Activer venv
source venv/bin/activate

# Créer votre scène (voir docs/simulations/GUIDE_IMPORT_IMAGES_MUJOCO.md)
# Placez dans : src/bbia_sim/sim/scenes/votre_scene.xml
```

---

## 🎮 Contrôles dans le Viewer

Une fois la fenêtre 3D ouverte :
- **Souris** : Rotation de la vue
- **Molette** : Zoom
- **Clic droit** : Déplacer la vue
- **Échap** : Fermer la fenêtre

---

## ⚠️ Résolution de Problèmes

### **Erreur : "Module mujoco non trouvé"**

```bash
# Activer venv
source venv/bin/activate

# Réinstaller MuJoCo
pip install --upgrade mujoco
```

### **Erreur : "Viewer ne s'ouvre pas" (macOS)**

```bash
# Activer venv
source venv/bin/activate

# Installer mujoco-python-viewer
pip install mujoco-python-viewer

# Ou utiliser mjpython
mjpython examples/view_scene_piece.py
```

### **Erreur : "XML Error"**

```bash
# Vérifier la syntaxe XML
python -c "import mujoco; m = mujoco.MjModel.from_xml_path('src/bbia_sim/sim/scenes/piece_bbia_simple.xml'); print('✅ XML valide')"
```

---

## 📁 Structure des Fichiers

```
/Volumes/T7/bbia-reachy-sim/
├── venv/                          # Venv principal (activer ici)
│   └── bin/activate               # Script d'activation
├── examples/
│   └── view_scene_piece.py       # Script pour visualiser scène
├── assets/textures/              # Vos textures Procreate ici
└── src/bbia_sim/sim/scenes/
    ├── minimal.xml                # Scène minimale par défaut
    └── votre_scene.xml            # Créez votre scène ici
```

---

## 🔄 Workflow Complet

### **Étape par étape** :

```bash
# 1. Aller dans le projet
cd /Volumes/T7/bbia-reachy-sim

# 2. Activer venv
source venv/bin/activate

# 3. Vérifier MuJoCo
python -c "import mujoco; print('✅ MuJoCo OK')"

# 4. Lancer la scène
python examples/view_scene_piece.py
```

---

## 💡 Alternatives

### **Via CLI BBIA-SIM** :

```bash
# Activer venv
source venv/bin/activate

# Charger scène via CLI
python -m bbia_sim --sim --scene piece_bbia_simple.xml
```

### **Mode Headless** (sans viewer) :

```bash
# Activer venv
source venv/bin/activate

# Lancer sans interface graphique
python -m bbia_sim --sim --scene piece_bbia_simple.xml --headless
```

---

## 🎨 Créer Votre Scène avec Procreate

**Workflow complet** :

1. **Créer textures** : `mur.png`, `sol.png`, `plafond.png` (2048x2048px) avec Procreate
2. **Placer dans** : `assets/textures/`
3. **Créer scène XML** : `src/bbia_sim/sim/scenes/votre_scene.xml` (voir guide ci-dessous)
4. **Visualiser** : `python examples/view_scene_piece.py src/bbia_sim/sim/scenes/votre_scene.xml`

**Guide complet** : `docs/simulations/GUIDE_IMPORT_IMAGES_MUJOCO.md`

---

## ✅ Checklist Rapide

- [ ] Venv activé : `source venv/bin/activate`
- [ ] MuJoCo installé : `python -c "import mujoco"`
- [ ] Script existe : `ls examples/view_scene_piece.py`
- [ ] Scène existe : `ls src/bbia_sim/sim/scenes/piece_bbia_simple.xml`
- [ ] Fenêtre 3D ouverte : Voir la pièce dans le viewer

---

*Guide Commandes - BBIA-SIM - 2025-01-31*

