# 🤖 Robot Reachy Mini 3D - Guide d'Utilisation

> **Guide complet pour lancer et utiliser le robot Reachy Mini en simulation 3D**

## 🎯 **Vue d'Ensemble**

Le robot Reachy Mini est maintenant disponible en simulation 3D complète avec :
- ✅ **Modèle MJCF corrigé** avec assemblage correct des pièces
- ✅ **Assets officiels** intégrés (STL de Pollen Robotics)
- ✅ **15 articulations** fonctionnelles
- ✅ **Physique réaliste** avec gravité et inertie
- ✅ **Rendu 3D** avec ombres et éclairage

---

## 🚀 **Lancement Rapide**

### 🎮 **Mode Graphique (Fenêtre 3D)**
```bash
# Depuis la racine du projet
./launch_robot.sh

# Ou depuis le dossier scripts
./scripts/launch_robot_3d.sh

# Ou avec Python
python scripts/launch_robot.py
```

### 🔄 **Mode Headless (Sans Fenêtre)**
```bash
# Mode headless continu
./launch_robot.sh headless

# Test rapide (2 secondes)
./launch_robot.sh test
```

---

## 🎮 **Contrôles de la Fenêtre 3D**

### 🖱️ **Navigation**
- **Souris** : Rotation de la vue autour du robot
- **Molette** : Zoom avant/arrière
- **Clic droit** : Déplacer la vue
- **Échap** : Fermer la fenêtre

### 🎯 **Ce que tu devrais voir**
- **🤖 Robot complet** : Toutes les pièces assemblées
- **🎨 Couleurs distinctes** : Bleu (torse), Rouge (tête), Vert (bras)
- **🔧 Articulations** : 15 joints fonctionnels
- **⚡ Physique** : Gravité, inertie, collisions

---

## 🤖 **Articulations Disponibles**

### 🧠 **Tête (3 articulations)**
- `neck_yaw` : Rotation gauche/droite (±90°)
- `neck_pitch` : Inclinaison avant/arrière (±45°)
- `neck_roll` : Roulis gauche/droite (±30°)

### 🤚 **Bras Droit (6 articulations)**
- `right_shoulder_pitch` : Épaule avant/arrière (±90°)
- `right_shoulder_roll` : Épaule gauche/droite (±90°)
- `right_elbow_pitch` : Coude flexion/extension (±90°)
- `right_wrist_pitch` : Poignet avant/arrière (±90°)
- `right_wrist_roll` : Poignet rotation (±90°)
- `right_gripper_joint` : Pince ouverture/fermeture (±0.5 rad)

### 🤚 **Bras Gauche (6 articulations)**
- `left_shoulder_pitch` : Épaule avant/arrière (±90°)
- `left_shoulder_roll` : Épaule gauche/droite (±90°)
- `left_elbow_pitch` : Coude flexion/extension (±90°)
- `left_wrist_pitch` : Poignet avant/arrière (±90°)
- `left_wrist_roll` : Poignet rotation (±90°)
- `left_gripper_joint` : Pince ouverture/fermeture (±0.5 rad)

---

## 🎯 **Contrôle via API**

### 🌐 **Lancement de l'API**
```bash
# Terminal 1 : API
uvicorn src.bbia_sim.daemon.app.main:app --port 8000 --host 0.0.0.0

# Terminal 2 : Robot 3D
./launch_robot.sh
```

### 📡 **Commandes API**
```bash
# Mouvement de la tête
curl -X POST "http://localhost:8000/api/motion/joints" \
  -H "Authorization: Bearer bbia-secret-key-dev" \
  -H "Content-Type: application/json" \
  -d '[{"joint_name": "neck_yaw", "position": 0.5}]'

# Mouvement du bras droit
curl -X POST "http://localhost:8000/api/motion/joints" \
  -H "Authorization: Bearer bbia-secret-key-dev" \
  -H "Content-Type: application/json" \
  -d '[{"joint_name": "right_shoulder_pitch", "position": -0.8}]'

# État des articulations
curl -X GET "http://localhost:8000/api/state/joints" \
  -H "Authorization: Bearer bbia-secret-key-dev"
```

---

## 🧪 **Scripts de Démonstration**

### 🎭 **Démo Salut (Wave)**
```bash
./scripts/demo_wave.sh
```

### 🎭 **Démo Hochement (Nod)**
```bash
./scripts/demo_nod.sh
```

### 📊 **Visualisation Temps Réel**
```bash
python examples/visualize_movements.py --demo monitor --duration 10
```

---

## 🔧 **Dépannage**

### ❌ **Problèmes Courants**

#### **Fenêtre 3D ne s'ouvre pas (macOS)**
```bash
# Solution : Utiliser mjpython
mjpython scripts/launch_complete_robot.py

# Ou installer le viewer
pip install mujoco-python-viewer
```

#### **Robot apparaît en pièces détachées**
- ✅ **Solution** : Utilise le modèle `reachy_mini_complete.xml`
- ❌ **Ancien** : `reachy_mini.xml` (pièces séparées)

#### **Erreur "Modèle non trouvé"**
```bash
# Vérifier que le modèle existe
ls -la src/bbia_sim/sim/models/reachy_mini_complete.xml

# Relancer depuis la racine
cd /Volumes/T7/bbia-reachy-sim
./launch_robot.sh
```

### ✅ **Solutions**

#### **Vérification Rapide**
```bash
# Test du modèle
./launch_robot.sh test

# Vérification des articulations
python -c "
import mujoco
model = mujoco.MjModel.from_xml_path('src/bbia_sim/sim/models/reachy_mini_complete.xml')
print('Articulations:', [model.joint(i).name for i in range(model.njnt)])
"
```

---

## 📁 **Structure des Fichiers**

```
bbia-reachy-sim/
├── 🤖 launch_robot.sh                    # Lien vers le lanceur principal
├── 📁 scripts/
│   ├── 🚀 launch_robot_3d.sh             # Script bash principal
│   ├── 🐍 launch_robot.py                # Script Python simple
│   ├── 🎯 launch_complete_robot.py       # Lanceur complet
│   ├── 🎭 demo_wave.sh                   # Démo salut
│   └── 🎭 demo_nod.sh                    # Démo hochement
├── 📁 src/bbia_sim/sim/
│   ├── 📁 models/
│   │   ├── 🤖 reachy_mini_complete.xml   # Modèle MJCF corrigé
│   │   └── 🤖 reachy_mini.xml            # Ancien modèle (pièces séparées)
│   └── 📁 assets/reachy_official/         # Assets STL officiels
└── 📁 examples/
    └── 📊 visualize_movements.py         # Visualiseur temps réel
```

---

## 🎯 **Prochaines Étapes**

### 🔄 **Améliorations Possibles**
1. **🎨 Textures** : Ajouter des textures réalistes
2. **💡 Éclairage** : Améliorer l'éclairage et les ombres
3. **🎭 Animations** : Créer des séquences d'animation
4. **🎮 Contrôles** : Ajouter des contrôles clavier/souris
5. **📊 Télémétrie** : Améliorer la visualisation des données

### 🚀 **Intégration**
- **API** : Contrôle temps réel via REST/WebSocket
- **Comportements** : Intégration avec le système de comportements
- **Vision** : Ajout de la caméra virtuelle
- **Audio** : Intégration audio-spatiale

---

## 💡 **Conseils d'Utilisation**

1. **🎮 Commencez simple** : Utilisez `./launch_robot.sh` pour voir le robot
2. **🔍 Explorez** : Utilisez la souris pour tourner autour du robot
3. **🎭 Testez les démos** : Lancez `demo_wave.sh` et `demo_nod.sh`
4. **📊 Visualisez** : Utilisez `visualize_movements.py` pour voir les données
5. **🌐 Contrôlez** : Lancez l'API pour contrôler le robot via HTTP

---

**🤖 BBIA Reachy Mini 3D** - *Simulation complète et réaliste* ✨

**Version** : 0.3.1  
**Date** : 23 octobre 2025  
**Status** : ✅ Fonctionnel  
**Robot** : 🤖 Complet et assemblé
