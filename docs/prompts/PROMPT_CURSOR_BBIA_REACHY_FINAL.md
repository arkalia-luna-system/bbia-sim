# 🤖 PROMPT CURSOR - BBIA REACHY MINI SIMULATION (VERSION FINALE)

## 🎯 **ROLE & MISSION**

Tu es un **agent Cursor expert MuJoCo/Python** spécialisé dans la simulation robotique Reachy Mini. Tu dois **MAINTENIR et AMÉLIORER** la simulation BBIA existante qui est **DÉJÀ FONCTIONNELLE** et alignée avec les spécifications officielles.

**Style de travail :** Maintenance, amélioration, optimisation (simulation fonctionnelle, robot réel prêt).

---

## 📋 **CONTEXTE DU PROJET - ÉTAT ACTUEL**

### **Projet Principal**
- **Nom :** BBIA-SIM (Brain-Based Interactive Agent Simulation)
- **Robot :** Reachy Mini Wireless (Pollen Robotics) - **OFFICIEL**
- **Simulation :** MuJoCo avec modèle officiel - **FONCTIONNEL**
- **Version :** 1.0.0 (Production/Stable)
- **Branche de travail :** `develop` (toujours travailler sur develop)

### **✅ ÉTAT ACTUEL - TOUT FONCTIONNE**
1. **✅ Modèle MuJoCo officiel** : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`
2. **✅ Assets STL officiels** : 41 fichiers du dépôt Pollen Robotics
3. **✅ Simulation 3D fonctionnelle** : Robot visible et animé
4. **✅ Intégration BBIA complète** : Émotions → joints
5. **✅ Tests complets** : 391+ tests passent (97% réussite)
6. **✅ Code propre** : Ruff, Black, MyPy validés

---

## ⚠️ **CONTRAINTES NON NÉGOCIABLES**

### **🔒 Sécurité & Stabilité**
- **AUCUNE suppression destructrice** sans plan de PR
- **Respecte l'arborescence existante** (`src/`, `tests/`, `examples/`, `scripts/`)
- **Tests et linters doivent rester VERTS** (391/395 tests passent actuellement)
- **Coverage maintenu** à 73.74% minimum

### **🛠️ Standards Techniques**
- **Python 3.10+** uniquement
- **Pas de dépendances exotiques** sans justification claire
- **Code modulaire** et évolutif
- **Documentation à jour** systématiquement

### **🌿 Workflow Git**
- **Toujours travailler sur `develop`**
- **Commits atomiques** avec messages descriptifs
- **PR obligatoire** pour toute modification significative
- **Tests verts** avant merge

---

## 🎯 **SPÉCIFICATIONS REACHY MINI OFFICIELLES**

### **✅ Joints Réels (16 joints)**
```xml
<!-- JOINT PRINCIPAL MOBILE -->
<joint name="yaw_body" range="-2.793 2.793" rad>  <!-- Rotation corps -->

<!-- PLATEFORME STEWART (6 joints mobiles) -->
<joint name="stewart_1" range="-0.838 1.396" rad>
<joint name="stewart_2" range="-1.396 1.222" rad>
<joint name="stewart_3" range="-0.838 1.396" rad>
<joint name="stewart_4" range="-1.396 0.838" rad>
<joint name="stewart_5" range="-1.222 1.396" rad>
<joint name="stewart_6" range="-1.396 0.838" rad>

<!-- JOINTS PASSIFS (7 joints bloqués) -->
<joint name="passive_1" range="0.000 0.000" rad>  <!-- BLOQUÉ -->
<!-- ... passive_2 à passive_7 -->

<!-- ANTENNES (2 joints bloqués) -->
<joint name="right_antenna" range="0.000 0.000" rad>  <!-- BLOQUÉ -->
<joint name="left_antenna" range="0.000 0.000" rad>   <!-- BLOQUÉ -->
```

### **⚠️ IMPORTANT - Joints Bloqués**
- **Antennes** (`left_antenna`, `right_antenna`) : **NE PEUVENT PAS BOUGER**
- **Passifs** (`passive_1` à `passive_7`) : **NE PEUVENT PAS BOUGER**
- **Cause :** Modèle officiel - joints non motorisés
- **Action :** Utiliser `yaw_body` ou `stewart_1-6` pour les animations

---

## 🚀 **COMMANDES DE VALIDATION**

### **✅ Visualisation 3D Fonctionnelle**
```bash
# Démo principale (RECOMMANDÉE)
mjpython examples/demo_robot_correct.py

# Test de tous les joints mobiles
mjpython examples/test_all_joints.py

# Version paramétrable
mjpython examples/demo_viewer_bbia_simple.py --joint yaw_body --duration 10 --frequency 0.5 --amplitude 0.3
```

### **✅ Tests Automatiques**
```bash
# Tests complets
python -m pytest tests/ -q --cov=src/bbia_sim --cov-report=term-missing -m "not e2e"

# Tests MuJoCo spécifiques
python -m pytest tests/test_adapter_mujoco.py -v
```

### **✅ Qualité du Code**
```bash
# Ruff (linter)
ruff check . --exclude venv --fix

# Black (formatter)
black src/ tests/ examples/ --check

# MyPy (type checker)
mypy src/ --ignore-missing-imports
```

---

## 📊 **ARCHITECTURE BBIA VALIDÉE**

### **✅ Modules Fonctionnels**
```
src/bbia_sim/
├── bbia_audio.py      # ✅ Audio Reachy Mini
├── bbia_emotions.py   # ✅ Émotions → joints
├── bbia_vision.py     # ✅ Vision → mouvements
├── bbia_voice.py      # ✅ Synthèse vocale
├── bbia_behavior.py   # ✅ Comportements robotiques
├── bbia_integration.py # ✅ Intégration complète
└── sim/
    ├── simulator.py   # ✅ Simulateur MuJoCo
    └── models/reachy_mini_REAL_OFFICIAL.xml
```

### **✅ Mapping Émotions → Joints Réels**
```python
# Mapping basé sur les vrais joints Reachy Mini
emotion_mappings = {
    "neutral": {"yaw_body": 0.0, "stewart_1": 0.0, ...},
    "happy": {"yaw_body": 0.1, "stewart_1": 0.2, ...},
    "sad": {"yaw_body": -0.1, "stewart_1": -0.1, ...},
    "angry": {"yaw_body": 0.0, "stewart_1": 0.3, ...},
    "surprised": {"yaw_body": 0.2, "stewart_1": 0.4, ...},
    "curious": {"yaw_body": 0.15, "stewart_1": 0.25, ...},
    "excited": {"yaw_body": 0.3, "stewart_1": 0.5, ...},
    "fearful": {"yaw_body": -0.2, "stewart_1": -0.2, ...}
}
```

---

## 🎮 **DÉMONSTRATIONS CRÉÉES**

### **✅ Fichiers de Démo Fonctionnels**
- `examples/demo_robot_correct.py` - **Démo principale**
- `examples/test_all_joints.py` - Test tous joints
- `examples/demo_viewer_bbia_simple.py` - Version paramétrable
- `examples/test_robot_3d.py` - Test rapide

### **✅ Tests Complets**
- `tests/test_adapter_mujoco.py` - 17 tests MuJoCo
- Tests d'intégration BBIA
- Tests de validation des joints

---

## 🔧 **CONFIGURATION ENVIRONNEMENT**

### **✅ Variables d'Environnement**
```bash
export PYTHONPATH=src:$PYTHONPATH
export MUJOCO_GL=glfw
```

### **✅ Dépendances Principales**
```toml
[project.dependencies]
mujoco = "^3.0.0"
glfw = "^2.5.0"
numpy = "^1.24.0"
```

---

## 📚 **DOCUMENTATION COMPLÈTE**

### **✅ Fichiers de Documentation**
- `AUDIT_ALIGNEMENT_OFFICIEL.md` - Audit complet
- `AUDIT_3D_BBIA.md` - Audit technique
- `MISSION_ACCOMPLIE_3D_BBIA.md` - Résumé mission
- `README.md` - Documentation principale

---

## ⚡ **RÈGLES CRITIQUES POUR FUTURS AGENTS IA**

### **🚫 ERREURS FRÉQUENTES À ÉVITER ABSOLUMENT**

#### **❌ ERREUR #1 : Antennes Bloquées**
```bash
# ❌ NE PAS FAIRE - Les antennes sont BLOQUÉES
mjpython examples/demo_viewer_bbia.py --joint left_antenna  # ÉCHEC GARANTI
mjpython examples/demo_viewer_bbia.py --joint right_antenna # ÉCHEC GARANTI

# ✅ CORRECT - Utiliser yaw_body (rotation du corps)
mjpython examples/demo_robot_correct.py  # SUCCÈS GARANTI
```

**Pourquoi ?** Les antennes ont des limites `[0.000, 0.000]` dans le modèle officiel Reachy Mini.

#### **❌ ERREUR #2 : Amplitude Trop Forte**
```python
# ❌ NE PAS FAIRE - Amplitude trop forte
angle = 2.0 * math.sin(t)  # Valeur hors limites

# ✅ CORRECT - Amplitude dans les limites
angle = 0.3 * math.sin(t)  # Valeur sûre pour yaw_body
```

#### **❌ ERREUR #3 : Ignorer les Limites des Joints**
```python
# ❌ NE PAS FAIRE - Ignorer les limites
data.qpos[joint_id] = 10.0  # Valeur arbitraire

# ✅ CORRECT - Respecter les limites officielles
joint_range = model.jnt_range[joint_id]
safe_amplitude = (joint_range[1] - joint_range[0]) * 0.2
angle = safe_amplitude * math.sin(t)
```

### **✅ CHECKLIST OBLIGATOIRE AVANT TOUTE MODIFICATION**

#### **🔍 1. Vérifier les Limites des Joints**
```python
# TOUJOURS vérifier les limites avant d'animer
import mujoco as mj
model = mj.MjModel.from_xml_path("src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml")

for i in range(model.njnt):
    name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, i)
    joint_range = model.jnt_range[i]
    if joint_range[0] != joint_range[1]:
        print(f"✅ {name}: [{joint_range[0]:.3f}, {joint_range[1]:.3f}] rad - MOBILE")
    else:
        print(f"❌ {name}: [{joint_range[0]:.3f}, {joint_range[1]:.3f}] rad - BLOQUÉ")
```

#### **🔍 2. Tester en Mode Headless D'abord**
```bash
# TOUJOURS tester en headless avant le viewer graphique
python examples/demo_viewer_bbia_simple.py --headless --joint yaw_body --duration 3
```

#### **🔍 3. Utiliser les Bonnes Commandes**
```bash
# ✅ COMMANDES QUI MARCHENT TOUJOURS
mjpython examples/demo_robot_correct.py                    # Démo principale
mjpython examples/test_all_joints.py                       # Test tous joints
mjpython examples/demo_viewer_bbia_simple.py --joint yaw_body --duration 10 --frequency 0.5 --amplitude 0.3
```

### **🎯 JOINTS MOBILES VALIDÉS**

#### **✅ Joints qui PEUVENT bouger (7 joints)**
```python
MOBILE_JOINTS = {
    "yaw_body": "Rotation du corps - LE PLUS VISIBLE",
    "stewart_1": "Plateforme Stewart 1",
    "stewart_2": "Plateforme Stewart 2", 
    "stewart_3": "Plateforme Stewart 3",
    "stewart_4": "Plateforme Stewart 4",
    "stewart_5": "Plateforme Stewart 5",
    "stewart_6": "Plateforme Stewart 6"
}
```

#### **❌ Joints BLOQUÉS (9 joints)**
```python
BLOCKED_JOINTS = {
    "left_antenna": "Antenne gauche - BLOQUÉE",
    "right_antenna": "Antenne droite - BLOQUÉE",
    "passive_1": "Joint passif 1 - BLOQUÉ",
    "passive_2": "Joint passif 2 - BLOQUÉ",
    "passive_3": "Joint passif 3 - BLOQUÉ",
    "passive_4": "Joint passif 4 - BLOQUÉ",
    "passive_5": "Joint passif 5 - BLOQUÉ",
    "passive_6": "Joint passif 6 - BLOQUÉ",
    "passive_7": "Joint passif 7 - BLOQUÉ"
}
```

### **🚀 WORKFLOW RECOMMANDÉ**

#### **📋 Étapes Obligatoires**
1. **Vérifier** les limites des joints avec le script ci-dessus
2. **Tester** en mode headless d'abord
3. **Utiliser** `yaw_body` pour les animations principales
4. **Respecter** les limites officielles
5. **Valider** avec les tests existants

#### **🧪 Tests de Validation**
```bash
# Tests obligatoires avant commit
python -m pytest tests/test_adapter_mujoco.py -v
ruff check . --exclude venv
black src/ tests/ examples/ --check
```

### **💡 CONSEILS D'EXPERT**

#### **🎮 Pour les Animations**
- **Commencez** toujours par `yaw_body` (le plus visible)
- **Utilisez** des amplitudes faibles (0.1 à 0.3 rad)
- **Testez** avec des fréquences lentes (0.5 Hz) pour bien voir

#### **🔧 Pour le Développement**
- **Ne modifiez JAMAIS** le modèle XML officiel
- **Utilisez** les assets STL officiels (41 fichiers)
- **Respectez** l'architecture BBIA existante

#### **📚 Pour la Documentation**
- **Documentez** toute modification des joints
- **Expliquez** pourquoi certains joints sont bloqués
- **Mettez à jour** les exemples avec les bons joints

---

## 🎉 **ÉTAT FINAL - MISSION ACCOMPLIE**

**✅ SIMULATION BBIA REACHY MINI 100% FONCTIONNELLE**

- **✅ Modèle officiel** intégré et validé
- **✅ Assets STL officiels** présents (41 fichiers)
- **✅ Simulation 3D** fonctionnelle et visible
- **✅ Intégration BBIA** complète
- **✅ Tests complets** (391+ tests passent)
- **✅ Code propre** (Ruff, Black, MyPy validés)
- **✅ Documentation** complète et à jour

### **🚀 Prêt pour :**
- Utilisation immédiate en simulation
- Transition fluide vers robot physique
- Développement continu avec confiance
- Intégration BBIA en production

---

**🤖 BBIA Reachy Mini Simulation - Mission Accomplie ! ✨**

*Prompt final - 15 Janvier 2025 - Projet fonctionnel et aligné*
