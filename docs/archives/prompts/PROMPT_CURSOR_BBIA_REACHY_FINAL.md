# ARCHIVES/HISTORIQUE (non maintenu)

> Ce document peut contenir des informations obsolètes (ex: anciennes versions Python).
> Référez-vous au `README.md` et à `.github/workflows/ci.yml` pour la version active (Python 3.11+) et les procédures à jour.

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
- **Version :** 1.1.1 → 1.2.0 (Reachy-Ready + IA Légère) - Backend unifié RobotAPI + Golden Tests
- **Branche de travail :** `develop` (toujours travailler sur develop)

### **✅ ÉTAT ACTUEL - TOUT FONCTIONNE**
1. **✅ Modèle MuJoCo officiel** : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`
2. **✅ Assets STL officiels** : 41 fichiers du dépôt Pollen Robotics
3. **✅ Simulation 3D fonctionnelle** : Robot visible et animé
4. **✅ Intégration BBIA complète** : Émotions → joints
5. **✅ Backend unifié RobotAPI** : Switch facile Sim ↔ Robot réel
6. **✅ 4 Vertical Slices** : Émotion, Voix, Vision, Comportement
7. **✅ Record & Replay** : Enregistrement et rejeu d'animations
8. **✅ CONTRACT.md gelé** : API stable v1.1.x
9. **✅ Golden tests** : 3 traces référence + validation
10. **✅ CI solide** : Seed fixé, artefacts, headless
11. **✅ Tests smoke** : 11 tests automatiques <5s
12. **✅ Tests complets** : 431 passent, 11 skippés (100% réussite)
13. **✅ Code propre** : Ruff, Black, MyPy validés

### **🎯 STRATÉGIE VALIDÉE v1.2.0**
- **Objectif** : Démo professionnelle avec robot réel (pas de features supplémentaires)
- **Approche** : Cycles courts 1-2 semaines avec critères d'arrêt mesurables
- **Cible** : Développeurs/chercheurs (API/CLI, artefacts, portfolio)
- **Focus** : Reachy-ready + IA légère (Whisper + YOLOv8n + MediaPipe)

---

## ⚠️ **CONTRAINTES NON NÉGOCIABLES**

### **🔒 Sécurité & Stabilité**
- **AUCUNE suppression destructrice** sans plan de PR
- **Respecte l'arborescence existante** (`src/`, `tests/`, `examples/`, `scripts/`)
- **Tests et linters doivent rester VERTS** (531 tests collectés, 418 passent actuellement)
- **Coverage maintenu** à 76.70% minimum

### **🛠️ Standards Techniques**
- **Python 3.10+** uniquement
- **Pas de dépendances exotiques** sans justification claire
- **Code modulaire** et évolutif
- **Documentation à jour** systématiquement

### **🌿 Workflow Git**
- **Toujours travailler sur `develop`**
- **Commits atomiques** avec messages descriptifs
- **JAMAIS de guillemets doubles** dans les messages de commit
- **TOUJOURS utiliser des guillemets simples** pour les messages avec espaces
- **PR obligatoire** pour toute modification significative

### **🧪 Tests Golden**
- **JAMAIS modifier les traces de référence** sans raison valide
- **TOUJOURS respecter les tolérances** : ±0.25 rad position, ±20% cadence
- **JAMAIS commiter de nouvelles références** sans validation
- **TOUJOURS utiliser le seed fixé** : SEED=42

### **📋 CONTRACT RobotAPI**
- **JAMAIS modifier l'API** sans créer une nouvelle version
- **TOUJOURS respecter le CONTRACT.md gelé** v1.1.x
- **JAMAIS appeler MuJoCo directement** dans les nouvelles démos
- **TOUJOURS utiliser RobotAPI** pour le backend unifié
- **Tests verts** avant merge

### **🐍 Environnement Python**
- **TOUJOURS travailler dans le venv** : `source venv/bin/activate`
- **JAMAIS utiliser l'interpréteur système** directement
- **Vérifier l'environnement** avant chaque commande Python
- **JAMAIS laisser d'erreurs** (code ou autre) - tout doit être propre

---

## 🚨 **ERREURS CRITIQUES À ÉVITER ABSOLUMENT**

### **❌ ERREURS DE COMMITS**
- **JAMAIS de guillemets doubles** dans les messages de commit
- **TOUJOURS utiliser des guillemets simples** pour les messages avec espaces
- **Exemples corrects :**
  ```bash
  git commit -m 'Message simple'
  git commit -m 'Message avec espaces'
  ```
- **Exemples INCORRECTS :**
  ```bash
  git commit -m "Message avec guillemets doubles"  # ❌ INTERDIT
  git commit -m Message sans guillemets            # ❌ INTERDIT si espaces
  ```

### **❌ ERREURS D'ENVIRONNEMENT**
- **JAMAIS utiliser l'interpréteur système** : `python` directement
- **TOUJOURS activer le venv** : `source venv/bin/activate`
- **JAMAIS laisser d'erreurs** (code ou autre) - tout doit être propre
- **Vérifier l'environnement** avant chaque commande Python
- **Exemples corrects :**
  ```bash
  source venv/bin/activate
  python examples/demo_viewer_bbia_corrected.py
  ```
- **Exemples INCORRECTS :**
  ```bash
  python examples/demo_viewer_bbia_corrected.py  # ❌ Sans venv
  /usr/bin/python3 examples/demo_viewer_bbia_corrected.py  # ❌ Système
  ```

### **❌ ERREURS DE JOINTS**
- **JAMAIS animer les antennes** : `left_antenna`, `right_antenna` (BLOQUÉES)
- **JAMAIS animer les joints passifs** : `passive_1` à `passive_7` (BLOQUÉS)
- **JAMAIS dépasser 0.3 rad** d'amplitude pour éviter les instabilités
- **TOUJOURS utiliser `yaw_body`** pour les animations visibles
- **Exemples corrects :**
  ```bash
  python examples/demo_viewer_bbia_corrected.py --joint yaw_body --amplitude 0.3
  ```
- **Exemples INCORRECTS :**
  ```bash
  python examples/demo_viewer_bbia_corrected.py --joint left_antenna  # ❌ BLOQUÉ
  python examples/demo_viewer_bbia_corrected.py --amplitude 1.0        # ❌ TROP ÉLEVÉ
  ```

### **❌ ERREURS DE QUALITÉ CODE**
- **JAMAIS commiter sans tests verts**
- **JAMAIS ignorer les erreurs de linting** (Ruff, Black, MyPy)
- **TOUJOURS corriger les erreurs** avant de continuer
- **Exemples corrects :**
  ```bash
  ruff check src/ examples/ tests/ --fix
  black src/ examples/ tests/
  mypy src/
  python -m pytest tests/ -v
  ```
- **Exemples INCORRECTS :**
  ```bash
  git commit -m 'Fix bug'  # ❌ Sans tests
  # Ignorer les erreurs ruff/black/mypy  # ❌ INTERDIT
  ```

### **❌ ERREURS DE MODIFICATION**
- **JAMAIS modifier le modèle XML officiel** sans validation
- **JAMAIS supprimer des fichiers** sans plan de PR
- **JAMAIS casser la compatibilité** avec le vrai robot Reachy
- **TOUJOURS tester les modifications** avec la démo 3D
- **Exemples corrects :**
  ```bash
  # Tester avant modification
  python examples/demo_viewer_bbia_corrected.py --headless --duration 2
  # Puis modifier
  # Puis retester
  ```
- **Exemples INCORRECTS :**
  ```bash
  # Modifier directement sans test  # ❌ INTERDIT
  # Supprimer des fichiers sans PR  # ❌ INTERDIT
  ```

### **❌ ERREURS DE VISUALISATION 3D**
- **JAMAIS utiliser --headless** si on veut voir la 3D
- **TOUJOURS utiliser mjpython** sur macOS pour la visualisation graphique
- **JAMAIS oublier de spécifier le backend** (mujoco ou reachy)
- **Exemples corrects :**
  ```bash
  # VOIR LA 3D (mode graphique)
  mjpython examples/demo_emotion_ok.py --emotion happy --duration 10 --backend mujoco

  # TESTS RAPIDES (mode headless)
  python examples/demo_emotion_ok.py --headless --emotion happy --duration 5 --backend mujoco
  ```
- **Exemples INCORRECTS :**
  ```bash
  python examples/demo_emotion_ok.py --headless --emotion happy  # ❌ Pas de 3D
  python examples/demo_emotion_ok.py --emotion happy              # ❌ Pas de backend
  ```

### **❌ ERREURS DE BACKEND UNIFIÉ**
- **JAMAIS utiliser MuJoCo directement** dans les nouvelles démos
- **TOUJOURS utiliser RobotAPI** pour le backend unifié
- **JAMAIS oublier de spécifier le backend** (mujoco ou reachy)
- **Exemples corrects :**
  ```bash
  # Backend unifié
  python examples/demo_emotion_ok.py --backend mujoco --emotion happy
  python examples/demo_emotion_ok.py --backend reachy --emotion happy
  ```
- **Exemples INCORRECTS :**
  ```bash
  # Utilisation directe MuJoCo (ancienne méthode)
  import mujoco  # ❌ Dans les nouvelles démos
  ```

### **❌ ERREURS D'ORGANISATION**
- **JAMAIS laisser des fichiers à la racine** (sauf README.md, pyproject.toml, requirements.txt)
- **TOUJOURS organiser les fichiers** dans les bons dossiers
- **Exemples corrects :**
  ```bash
  docs/audit/          # Fichiers d'audit (AUDIT_3D_BBIA_COMPLET.md)
  docs/mission/        # Fichiers de mission
  docs/opportunities/  # Fichiers d'opportunités
  docs/prompts/        # Fichiers de prompts
  docs/requirements/   # Fichiers de requirements
  scripts/             # Scripts d'analyse (analyze_joints_detailed.py)
  ```
- **Exemples INCORRECTS :**
  ```bash
  AUDIT_3D_BBIA.md     # ❌ À la racine
  MISSION_FINALE.md    # ❌ À la racine
  OPPORTUNITES.md      # ❌ À la racine
  analyze_joints.py    # ❌ À la racine
  ```

### **❌ ERREURS D'AUDIT**
- **JAMAIS ignorer l'audit complet** avant de développer
- **TOUJOURS vérifier** : 418 tests passent, démo fonctionne
- **TOUJOURS utiliser** : `yaw_body` pour les animations visibles
- **JAMAIS animer** : `left_antenna`, `right_antenna`, `passive_*`
- **Exemples corrects :**
  ```bash
  # Vérification avant développement
  python examples/demo_viewer_bbia_corrected.py --headless --duration 2 --joint yaw_body
  python -m pytest tests/ -m "not e2e" -q
  ```
- **Exemples INCORRECTS :**
  ```bash
  # Développement sans vérification  # ❌ INTERDIT
  # Animation des antennes           # ❌ INTERDIT
  # Amplitude > 0.3 rad              # ❌ INTERDIT
  ```

### **❌ ERREURS DE DOCUMENTATION**
- **JAMAIS laisser des dates incorrectes** (utiliser "Oct / Oct / Nov. 20255")
- **JAMAIS documenter des fonctionnalités non testées**
- **TOUJOURS mettre à jour la documentation** avec les modifications
- **Exemples corrects :**
  ```markdown
  *Dernière mise à jour : Oct / Oct / Nov. 20255*
  ```
- **Exemples INCORRECTS :**
  ```markdown
  *Dernière mise à jour : Oct / Oct / Nov. 20255*  # ❌ Date incorrecte
  *Dernière mise à jour : 2025*             # ❌ Date incorrecte
  ```

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

### **✅ Visualisation 3D Fonctionnelle (AUDIT COMPLET)**
```bash
# 🎯 DÉMO CORRIGÉE - Version stable et paramétrable (RECOMMANDÉE)
python examples/demo_viewer_bbia_corrected.py --list-joints  # Lister tous les joints
python examples/demo_viewer_bbia_corrected.py --headless --duration 5 --joint yaw_body  # Mode headless
mjpython examples/demo_viewer_bbia_corrected.py --duration 10 --joint yaw_body  # Mode graphique

# Démo principale (fonctionnelle)
mjpython examples/demo_robot_correct.py

# Test des joints sûrs uniquement
mjpython examples/test_safe_joints.py

# Version paramétrable
mjpython examples/demo_viewer_bbia_simple.py --joint yaw_body --duration 10 --frequency 0.5 --amplitude 0.3
```

### **📊 RÉSULTATS AUDIT COMPLET**
- **✅ Tests** : 418/418 passent (100% de réussite)
- **✅ Démo** : Animation stable en headless ET graphique
- **✅ Joints** : 16 joints analysés (1 SAFE, 6 RISKY, 9 FORBIDDEN)
- **✅ Architecture** : MuJoCoSimulator + SimulationService + BBIAIntegration
- **✅ Documentation** : Complète et organisée

### **📋 Documentation Organisée**
- **Audit 3D** : `docs/audit/AUDIT_3D_BBIA.md`
- **Audit Complet** : `docs/audit/AUDIT_3D_BBIA_COMPLET.md`
- **Résultats** : `docs/audit/RESULTATS_AUDIT_3D_BBIA.md`
- **Mission** : `docs/mission/MISSION_FINALE_ACCOMPLIE.md`
- **Opportunités** : `docs/opportunities/OPPORTUNITES_DEVELOPPEMENT.md`
- **Prompts** : `docs/prompts/PROMPT_CURSOR_BBIA_REACHY_FINAL.md`

### **✅ Tests Automatiques**
```bash
# Tests smoke RobotAPI (6 tests <5s)
python -m pytest tests/test_robot_api_smoke.py -v

# Tests vertical slices (9 tests)
python -m pytest tests/test_vertical_slices.py -v

# Tests complets (427 tests)
python -m pytest tests/ -m "not e2e" -q

# Tests avec coverage
python -m pytest tests/ --cov=src/bbia_sim --cov-report=html
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
- `examples/demo_emotion_ok.py` - **Démo Émotion → Pose (RobotAPI)**
- `examples/demo_voice_ok.py` - **Démo Voix → Action (RobotAPI)**
- `examples/demo_vision_ok.py` - **Démo Vision → Suivi (RobotAPI)**
- `examples/demo_behavior_ok.py` - **Démo Comportement → Scénario (RobotAPI)**
- `examples/demo_robot_correct.py` - Démo principale (ancienne)
- `examples/test_all_joints.py` - Test tous joints
- `examples/demo_viewer_bbia_simple.py` - Version paramétrable
- `examples/test_robot_3d.py` - Test rapide

### **✅ Tests Complets**
- `tests/test_robot_api_smoke.py` - **6 tests smoke RobotAPI**
- `tests/test_vertical_slices.py` - **9 tests vertical slices**
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

#### **❌ ERREUR #0 : Guillemets et Environnement**
```bash
# ❌ NE JAMAIS utiliser de guillemets doubles dans les messages de commit
git commit -m "Message avec guillemets doubles"  # ÉCHEC GARANTI

# ✅ CORRECT - Utiliser des guillemets simples
git commit -m 'Message avec guillemets simples'  # SUCCÈS GARANTI

# ❌ NE JAMAIS travailler en dehors du venv
python script.py  # Peut causer des erreurs de dépendances

# ✅ CORRECT - Toujours dans le venv
source venv/bin/activate  # OU utiliser mjpython directement
mjpython script.py  # SUCCÈS GARANTI
```

**Règles absolues :**
- **JAMAIS de guillemets** dans les messages de commit
- **TOUJOURS travailler dans le venv** ou utiliser `mjpython`
- **JAMAIS laisser d'erreurs** de code, linting, ou autres
- **TOUJOURS vérifier** Ruff, Black, MyPy avant commit

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
mjpython examples/demo_robot_correct.py                    # Démo principale (RECOMMANDÉE)
mjpython examples/test_safe_joints.py                     # Test joints sûrs uniquement
mjpython examples/demo_viewer_bbia_simple.py --joint yaw_body --duration 10 --frequency 0.5 --amplitude 0.3

# ⚠️ COMMANDES AVEC PRÉCAUTION
mjpython examples/test_all_joints.py                       # Test tous joints (sécurisé)
```

### **🎯 JOINTS MOBILES VALIDÉS**

#### **✅ Joints qui PEUVENT bouger (7 joints)**
```python
MOBILE_JOINTS = {
    "yaw_body": "Rotation du corps - LE PLUS VISIBLE ET SÛR",
    "stewart_1": "Plateforme Stewart 1 - PROBLÉMATIQUE",
    "stewart_2": "Plateforme Stewart 2 - PROBLÉMATIQUE",
    "stewart_3": "Plateforme Stewart 3 - PROBLÉMATIQUE",
    "stewart_4": "Plateforme Stewart 4 - PROBLÉMATIQUE",
    "stewart_5": "Plateforme Stewart 5 - PROBLÉMATIQUE",
    "stewart_6": "Plateforme Stewart 6 - PROBLÉMATIQUE"
}
```

#### **⚠️ DIAGNOSTIC CRITIQUE DES JOINTS**
```bash
# Script de diagnostic obligatoire
python scripts/diagnose_joints.py
```

**Résultat du diagnostic :**
- ✅ **1 joint sûr** : `yaw_body` (rotation du corps) - **LE PLUS SÛR**
- ⚠️ **6 joints problématiques** : `stewart_1-6` (plages importantes, peuvent causer des problèmes)
- ❌ **9 joints bloqués** : `passive_1-7`, `left_antenna`, `right_antenna`

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

### **🚀 WORKFLOW OBLIGATOIRE**

#### **📋 Étapes Obligatoires AVANT TOUT COMMIT**
1. **Activer le venv** : `source venv/bin/activate` OU utiliser `mjpython`
2. **Vérifier Ruff** : `ruff check . --exclude venv --fix`
3. **Vérifier Black** : `black src/ tests/ examples/ scripts/ --check`
4. **Vérifier MyPy** : `mypy src/ --ignore-missing-imports`
5. **Tester** : `python -m pytest tests/test_adapter_mujoco.py -v`
6. **Commit avec guillemets simples** : `git commit -m 'Message simple'`
7. **Push** : `git push origin develop`

#### **🔍 Checklist de Qualité OBLIGATOIRE**
```bash
# 1. Environnement
source venv/bin/activate  # OU utiliser mjpython directement

# 2. Linting et Formatage
ruff check . --exclude venv --fix
black src/ tests/ examples/ scripts/

# 3. Tests
python -m pytest tests/test_adapter_mujoco.py -v

# 4. Commit (SANS guillemets)
git add .
git commit -m 'Message simple avec guillemets simples'
git push origin develop
```

#### **⚠️ RÈGLES ABSOLUES**
- **JAMAIS de guillemets doubles** dans les messages de commit
- **TOUJOURS dans le venv** ou utiliser `mjpython`
- **JAMAIS d'erreurs** de linting, formatage, ou tests
- **TOUJOURS vérifier** la qualité avant commit
- **TOUJOURS utiliser** les scripts de diagnostic

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
- **✅ Tests complets** (531 tests collectés, 418 passent)
- **✅ Code propre** (Ruff, Black, MyPy validés)
- **✅ Documentation** complète et à jour

### **🚀 Prêt pour :**
- Utilisation immédiate en simulation
- Transition fluide vers robot physique
- Développement continu avec confiance
- Intégration BBIA en production

---

## 🎯 **OPPORTUNITÉS DE DÉVELOPPEMENT**

### **🚀 Fonctionnalités à Développer**
1. **Nouvelles émotions** : Confusion, détermination, nostalgie, fierté
2. **Comportements avancés** : Interactions sociales, apprentissage
3. **Vision améliorée** : Reconnaissance d'expressions, suivi multi-objets
4. **Audio avancé** : Reconnaissance de commandes, synthèse émotionnelle
5. **API étendue** : Endpoints pour contrôle fin, monitoring
6. **Intégration Unity** : Synchronisation temps réel avec Unity
7. **Tests automatisés** : CI/CD, tests de régression
8. **Documentation interactive** : Tutoriels, guides vidéo

### **🔧 Améliorations Techniques**
1. **Performance** : Optimisation MuJoCo, parallélisation
2. **Sécurité** : Validation des entrées, gestion d'erreurs
3. **Monitoring** : Métriques temps réel, alertes
4. **Configuration** : Fichiers de config dynamiques
5. **Logging** : Système de logs structuré
6. **Docker** : Containerisation complète
7. **Tests** : Couverture 90%+, tests d'intégration

### **🎮 Exemples Concrets**
```python
# Nouvelle émotion "confusion"
confusion_emotion = {
    "yaw_body": 0.1,      # Rotation légère
    "stewart_1": 0.05,    # Mouvement subtil
    "duration": 3.0,      # Durée de l'émotion
    "transition": "slow"  # Transition lente
}

# Reconnaissance d'expressions humaines
face_emotions = await vision.detect_human_emotions()
if face_emotions["happy"] > 0.8:
    await integration.apply_emotion("happy", intensity=0.9)

# Commandes vocales
command = await voice.recognize_command()
if "tourne" in command and "gauche" in command:
    await integration.turn_head("left", speed=0.5)
```

---

**🤖 BBIA Reachy Mini Simulation - Mission Accomplie + Opportunités Identifiées ! ✨**

*Prompt final - Oct / Oct / Nov. 20255 - Projet fonctionnel, aligné et prêt pour le développement avancé*
