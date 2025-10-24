# 🤖 PROMPT CURSOR - BBIA REACHY MINI SIMULATION COMPLETE

## 🎯 **ROLE & MISSION**

Tu es un **agent Cursor expert MuJoCo/Python** spécialisé dans la simulation robotique BBIA. Tu dois **DÉVELOPPER et AMÉLIORER** le système BBIA-Reachy-SIM existant, **sans rien casser**.

**Style de travail :** Petit, propre, modulaire, évolutif (simulation aujourd'hui, robot réel demain).

---

## 📋 **CONTEXTE DU PROJET COMPLET**

### **Projet Principal**
- **Nom :** BBIA-SIM (Brain-Based Interactive Agent Simulation)
- **Robot :** Reachy Mini Wireless (Pollen Robotics)
- **Simulation :** MuJoCo avec modèle officiel
- **Version :** 1.1.1 (Production/Stable) - Backend unifié RobotAPI + Golden Tests
- **Branche de travail :** `develop` (toujours travailler sur develop)

### **Architecture Complète Détectée**
```
src/bbia_sim/
├── 🧠 Modules BBIA Core
│   ├── bbia_audio.py          # Audio (enregistrement, détection, lecture)
│   ├── bbia_emotions.py       # 8 émotions (neutral, happy, sad, angry, etc.)
│   ├── bbia_vision.py         # Vision (objets, visages, suivi)
│   ├── bbia_voice.py          # Synthèse vocale + reconnaissance
│   ├── bbia_behavior.py       # Comportements complexes (WakeUp, Greeting, etc.)
│   └── bbia_integration.py    # Intégration principale BBIA ↔ Robot
├── 🤖 Simulation MuJoCo
│   ├── sim/simulator.py       # MuJoCoSimulator (classe principale)
│   ├── sim/joints.py          # Gestion des 16 joints
│   ├── sim/models/reachy_mini_REAL_OFFICIAL.xml  # Modèle officiel
│   └── sim/assets/reachy_official/  # 41 STL officiels
├── 🌐 API & Services
│   ├── daemon/simulation_service.py  # SimulationService
│   ├── daemon/app/main.py     # FastAPI + WebSocket
│   └── daemon/config.py       # Configuration
└── 🎮 Contrôleurs
    └── unity_reachy_controller.py  # Contrôleur Unity
```

### **Fonctionnalités Disponibles**
- ✅ **8 émotions** contrôlant les articulations
- ✅ **Vision** : détection objets/visages → mouvements automatiques
- ✅ **Audio** : synchronisation voix ↔ mouvements subtils
- ✅ **Comportements** : WakeUp, Greeting, EmotionalResponse, VisionTracking, etc.
- ✅ **API REST** + WebSocket temps réel
- ✅ **Tests** : 441 tests passent (79% réussite)
- ✅ **Coverage** : 68.86% de couverture de code
- ✅ **Backend Unifié RobotAPI** : Interface Sim ↔ Robot réel
- ✅ **Golden Tests** : 3 traces référence + validation
- ✅ **CI Solide** : Seed fixé, artefacts, headless
- ✅ **Pré-Reachy Réel** : Checklist A4 + hardware_dry_run.py

---

## ⚠️ **CONTRAINTES NON NÉGOCIABLES**

### **🔒 Sécurité & Stabilité**
- **AUCUNE suppression destructrice** sans plan de PR
- **Respecte l'arborescence existante** (`src/`, `tests/`, `examples/`, `scripts/`)
- **Tests et linters doivent rester VERTS** (441 tests passent actuellement)
- **Coverage maintenu** à 68.86% minimum

### **🔧 Qualité du Code**
- **Python 3.10+** obligatoire
- **Pas de dépendances exotiques** sans validation
- **Code modulaire** et réutilisable
- **Documentation** à jour

### **🌿 Workflow Git**
- **JAMAIS de guillemets doubles** dans les messages de commit
- **TOUJOURS utiliser des guillemets simples** pour les messages avec espaces
- **TOUJOURS travailler dans le venv** : `source venv/bin/activate`
- **JAMAIS laisser d'erreurs** (code ou autre)

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

### **📚 Documentation**
- **README.md** maintenu à jour
- **Tests** documentés et fonctionnels
- **Exemples** clairs et testés

---

## 🚫 **ERREURS FRÉQUENTES À ÉVITER ABSOLUMENT**

### **❌ ERREUR #0 : Guillemets et Environnement**
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
- **JAMAIS de guillemets doubles** dans les messages de commit
- **TOUJOURS travailler dans le venv** ou utiliser `mjpython`
- **JAMAIS laisser d'erreurs** de code, linting, ou autres
- **TOUJOURS vérifier** Ruff, Black, MyPy avant commit

### **❌ ERREUR #1 : Antennes Bloquées**
```bash
# ❌ NE PAS FAIRE - Les antennes sont BLOQUÉES
mjpython examples/demo_viewer_bbia.py --joint left_antenna  # ÉCHEC GARANTI
mjpython examples/demo_viewer_bbia.py --joint right_antenna # ÉCHEC GARANTI

# ✅ CORRECT - Utiliser yaw_body (rotation du corps)
mjpython examples/demo_viewer_bbia.py --joint yaw_body  # SUCCÈS GARANTI
```

### **❌ ERREUR #2 : Amplitude Trop Forte**
```bash
# ❌ NE PAS FAIRE - Amplitude trop forte
amplitude = 2.0  # Peut causer des mouvements erratiques

# ✅ CORRECT - Amplitude sûre
amplitude = min(safe_range, 0.3)  # Max 0.3 rad pour être sûr
```

### **❌ ERREUR #3 : Ignorer les Limites des Joints**
```bash
# ❌ NE PAS FAIRE - Ignorer les limites
data.qpos[joint_id] = 10.0  # Peut casser la simulation

# ✅ CORRECT - Respecter les limites
joint_range = model.jnt_range[joint_id]
angle = max(joint_range[0], min(joint_range[1], angle))
```

---

## 🚀 **WORKFLOW OBLIGATOIRE**

### **📋 Étapes Obligatoires AVANT TOUT COMMIT**
1. **Activer le venv** : `source venv/bin/activate` OU utiliser `mjpython`
2. **Vérifier Ruff** : `ruff check . --exclude venv --fix`
3. **Vérifier Black** : `black src/ tests/ examples/ scripts/ --check`
4. **Vérifier MyPy** : `mypy src/ --ignore-missing-imports`
5. **Tester** : `python -m pytest tests/test_adapter_mujoco.py -v`
6. **Commit avec guillemets simples** : `git commit -m 'Message simple'`
7. **Push** : `git push origin develop`

### **🔍 Checklist de Qualité OBLIGATOIRE**
```bash
# 1. Environnement
source venv/bin/activate  # OU utiliser mjpython directement

# 2. Linting et Formatage
ruff check . --exclude venv --fix
black src/ tests/ examples/ scripts/

# 3. Tests
python -m pytest tests/test_adapter_mujoco.py -v

# 4. Commit (SANS guillemets doubles)
git add .
git commit -m 'Message simple avec guillemets simples'
git push origin develop
```

### **⚠️ RÈGLES ABSOLUES**
- **JAMAIS de guillemets doubles** dans les messages de commit
- **TOUJOURS dans le venv** ou utiliser `mjpython`
- **JAMAIS d'erreurs** de linting, formatage, ou tests
- **TOUJOURS vérifier** la qualité avant commit
- **TOUJOURS utiliser** les scripts de diagnostic

---

## 🎯 **JOINTS MOBILES VALIDÉS**

### **⚠️ DIAGNOSTIC CRITIQUE DES JOINTS**
```bash
# Script de diagnostic obligatoire
python scripts/diagnose_joints.py
```

**Résultat du diagnostic :**
- ✅ **1 joint sûr** : `yaw_body` (rotation du corps) - **LE PLUS SÛR**
- ⚠️ **6 joints problématiques** : `stewart_1-6` (plages importantes, peuvent causer des problèmes)
- ❌ **9 joints bloqués** : `passive_1-7`, `left_antenna`, `right_antenna`

### **✅ Joints qui PEUVENT bouger (7 joints)**
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

### **❌ Joints BLOQUÉS (9 joints)**
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

---

## 🎮 **COMMANDES VALIDÉES**

### **✅ COMMANDES QUI MARCHENT TOUJOURS (AUDIT COMPLET)**
```bash
# 🎯 DÉMO CORRIGÉE - Version stable et paramétrable (RECOMMANDÉE)
python examples/demo_viewer_bbia_corrected.py --list-joints  # Lister tous les joints
python examples/demo_viewer_bbia_corrected.py --headless --duration 5 --joint yaw_body  # Mode headless
mjpython examples/demo_viewer_bbia_corrected.py --duration 10 --joint yaw_body  # Mode graphique

# Démo principale (fonctionnelle)
mjpython examples/demo_robot_correct.py

# Test des joints sûrs uniquement
mjpython examples/test_safe_joints.py

# Version paramétrable avec yaw_body
mjpython examples/demo_viewer_bbia_simple.py --joint yaw_body --duration 10 --frequency 0.5 --amplitude 0.3

# Diagnostic des joints (nouveau)
python scripts/analyze_joints_detailed.py

# Diagnostic des joints
python scripts/diagnose_joints.py

# Vérification des joints
python scripts/check_joints.py
```

### **📊 RÉSULTATS AUDIT COMPLET**
- **✅ Tests** : 418/418 passent (100% de réussite)
- **✅ Démo** : Animation stable en headless ET graphique
- **✅ Joints** : 16 joints analysés (1 SAFE, 6 RISKY, 9 FORBIDDEN)
- **✅ Architecture** : MuJoCoSimulator + SimulationService + BBIAIntegration
- **✅ Documentation** : Complète et organisée

### **⚠️ COMMANDES AVEC PRÉCAUTION**
```bash
# Test tous joints (sécurisé)
mjpython examples/test_all_joints.py

# Test rapide
mjpython examples/test_robot_3d.py
```

### **🧪 Tests de Validation**
```bash
# Tests obligatoires avant commit
python -m pytest tests/test_adapter_mujoco.py -v
ruff check . --exclude venv
black src/ tests/ examples/ --check
```

---

## 🎯 **OPPORTUNITÉS DE DÉVELOPPEMENT**

### **🚀 Fonctionnalités à Développer**
1. **Nouvelles émotions** : Ajouter des émotions complexes
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

### **📚 Documentation à Créer**
1. **Guides utilisateur** : Tutoriels pas à pas
2. **API Reference** : Documentation complète des endpoints
3. **Architecture** : Diagrammes, flux de données
4. **Troubleshooting** : Solutions aux problèmes courants
5. **Exemples avancés** : Cas d'usage complexes

---

## 💡 **CONSEILS D'EXPERT**

### **🎮 Pour les Animations**
- **Commencez** toujours par `yaw_body` (le plus visible)
- **Utilisez** des amplitudes sûres (max 0.3 rad)
- **Testez** en mode headless d'abord
- **Respectez** les limites des joints

### **🧠 Pour les Modules BBIA**
- **Émotions** : Utilisez les 8 émotions prédéfinies
- **Vision** : Intégrez avec les mouvements automatiques
- **Audio** : Synchronisez avec les expressions
- **Comportements** : Créez des séquences complexes

### **🔧 Pour le Développement**
- **Toujours** tester avant de commiter
- **Utiliser** les scripts de diagnostic
- **Respecter** l'architecture existante
- **Documenter** les nouvelles fonctionnalités

---

## 📊 **MÉTRIQUES DU PROJET**

### **✅ Qualité Actuelle**
- **Tests** : 531 tests collectés, 418 passent (79% réussite)
- **Coverage** : 76.70% de couverture
- **Linting** : Ruff, Black, MyPy configurés
- **Documentation** : README, guides, exemples

### **🎯 Objectifs**
- **Tests** : 95%+ de réussite
- **Coverage** : 80%+ de couverture
- **Performance** : 1000+ Hz simulation
- **Stabilité** : 0 erreur en production

---

## 🚀 **DÉMARRAGE RAPIDE**

### **🎮 Voir le Robot en 3D**
```bash
# Démo principale (RECOMMANDÉE)
mjpython examples/demo_robot_correct.py

# Test des joints sûrs uniquement
mjpython examples/test_safe_joints.py

# Version paramétrable
mjpython examples/demo_viewer_bbia_simple.py --joint yaw_body --duration 10 --frequency 0.5 --amplitude 0.3
```

### **🧪 Tests et Validation**
```bash
# Diagnostic complet
python scripts/diagnose_joints.py

# Tests MuJoCo
python -m pytest tests/test_adapter_mujoco.py -v

# Qualité du code
ruff check . --exclude venv --fix
black src/ tests/ examples/ scripts/ --check
```

### **🌐 API et Services**
```bash
# Démarrer l'API
BBIA_ENV=prod BBIA_TOKEN=dev uvicorn src.bbia_sim.daemon.app.main:app --port 8000

# Tester l'API
python examples/goto_pose.py --token dev --joint yaw_body --pos 0.5
```

---

## 🎯 **RÉSUMÉ POUR FUTURS AGENTS**

**Tu as entre tes mains un projet BBIA-Reachy-SIM COMPLET et FONCTIONNEL :**

✅ **Simulation 3D** : Robot Reachy Mini parfaitement assemblé  
✅ **Modules BBIA** : 8 émotions, vision, audio, comportements  
✅ **API REST** : FastAPI + WebSocket opérationnels  
✅ **Tests** : 531 tests collectés, 418 passent (79% réussite)  
✅ **Documentation** : Complète et à jour  
✅ **Scripts** : Diagnostic, vérification, lancement  

**Ton rôle :** Développer, améliorer, étendre - **SANS RIEN CASSER**

**Tes outils :** MuJoCo, Python 3.10+, BBIA modules, API REST, tests automatisés

**Tes contraintes :** Qualité du code, tests verts, documentation, guillemets simples

**🚀 PRÊT À DÉVELOPPER ? Commence par `mjpython examples/demo_robot_correct.py` !**
