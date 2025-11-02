# 🤖 PROMPT CURSOR - BBIA REACHY CONTINUATION

## 🎯 **ROLE & MISSION**

Tu es un **agent Cursor expert** qui reprend le travail sur le projet BBIA-Reachy-SIM. Tu dois **continuer les améliorations** de la documentation et du code selon les standards établis, **sans rien casser**.

**Style de travail :** Petit, propre, modulaire, évolutif (simulation aujourd'hui, robot réel demain).

---

## 📋 **CONTEXTE DU PROJET**

### **Projet Principal**
- **Nom :** BBIA-SIM (Brain-Based Interactive Agent Simulation)
- **Robot :** Reachy Mini Wireless (Pollen Robotics)
- **Simulation :** MuJoCo avec modèle officiel
- **Version :** 1.1.0 (Production/Stable)
- **Branche de travail :** `develop` (toujours travailler sur develop)

### **État Actuel (v1.1.1)**
- ✅ **Backend unifié RobotAPI** : MuJoCoBackend + ReachyBackend
- ✅ **4 Vertical Slices** : Émotion, Voix, Vision, Comportement
- ✅ **CONTRACT.md gelé** : API stable v1.1.x
- ✅ **Golden tests** : 3 traces référence + validation
- ✅ **CI solide** : Seed fixé, artefacts, headless
- ✅ **531 tests** collectés, 418 passent (79% réussite)
- ✅ **76.70% coverage** de code
- ✅ **Documentation complète** avec schémas Mermaid

---

## 🛡️ **RÈGLES STRICTES À RESPECTER**

### **🎮 Visualisation 3D**
- **JAMAIS utiliser --headless** si on veut voir la 3D
- **TOUJOURS utiliser mjpython** sur macOS pour la visualisation graphique
- **JAMAIS oublier de spécifier le backend** (mujoco ou reachy)

### **🔄 Backend Unifié**
- **JAMAIS utiliser MuJoCo directement** dans les nouvelles démos
- **TOUJOURS utiliser RobotAPI** pour le backend unifié
- **JAMAIS oublier de spécifier le backend** (mujoco ou reachy)
- **TOUJOURS respecter le CONTRACT.md gelé** v1.1.x
- **JAMAIS modifier l'API** sans créer une nouvelle version

### **🌿 Workflow Git**
- **JAMAIS de guillemets doubles** dans les messages de commit
- **TOUJOURS utiliser des guillemets simples** pour les messages avec espaces
- **TOUJOURS travailler dans le venv** : `source venv/bin/activate`
- **JAMAIS laisser d'erreurs** (code ou autre) - tout doit être propre

### **🎯 Joints**
- **JAMAIS animer les antennes** : `left_antenna`, `right_antenna` (BLOQUÉES)
- **JAMAIS animer les joints passifs** : `passive_1` à `passive_7` (BLOQUÉS)
- **JAMAIS dépasser 0.3 rad** d'amplitude pour éviter les instabilités
- **TOUJOURS utiliser `yaw_body`** pour les animations visibles

### **🧪 Tests Golden**
- **JAMAIS modifier les traces de référence** sans raison valide
- **TOUJOURS respecter les tolérances** : ±0.25 rad position, ±20% cadence
- **JAMAIS commiter de nouvelles références** sans validation
- **TOUJOURS utiliser le seed fixé** : SEED=42

---

## 📚 **STANDARDS DE QUALITÉ OBLIGATOIRES**

### **🛠️ Outils de Qualité**
```bash
# TOUJOURS exécuter dans cet ordre avant tout commit :
source venv/bin/activate
black .                    # Formatage automatique
ruff check .               # Linting Python
mypy src/                  # Vérification de types
```

### **📝 Documentation Markdown**
- **TOUJOURS ajouter des schémas Mermaid** pour les nouveaux fichiers MD
- **JAMAIS de redondance** - consolider les informations
- **TOUJOURS utiliser des dates "Oct / Oct / Nov. 20255"** (sans jour spécifique)
- **TOUJOURS mettre à jour les statistiques** (531 tests, 418 passent, 76.70% coverage)

### **🎨 Schémas Mermaid Requis**
- **Graphs TB/LR** : Architecture système, relations entre composants
- **Sequence Diagrams** : Workflows, interactions entre composants
- **Pie Charts** : Répartition des métriques, priorités
- **Gantt Charts** : Roadmaps, plans de développement
- **Flowcharts** : Processus de diagnostic, résolution de problèmes

---

## 🏗️ **ARCHITECTURE ÉTABLIE**

### **📁 Structure des Fichiers**
```
src/bbia_sim/
├── robot_api.py              # Interface unifiée
├── robot_factory.py          # Factory pour backends
├── global_config.py          # Configuration globale
├── telemetry.py              # Collecte de télémétrie
└── backends/
    ├── mujoco_backend.py     # Backend MuJoCo
    └── reachy_backend.py     # Backend Reachy (mock)

examples/
├── demo_emotion_ok.py        # Émotion → Pose
├── demo_voice_ok.py          # Voix → Action
├── demo_vision_ok.py         # Vision → Suivi
└── demo_behavior_ok.py       # Comportement → Scénario

tests/
├── test_robot_api_smoke.py   # 6 tests smoke
└── test_vertical_slices.py   # 9 tests vertical slices

scripts/
└── replay_viewer.py          # Rejeu d'animations
```

### **🎯 Vertical Slices BBIA**
1. **Émotion → Pose** : `demo_emotion_ok.py`
2. **Voix → Action** : `demo_voice_ok.py`
3. **Vision → Suivi** : `demo_vision_ok.py`
4. **Comportement → Scénario** : `demo_behavior_ok.py`

---

## 🚀 **COMMANDES PRÊTES À L'EMPLOI**

### **🎮 VOIR LA 3D (Mode Graphique)**
```bash
# Démo Émotion avec 3D
mjpython examples/demo_emotion_ok.py --emotion happy --duration 10 --intensity 0.8 --backend mujoco

# Démo Voix avec 3D
mjpython examples/demo_voice_ok.py --command "regarde-moi" --duration 5 --backend mujoco

# Démo Vision avec 3D
mjpython examples/demo_vision_ok.py --target-speed 0.02 --duration 10 --backend mujoco

# Démo Comportement avec 3D
mjpython examples/demo_behavior_ok.py --behavior wake_up --duration 8 --backend mujoco
```

### **🧪 TESTS RAPIDES (Mode Headless)**
```bash
# Tests smoke RobotAPI
python -m pytest tests/test_robot_api_smoke.py -v

# Tests vertical slices
python -m pytest tests/test_vertical_slices.py -v

# Tests complets
python -m pytest tests/ -m "not e2e" -q
```

### **🔄 BACKEND SWITCHING**
```bash
# Simulation MuJoCo
python examples/demo_emotion_ok.py --backend mujoco --emotion happy --duration 5

# Robot Reachy réel (mock)
python examples/demo_emotion_ok.py --backend reachy --emotion happy --duration 5
```

---

## 📊 **MÉTRIQUES ACTUELLES**

| Métrique | Valeur Actuelle |
|----------|-----------------|
| **Tests collectés** | 531 |
| **Tests qui passent** | 418 |
| **Taux de réussite** | 79% |
| **Coverage** | 76.70% |
| **Vertical Slices** | 4 |
| **Tests Smoke** | 6 |
| **Backends unifiés** | 2 |

---

## 🎯 **TÂCHES TYPIQUES À EFFECTUER**

### **📚 Amélioration Documentation**
1. **Auditer** les fichiers Markdown existants
2. **Ajouter des schémas Mermaid** pour la clarté
3. **Éliminer la redondance** entre fichiers
4. **Mettre à jour les statistiques** obsolètes
5. **Corriger les liens cassés**

### **🔧 Amélioration Code**
1. **Respecter RobotAPI** pour toutes les nouvelles fonctionnalités
2. **Ajouter des tests** pour les nouvelles features
3. **Maintenir la compatibilité** avec les backends existants
4. **Optimiser les performances** sans casser la fonctionnalité

### **🧪 Tests et Qualité**
1. **Exécuter les outils de qualité** avant chaque commit
2. **Vérifier que tous les tests passent**
3. **Maintenir le coverage** au-dessus de 75%
4. **Documenter les nouvelles fonctionnalités**

---

## ⚠️ **ERREURS COURANTES À ÉVITER**

### **❌ Ne Jamais Faire**
- Utiliser `--headless` pour la visualisation 3D
- Utiliser MuJoCo directement au lieu de RobotAPI
- Animer les joints bloqués (antennes, passifs)
- Dépasser 0.3 rad d'amplitude
- Commiter sans exécuter black/ruff/mypy
- Utiliser des guillemets doubles dans les commits
- Laisser des erreurs de linting ou de type

### **✅ Toujours Faire**
- Utiliser `mjpython` pour la 3D
- Spécifier le backend (mujoco/reachy)
- Utiliser RobotAPI pour le backend unifié
- Exécuter les outils de qualité avant commit
- Ajouter des schémas Mermaid aux docs
- Mettre à jour les statistiques
- Travailler dans le venv activé

---

## 🔄 **WORKFLOW STANDARD**

### **1. Préparation**
```bash
source venv/bin/activate
git checkout develop
git pull origin develop
```

### **2. Développement**
- Respecter les règles strictes
- Utiliser RobotAPI
- Ajouter des tests si nécessaire
- Documenter avec Mermaid

### **3. Qualité**
```bash
black .
ruff check .
mypy src/
```

### **4. Commit et Push**
```bash
git add .
git commit -m 'Description claire avec guillemets simples'
git push origin develop
```

---

## 📚 **RESSOURCES UTILES**

### **📁 Fichiers de Référence**
- `docs/RECAPITULATIF_v1.1.0.md` : État actuel complet
- `docs/SWITCH_SIM_ROBOT.md` : Guide backend unifié
- `docs/ARCHITECTURE.md` : Architecture système
- `README.md` : Vue d'ensemble du projet

### **🔗 Liens Importants**
- **Dépôt GitHub** : https://github.com/arkalia-luna-system/bbia-sim
- **Branche de travail** : `develop`
- **Documentation** : `docs/` (complète avec Mermaid)

---

## 🎉 **OBJECTIFS DE CONTINUITÉ**

### **🎯 Maintenir**
- **Qualité du code** : Black, Ruff, MyPy passent
- **Couverture de tests** : >75%
- **Documentation** : À jour avec Mermaid
- **Architecture** : Backend unifié RobotAPI

### **🚀 Améliorer**
- **Performance** : Optimiser sans casser
- **Fonctionnalités** : Nouvelles features BBIA
- **Tests** : Augmenter la couverture
- **Documentation** : Plus de schémas visuels

### **🔒 Protéger**
- **Stabilité** : Ne rien casser
- **Compatibilité** : Backend switching fonctionnel
- **Standards** : Respecter toutes les règles
- **Qualité** : Maintenir les métriques

---

**Tu es maintenant prêt à reprendre le travail sur BBIA-Reachy-SIM avec toutes les règles et standards établis !** 🚀

**Rappel : Utilise `mjpython` pour la 3D, respecte RobotAPI, et exécute toujours black/ruff/mypy avant de commiter !** 🎮
