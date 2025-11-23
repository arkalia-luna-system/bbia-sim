# 🧪 Tests BBIA-SIM

> **Tests unitaires et d'intégration pour BBIA-SIM - Brain-Based Interactive Agent**

## 🎯 **Couverture de Tests**

### 📊 **Statistiques de Couverture ACTUELLES**

- **Coverage total** : **68.86%** (excellent)
- **1,743 tests collectés** (1,805 total, 62 deselected) — [✅ Vérifié Décembre 2025](../docs/quality/audits/VERIFICATION_ANALYSE_COMPLETE_DEC2025.md)
- **Tests passent** : Voir résultats pytest récents
- **Tests skippés** : Tests conditionnels (hardware, CI)
- **Golden Tests** : 3 traces référence + validation

### 📊 **Détail par Module**

- **bbia_audio.py** : **98.47%** ✅ (amélioré Janvier 2025)
- **bbia_behavior.py** : **72.50%** ✅
- **bbia_emotions.py** : **81.71%** ✅
- **bbia_vision.py** : **88.52%** ✅
- **bbia_voice.py** : **~65-70%** ✅ (amélioré Janvier 2025)
- **daemon/config.py** : **100%** ✅
- **daemon/models.py** : **95.35%** ✅
- **daemon/middleware.py** : **91.30%** ✅
- **daemon/app/routers/motion.py** : **93.22%** ✅
- **daemon/simulation_service.py** : **89.83%** ✅
- **sim/simulator.py** : **90.00%** ✅
- **unity_reachy_controller.py** : **81.20%** ✅

### 📊 **Modules Critiques (Coverage Excellent)**

- **vision_yolo.py** : **99.45%** ✅ (42 tests) - Objectif 50%+ largement dépassé
- **voice_whisper.py** : **92.52%** ✅ (66 tests) - Objectif 50%+ largement dépassé
- **dashboard_advanced.py** : **76.71%** ✅ (47 tests, 1156 lignes) - Objectif 50%+ dépassé
- **daemon/bridge.py** : **55.41%** ✅ (34 tests) - Objectif 30%+ dépassé

**Total tests modules critiques** : **189 tests** pour les 4 modules critiques

### 🧪 **Golden Tests**

- **test_golden_traces.py** : 3 tests de non-régression
- **Traces référence** : happy_mujoco.jsonl, lookat_mujoco.jsonl, wakeup_mujoco.jsonl
- **Tolérances** : ±0.25 rad position, ±20% cadence
- **Seed fixé** : SEED=42 pour reproductibilité

### 🚀 **Lancer les Tests**

```bash
# Tests complets avec coverage (recommandé)
python -m pytest tests/ --cov=src --cov-report=term-missing --cov-report=html

# Tests rapides sans détails
python -m pytest tests/ --cov=src --cov-fail-under=0 --tb=no -q

# Tests avec arrêt au premier échec
python -m pytest tests/ --cov=src --cov-report=term-missing -x

# Tests spécifiques
python -m pytest tests/test_bbia_emotions.py -v
python -m pytest tests/e2e/ -v

# Voir le rapport HTML de coverage
open htmlcov/index.html

# Tests Golden Traces
python -m pytest tests/test_golden_traces.py -v

# Tests RobotAPI Limits
python -m pytest tests/test_robot_api_limits.py -v

# Tests Vertical Slices
python -m pytest tests/test_vertical_slices.py -v
```

### 🧪 **Tests Spécialisés**

#### **Golden Tests**

```bash
# Tests de non-régression
python -m pytest tests/test_golden_traces.py -v

# Régénérer une trace de référence
python scripts/record_trace.py --emotion happy --duration 5

# Valider une trace contre référence
python scripts/validate_trace.py --ref artifacts/golden/happy_mujoco.jsonl --cur current_trace.jsonl
```

#### **RobotAPI Tests**

```bash
# Tests limites et sécurité
python -m pytest tests/test_robot_api_limits.py -v

# Tests smoke
python -m pytest tests/test_robot_api_smoke.py -v
```

#### **Vertical Slices**

```bash
# Tests 4 démos BBIA
python -m pytest tests/test_vertical_slices.py -v
```

---

## 📁 **Structure des Tests**

```text
tests/
├── 🧪 test_simulator.py              # Tests MuJoCo Simulator (97% couverture)
├── 🧪 test_simulation_service.py     # Tests Simulation Service (90% couverture)
├── 🧪 test_routers.py                # Tests API Routers (99% couverture)
├── 🧪 test_config.py                 # Tests Configuration (100% couverture)
├── 🧪 test_middleware.py             # Tests Middleware (91% couverture)
├── 🧪 test_models.py                 # Tests Modèles Pydantic (95% couverture)
├── 🧪 test_main.py                   # Tests CLI Module
├── 🧪 test_simulation_integration.py # Tests d'intégration
├── 🧪 test_api_integration.py        # Tests API d'intégration
├── 🧪 test_joints.py                 # Tests validation joints
├── 🧪 test_duration.py               # Tests durée simulation
├── 🤖 test_reachy_mini_*.py          # Tests Backend Reachy Mini (118 tests, voir section dédiée)
├── 🌐 e2e/                           # Tests end-to-end
│   ├── test_motion_roundtrip.py      # Tests E2E motion
│   └── __init__.py
└── 📖 README.md                      # Ce fichier
```

### 🤖 **Tests Backend Reachy Mini**

**📊 Statistiques (21 novembre 2025)**:

- ✅ **118 tests** répartis dans **8 fichiers complémentaires**
- ✅ **116 tests uniques** (98.3% - très peu de redondance)
- ✅ **1 doublon mineur** (`test_robot_factory_integration`)

**📁 Fichiers de tests** (tous complémentaires, à conserver):

| Fichier | Tests | Rôle Principal |
|---------|-------|---------------|
| `test_reachy_mini_full_conformity_official.py` | 37 | Conformité complète SDK officiel |
| `test_reachy_mini_backend.py` | 24 | Tests de base du backend |
| `test_reachy_mini_complete_conformity.py` | 16 | Conformité API complète |
| `test_reachy_mini_advanced_conformity.py` | 12 | Patterns/optimisations expertes |
| `test_reachy_mini_strict_conformity.py` | 10 | Tests stricts (valeurs exactes XML) |
| `test_reachy_mini_backend_extended.py` | 9 | Tests structure/compatibilité |
| `test_reachy_mini_backend_rapid.py` | 8 | Tests coverage rapide |
| `test_reachy_mini_conformity.py` | 2 | Script de vérification |

**💡 Documentation complète**: Voir [`REACHY_MINI_TESTS_STRUCTURE.md`](REACHY_MINI_TESTS_STRUCTURE.md)

```bash
# Lancer tous les tests Reachy Mini
pytest tests/test_reachy_mini*.py -v

# Test de conformité complet (recommandé)
pytest tests/test_reachy_mini_full_conformity_official.py -v

# Tests stricts (valeurs exactes)
pytest tests/test_reachy_mini_strict_conformity.py -v

# Vérifier les doublons/redondances
python scripts/verify_tests_consolidation.py
```

---

## 🎯 **Types de Tests**

### 🧪 **Tests Unitaires**

- **Simulateur** : Tests MuJoCo headless/graphique, gestion erreurs, clamp angles
- **Service** : Cycle de vie simulation, gestion erreurs, état robot
- **Routers** : Endpoints API, validation données, gestion erreurs
- **Configuration** : Environnements dev/prod, sécurité, CORS
- **Middleware** : Rate limiting, sécurité, headers
- **Modèles** : Validation Pydantic, contraintes données

### 🔗 **Tests d'Intégration**

- **Simulation** : Tests durée, performance, accès concurrent
- **API** : Tests endpoints complets, middleware, authentification
- **Joints** : Validation limites physiques, clamp automatique

### 🌐 **Tests End-to-End**

- **Motion** : Cycle complet GET → SET → GET avec vérification
- **WebSocket** : Télémétrie temps réel, fréquence messages
- **Performance** : Temps de réponse, mouvements concurrents

---

## 🎯 **Qualité du Code**

### ✅ **Standards Respectés**

- **Black** : Formatage automatique
- **Ruff** : Linting et corrections automatiques  
- **MyPy** : Vérification types statiques
- **Pytest** : Framework de tests moderne

### 🚀 **Commandes Qualité**

```bash
# Vérification complète
ruff check src/ tests/
black --check src/ tests/
mypy src/

# Correction automatique
ruff check src/ tests/ --fix
black src/ tests/
```

---

## 🎯 **Tests par Module**

### 🤖 **Simulateur MuJoCo** (`test_simulator.py`)

- ✅ Simulation headless/graphique
- ✅ Chargement scènes avec gestion erreurs
- ✅ Clamp angles dans limites physiques
- ✅ Gestion erreurs joints inexistants
- ✅ Fermeture propre simulateur

### 🔧 **Service Simulation** (`test_simulation_service.py`)

- ✅ Cycle de vie start/stop simulation
- ✅ Mode graphique avec fallback headless
- ✅ Gestion erreurs simulateur
- ✅ État robot et positions joints
- ✅ Méthodes par défaut

### 🌐 **Routers API** (`test_routers.py`)

- ✅ Endpoints motion (joints, gripper, head, stop)
- ✅ Endpoints state (full, position, battery, temperature, status, sensors)
- ✅ Validation joints invalides
- ✅ Clamp angles avec warnings
- ✅ Validation gripper (côté/action)

### ⚙️ **Configuration** (`test_config.py`)

- ✅ Environnements dev/prod
- ✅ Chargement dotenv avec gestion erreurs
- ✅ CORS origins selon environnement
- ✅ Headers sécurité production
- ✅ Masquage tokens pour logs

### 🛡️ **Middleware** (`test_middleware.py`)

- ✅ Rate limiting par minute
- ✅ Headers sécurité
- ✅ Gestion requêtes volumineuses
- ✅ Ordre middlewares

### 📋 **Modèles** (`test_models.py`)

- ✅ Validation Pydantic complète
- ✅ Contraintes physiques réalistes
- ✅ Messages d'erreur explicites
- ✅ Valeurs par défaut

---

## 🎯 **Tests d'Intégration**

### 🔗 **Simulation** (`test_simulation_integration.py`)

- ✅ Durée headless précise (±0.05s)
- ✅ Performance simulation
- ✅ Accès concurrent sécurisé
- ✅ Gestion erreurs robuste

### 🌐 **API** (`test_api_integration.py`)

- ✅ Endpoints complets avec authentification
- ✅ Middleware CORS et sécurité
- ✅ Gestion erreurs HTTP
- ✅ Headers sécurité

### 🎯 **Joints** (`test_joints.py`)

- ✅ Validation noms joints autorisés
- ✅ Clamp angles dans limites physiques
- ✅ Gestion erreurs explicites

---

## 🎯 **Tests End-to-End**

### 🌐 **Motion Roundtrip** (`e2e/test_motion_roundtrip.py`)

- ✅ Cycle complet GET → SET → GET
- ✅ Vérification changement positions
- ✅ WebSocket télémétrie temps réel
- ✅ Rejet joints invalides (422)
- ✅ Clamp angles hors limites
- ✅ Performance mouvements concurrents

---

## 🎯 **Commandes Rapides**

### 🚀 **Tests Principaux**

```bash
# Tous les tests
pytest tests/ -v

# Tests avec couverture
pytest --cov=src/bbia_sim --cov-report=html

# Tests spécifiques
pytest tests/test_simulator.py -v
pytest tests/test_simulation_service.py -v
pytest tests/test_routers.py -v
```

### 🔍 **Vérifications Qualité**

```bash
# Linting et formatage
ruff check src/ tests/
black --check src/ tests/
mypy src/

# Correction automatique
ruff check src/ tests/ --fix
black src/ tests/
```

### 📊 **Couverture**

```bash
# Rapport HTML
pytest --cov=src/bbia_sim --cov-report=html
open htmlcov/index.html

# Rapport terminal
pytest --cov=src/bbia_sim --cov-report=term-missing
```

---

## 🎯 **Résultats Attendus**

### ✅ **Tests Unitaires**

- **215+ tests** passent avec succès
- **Couverture globale** ≥80% (objectif atteint)
- **Qualité code** : Black/Ruff/MyPy ✅
- **Stabilité** : Aucun test flaky

### ✅ **Tests d'Intégration**

- **Simulation** : Durée précise, performance optimale
- **API** : Endpoints robustes, authentification sécurisée
- **Joints** : Validation physique réaliste

### ✅ **Tests E2E**

- **Motion** : Cycle complet fonctionnel
- **WebSocket** : Télémétrie temps réel
- **Performance** : Réponses rapides

---

## 💡 **Conseils d'Utilisation**

1. **Tests rapides** : `pytest tests/ -v -m "not e2e"`
2. **Couverture** : `pytest --cov=src/bbia_sim --cov-report=html`
3. **Qualité** : `ruff check src/ tests/ && black --check src/ tests/ && mypy src/`
4. **Tests spécifiques** : `pytest tests/test_simulator.py -v`
5. **Debug** : `pytest tests/ -v -s --tb=short`

---

## 🎯 **Dépannage**

### ❌ **Problèmes Courants**

- **Tests E2E lents** : Utilisez `-m "not e2e"` pour les exclure
- **Erreurs MuJoCo** : Tests en mode headless uniquement
- **Imports** : Vérifiez les chemins relatifs dans les tests

### ✅ **Solutions**

- **Tous les tests** sont fonctionnels et stables
- **Gestion d'erreurs** robuste avec mocks appropriés
- **Documentation** complète pour chaque test
- **CI/CD** prêt avec GitHub Actions

---

**BBIA-SIM** - Tests et Qualité 🧪✨

**Version** : 2.0  
**Date** : Décembre 2025  
**Tests** : ✅ **1,362 tests collectés** (1,804 fonctions de test identifiées, CI GitHub Actions)  
**Couverture** : ✅ **68.86%** (excellent)  
**Couverture modules critiques** : ✅ **99.45%** vision_yolo, **92.52%** voice_whisper, **76.71%** dashboard_advanced  
**Qualité** : ✅ Black/Ruff/MyPy compliant

**Dernière mise à jour** : Décembre 2025 - Coverage modules critiques amélioré, auth WebSocket ajoutée
