# 🧪 Tests BBIA-SIM

> **Tests unitaires et d'intégration pour BBIA-SIM - Brain-Based Interactive Agent**

## 🎯 **Couverture de Tests**

### 📊 **Statistiques de Couverture**
- **Simulateur MuJoCo** : 97% de couverture
- **Service de Simulation** : 90% de couverture  
- **Routers API** : 99% de couverture (Motion: 98%, State: 100%)
- **Configuration** : 100% de couverture
- **Middleware & Modèles** : 91-95% de couverture
- **Module CLI** : Tests de base implémentés

### 🚀 **Lancer les Tests**

```bash
# Tests unitaires complets
pytest tests/ -v

# Tests avec couverture
pytest --cov=src/bbia_sim --cov-report=html

# Tests rapides (sans E2E)
pytest tests/ -v -m "not e2e"

# Tests spécifiques
pytest tests/test_simulator.py -v
pytest tests/test_simulation_service.py -v
pytest tests/test_routers.py -v
```

---

## 📁 **Structure des Tests**

```
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
├── 🌐 e2e/                           # Tests end-to-end
│   ├── test_motion_roundtrip.py      # Tests E2E motion
│   └── __init__.py
└── 📖 README.md                      # Ce fichier
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
**Date** : Janvier 2025  
**Tests** : ✅ 215+ tests fonctionnels  
**Couverture** : ✅ ≥80% (objectif atteint)  
**Qualité** : ✅ Black/Ruff/MyPy compliant