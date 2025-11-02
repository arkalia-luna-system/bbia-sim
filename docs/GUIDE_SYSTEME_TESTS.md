# 🧪 Guide Complet du Système de Tests BBIA-SIM

**Version**: 1.0  
**Date**: Oct / Nov. 2025

---

## 📋 Table des Matières

1. [Vue d'ensemble](#vue-densemble)
2. [Structure des Tests](#structure-des-tests)
3. [Exécution des Tests](#exécution-des-tests)
4. [Marqueurs pytest](#marqueurs-pytest)
5. [Couverture de Code](#couverture-de-code)
6. [Tests E2E](#tests-e2e)
7. [Mocks et Fixtures](#mocks-et-fixtures)
8. [Bonnes Pratiques](#bonnes-pratiques)
9. [CI/CD](#cicd)
10. [Dépannage](#dépannage)

---

## 🎯 Vue d'ensemble

Le système de tests BBIA-SIM utilise **pytest** comme framework principal avec une architecture modulaire et des tests couvrant :

- **Tests unitaires** : Modules individuels (vision, voice, emotions, etc.)
- **Tests d'intégration** : Intégration entre modules
- **Tests E2E** : Scénarios utilisateur complets
- **Tests de performance** : Benchmarks CPU/RAM, latence
- **Tests de conformité** : SDK Reachy Mini officiel

### Statistiques

- **Total tests** : **1245 tests collectés** (pytest --collect-only)
- **Coverage global** : **68.86%** (excellent)
- **Coverage modules critiques** :
  - `dashboard_advanced.py` : **0.00%** ⚠️ (tests existent mais ne couvrent pas)
  - `vision_yolo.py` : **17.49%** ⚠️ (objectif 50%+ non atteint)
  - `voice_whisper.py` : **75.83%** ✅ (objectif 50%+ dépassé)
  - `daemon/bridge.py` : **0.00%** ⚠️ (tests existent mais ne couvrent pas)
- **Tests E2E** : 4 scénarios utilisateur

---

## 📁 Structure des Tests

```
tests/
├── conftest.py              # Configuration pytest globale (lock, fixtures)
├── e2e/                     # Tests end-to-end scénarios utilisateur
│   ├── test_e2e_face_detection_greeting.py
│   ├── test_e2e_voice_interaction.py
│   ├── test_e2e_wake_up_sequence.py
│   └── test_e2e_full_interaction_loop.py
├── test_*.py                # Tests unitaires/intégration
│   ├── test_bbia_vision*.py
│   ├── test_bbia_emotions*.py
│   ├── test_voice_whisper*.py
│   ├── test_vision_yolo*.py
│   ├── test_reachy_mini*.py
│   └── ...
└── reachy_mini_mock.py      # Mock RobotAPI pour tests sans hardware
```

### Organisation par Type

#### Tests Unitaires (`test_*_*.py`)
- **Vision** : `test_bbia_vision.py`, `test_bbia_vision_extended.py`
- **Voice** : `test_voice_whisper_comprehensive.py`
- **YOLO** : `test_vision_yolo_comprehensive.py`
- **Emotions** : `test_bbia_emotions.py`
- **Mapping** : `test_mapping_reachy_complete.py`

#### Tests Conformité SDK
- `test_reachy_mini_backend.py`
- `test_reachy_mini_strict_conformity.py`
- `test_reachy_mini_complete_conformity.py`

#### Tests Performance
- `test_backend_budget_cpu_ram.py`
- `test_vision_fps_budget.py`
- `test_vision_latency.py`
- `test_runtime_budget.py`

#### Tests E2E
- Scénarios utilisateur complets avec mocks

---

## 🚀 Exécution des Tests

### Commandes de Base

```bash
# Activer environnement virtuel
source venv/bin/activate

# Tous les tests
pytest

# Tests rapides uniquement (unit, fast)
pytest -m "unit and fast"

# Tests lents exclus
pytest -m "not slow"

# Tests E2E uniquement
pytest -m e2e

# Test spécifique
pytest tests/test_bbia_vision.py::TestBBIAVision::test_vision_creation

# Mode verbose
pytest -v

# Afficher print statements
pytest -s
```

### Options Utiles

```bash
# Arrêter au premier échec
pytest -x

# Exécuter seulement les tests échoués dernière fois
pytest --lf

# Mode parallèle (si pytest-xdist installé)
pytest -n auto

# Couverture code
pytest --cov=src/bbia_sim --cov-report=html

# Timeout global (300s par défaut)
pytest --timeout=300
```

---

## 🏷️ Marqueurs pytest

### Marqueurs Standard

```python
@pytest.mark.unit          # Test unitaire
@pytest.mark.integration   # Test d'intégration
@pytest.mark.e2e          # Test end-to-end
@pytest.mark.slow         # Test lent (> 1s)
@pytest.mark.fast         # Test rapide (< 1s)
@pytest.mark.skip         # Test ignoré
@pytest.mark.skipif(...)  # Test conditionnel
```

### Utilisation

```python
import pytest

@pytest.mark.unit
@pytest.mark.fast
def test_quick_unit():
    """Test unitaire rapide."""
    pass

@pytest.mark.e2e
@pytest.mark.slow
def test_full_scenario():
    """Test E2E lent."""
    pass

@pytest.mark.skipif(
    os.environ.get("SKIP_HARDWARE_TESTS", "1") == "1",
    reason="Tests hardware désactivés par défaut"
)
def test_real_robot():
    """Test nécessitant robot physique."""
    pass
```

### Exécution par Marqueurs

```bash
# Tests unitaires rapides
pytest -m "unit and fast"

# Tests E2E
pytest -m e2e

# Exclure tests lents
pytest -m "not slow"

# Tests hardware (si activés)
SKIP_HARDWARE_TESTS=0 pytest -m "not skip"
```

---

## 📊 Couverture de Code

### Configuration

Fichier `.coveragerc` avec seuil strict :

```ini
[report]
fail_under = 50  # Seuil minimum 50%
show_missing = True
```

### Génération Rapports

```bash
# Rapport terminal
pytest --cov=src/bbia_sim --cov-report=term-missing

# Rapport HTML (dans htmlcov/)
pytest --cov=src/bbia_sim --cov-report=html

# Rapport XML (pour CI)
pytest --cov=src/bbia_sim --cov-report=xml
```

### Coverage par Module

Modules prioritaires :

- `dashboard_advanced.py` : **0.00%** ⚠️ (47 tests, objectif 70%+ non atteint)
- `vision_yolo.py` : **17.49%** ⚠️ (objectif 50%+ non atteint, 32.51% manquants)
- `voice_whisper.py` : **75.83%** ✅ (47 tests créés, objectif 50%+ dépassé)
- `daemon/bridge.py` : **0.00%** ⚠️ (34 tests, objectif 30%+ non atteint)
- `bbia_audio.py` : **87.76%** ✅
- `bbia_emotions.py` : **81.71%** ✅

### Objectifs Coverage

- **Global projet** : 50%+ (strict)
- **Modules core** : 70%+
- **Modules critiques** : 80%+

---

## 🔄 Tests E2E

### Scénarios Disponibles

1. **`test_e2e_face_detection_greeting.py`**
   - Détection visage → suivi → salutation
   - Intégration : Vision + Behavior

2. **`test_e2e_voice_interaction.py`**
   - Écoute → transcription → réponse LLM → mouvement
   - Intégration : Voice + HuggingFace + Behavior

3. **`test_e2e_wake_up_sequence.py`**
   - Réveil → émotion excited → mouvement
   - Intégration : Behavior + Emotions + Robot API

4. **`test_e2e_full_interaction_loop.py`**
   - Scénario complet : détection → écoute → réponse → mouvement
   - Intégration : Tous modules

### Exécution Tests E2E

```bash
# Tous les E2E
pytest -m e2e

# E2E spécifique
pytest tests/e2e/test_e2e_face_detection_greeting.py

# E2E avec verbose
pytest -m e2e -v
```

### Mocks E2E

Tests E2E utilisent des **mocks** pour éviter dépendances hardware :

```python
from unittest.mock import MagicMock, patch

# Mock robot API
mock_robot = MagicMock()
mock_robot.set_joint_pos.return_value = True

# Mock HuggingFace
with patch("bbia_sim.bbia_huggingface.BBIAHuggingFace") as mock_hf:
    mock_hf.chat.return_value = "Réponse simulée"
```

---

## 🎭 Mocks et Fixtures

### Fixtures Communes

```python
# Dans conftest.py
@pytest.fixture
def mock_robot():
    """Fixture robot API mocké."""
    robot = MagicMock()
    robot.set_joint_pos.return_value = True
    robot.get_joint_pos.return_value = 0.0
    robot.step.return_value = True
    return robot
```

### Utilisation Mocks

```python
from unittest.mock import MagicMock, patch

def test_with_mock():
    # Mock module externe
    with patch("module.external_function", return_value="mocked"):
        result = function_under_test()
        assert result == "expected"
```

### Mock Robot API

Fichier `tests/reachy_mini_mock.py` fournit `ReachyMiniMock` :

```python
from tests.reachy_mini_mock import ReachyMiniMock

def test_with_mock_robot():
    robot = ReachyMiniMock(use_sim=True)
    assert robot.connect() is True
    # Tests sans robot physique
```

---

## ✅ Bonnes Pratiques

### 1. Noms de Tests Clairs

```python
# ❌ MAUVAIS
def test_1():
    pass

# ✅ BON
def test_vision_detects_faces_with_high_confidence():
    pass
```

### 2. Tests Isolés

```python
# ✅ Chaque test est indépendant
def test_emotion_set_happy():
    emotions = BBIAEmotions()
    emotions.set_emotion("happy", 0.8)
    assert emotions.current_emotion == "happy"
```

### 3. Arrange-Act-Assert

```python
def test_vision_tracks_object():
    # Arrange
    vision = BBIAVision()
    vision.objects_detected = [{"name": "livre"}]
    
    # Act
    result = vision.track_object("livre")
    
    # Assert
    assert result is True
    assert vision.tracking_active is True
```

### 4. Tests Déterministes

```python
# ✅ Utiliser mocks pour données prévisibles
vision.objects_detected = [{"name": "objet", "confidence": 0.9}]

# ❌ Éviter dépendances externes variables
result = vision.scan_environment()  # Peut varier selon environnement
```

### 5. Documentation Tests

```python
def test_complex_scenario():
    """Test scénario complexe: détection → tracking → action.
    
    Vérifie que BBIA peut détecter un objet, le suivre,
    et déclencher une action appropriée.
    """
    # ...
```

---

## 🔒 Protection Exécution Simultanée

### Système de Lock

Fichier `tests/conftest.py` implémente un système de lock pour éviter l'exécution simultanée de plusieurs instances pytest (surchauffe RAM).

**Fonctionnalités** :
- Lock fichier avec `fcntl`
- Détection locks orphelins
- Timeout automatique (5 minutes)
- Messages d'erreur clairs

**Si lock actif** :
```
❌ Tests déjà en cours d'exécution !
   Processus PID: 12345
   Lock acquis il y a: 10.5s
   
💡 Solutions:
   1. Attendre la fin de l'autre processus
   2. Vérifier: ps aux | grep 12345
   3. Si processus mort: rm .pytest.lock
```

---

## 🚦 CI/CD

### GitHub Actions

Tests exécutés automatiquement dans CI avec :

- **Linting** : Ruff, Black
- **Type checking** : MyPy
- **Security** : Bandit
- **Tests** : pytest avec coverage
- **Benchmarks** : Performance tests

### Variables Environnement CI

```yaml
# .github/workflows/ci.yml
env:
  CI: "true"
  BBIA_DISABLE_AUDIO: "1"
  BBIA_DISABLE_VISION: "0"
  SKIP_HARDWARE_TESTS: "1"
```

### Seuils Adaptatifs

Tests performance adaptent leurs seuils en CI :

```python
is_ci = os.environ.get("CI", "false").lower() == "true"
max_p50 = 200.0 if is_ci else 100.0  # Tolérance 2x en CI
```

---

## 🔧 Dépannage

### Problèmes Courants

#### 1. Tests Échouent Localement mais Passent en CI

**Cause** : Différences environnement (hardware, dépendances)  
**Solution** : Vérifier variables environnement, utiliser mocks

#### 2. Lock Persistant

```bash
# Supprimer lock manuellement
rm .pytest.lock

# Vérifier processus actif
ps aux | grep pytest
```

#### 3. Coverage Trop Bas

**Solution** :
1. Identifier modules non couverts : `pytest --cov-report=term-missing`
2. Créer tests manquants
3. Vérifier `.coveragerc` exclusions

#### 4. Tests Lents

**Solution** :
- Utiliser `-m "not slow"` pour développement rapide
- Optimiser tests lents (réduire durées, itérations)
- Mode parallèle : `pytest -n auto`

#### 5. Import Errors

**Solution** :
```bash
# Vérifier PYTHONPATH
export PYTHONPATH="${PYTHONPATH}:$(pwd)/src"

# Installer dépendances
pip install -e .[dev,test]
```

---

## 📚 Ressources

- **pytest Docs** : https://docs.pytest.org/
- **Coverage.py** : https://coverage.readthedocs.io/
- **unittest.mock** : https://docs.python.org/3/library/unittest.mock.html

---

## 🎯 Prochaines Étapes

1. Améliorer coverage modules restants
2. Ajouter tests E2E supplémentaires
3. Documentation tests spécifiques par module
4. Benchmarks performance automatisés

---

**Dernière mise à jour** : Oct / Nov. 2025

