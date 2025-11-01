# 🤖 Mock Robot + Tests Hardware Conditionnels

**Date**: 2025-10-31  
**Branche**: `future`

---

## 📋 Résumé

Création d'un mock complet pour tests robot sans hardware et mise en place d'un système de tests hardware conditionnels avec variable d'environnement.

---

## 🎭 Mock ReachyMiniMock

### Fichier: `tests/reachy_mini_mock.py`

Mock complet qui simule un `ReachyMiniBackend` sans nécessiter de robot physique.

### Fonctionnalités Simulées

1. **Connexion/Déconnexion**
   - `connect()` / `disconnect()` simulés
   - État `is_connected` géré

2. **Joints**
   - Liste complète des joints (yaw_body, stewart_1-6, antennas)
   - `set_joint_pos()` / `get_joint_pos()` avec validation
   - Limites physiques appliquées
   - Clamping selon `safe_amplitude_limit`

3. **Simulation**
   - `step()` avec délai minimal (0.001s)
   - Compteur `step_count`

4. **Mouvements**
   - `goto_target()` simulé (interpolation simplifiée)
   - `emergency_stop()` (réinitialise positions)

5. **Modules SDK**
   - `io` (MockIO)
   - `media` (MockMedia) avec:
     - `camera` (MockCamera) - frame vide simulée
     - `microphone` (MockMicrophone)
     - `speaker` (MockSpeaker)
   - `robot` (MockRobotSDK) avec head/joints

### Utilisation

```python
from tests.reachy_mini_mock import ReachyMiniMock

# Créer mock
mock_robot = ReachyMiniMock()

# Utiliser comme un backend normal
mock_robot.connect()
mock_robot.set_joint_pos("yaw_body", 0.5)
pos = mock_robot.get_joint_pos("yaw_body")
assert pos == 0.5
mock_robot.disconnect()
```

---

## 🔧 Tests Hardware Conditionnels

### Variable d'Environnement: `SKIP_HARDWARE_TESTS`

**Par défaut**: `SKIP_HARDWARE_TESTS=1` (tests désactivés)

**Pour activer**: `SKIP_HARDWARE_TESTS=0`

### Implémentation

```python
@pytest.mark.skipif(
    os.environ.get("SKIP_HARDWARE_TESTS", "1") == "1",
    reason="Tests hardware désactivés par défaut. Définir SKIP_HARDWARE_TESTS=0 pour activer",
)
class TestReachyMiniBackendReal:
    """Tests pour le backend avec SDK réel (nécessite robot physique)."""
    ...
```

### Tests Modifiés

#### `tests/test_reachy_mini_backend.py`

- `TestReachyMiniBackendReal`: Classe complète de tests hardware
  - `test_real_connection()`: Test connexion robot réel
  - `test_real_joint_control()`: Test contrôle joints réel

### Comportement

1. **Par défaut (SKIP_HARDWARE_TESTS=1)**:
   - Tests skippés automatiquement
   - Pas de timeout ni d'erreur
   - Message explicite: "Tests hardware désactivés par défaut..."

2. **Avec robot (SKIP_HARDWARE_TESTS=0)**:
   - Tests exécutés normalement
   - `use_sim=False` forcé pour chercher robot réel
   - Skip si backend indisponible ou connexion échouée
   - Nettoyage automatique (disconnect)

---

## 🚀 Utilisation

### Tests Normaux (Sans Robot)

```bash
# Tous les tests (hardware skippés par défaut)
pytest tests/

# Tests spécifiques
pytest tests/test_reachy_mini_backend.py
```

### Tests Avec Robot Physique

```bash
# Activer tests hardware
SKIP_HARDWARE_TESTS=0 pytest tests/test_reachy_mini_backend.py::TestReachyMiniBackendReal

# Ou en exportant la variable
export SKIP_HARDWARE_TESTS=0
pytest tests/test_reachy_mini_backend.py
```

### Utiliser le Mock dans Tests

```python
# Dans un test
from tests.reachy_mini_mock import ReachyMiniMock

def test_something_with_robot():
    robot = ReachyMiniMock()
    robot.connect()
    
    # Tester fonctionnalités...
    
    robot.disconnect()
```

---

## 📊 Avantages

### 1. Tests Sans Hardware
- ✅ Pas besoin de robot pour développer
- ✅ Tests rapides (pas de timeout réseau)
- ✅ CI/CD fonctionne sans hardware

### 2. Tests Avec Hardware
- ✅ Activation simple via variable d'environnement
- ✅ Pas de modification code nécessaire
- ✅ Tests réels quand robot disponible

### 3. Flexibilité
- ✅ Mock utilisable dans n'importe quel test
- ✅ Même interface que backend réel
- ✅ Simulation réaliste (limites, validation)

---

## 🔄 Migration Autres Tests

Pour migrer d'autres tests hardware:

```python
# Avant
@pytest.mark.skip(reason="Test nécessite robot physique")

# Après
@pytest.mark.skipif(
    os.environ.get("SKIP_HARDWARE_TESTS", "1") == "1",
    reason="Tests hardware désactivés. SKIP_HARDWARE_TESTS=0 pour activer",
)
def test_hardware_thing():
    robot = RobotFactory.create_backend("reachy_mini", use_sim=False)
    if robot is None:
        pytest.skip("Backend non disponible")
    # ...
```

---

## ✅ Validation

- [x] Mock créé et fonctionnel
- [x] Tests hardware conditionnels implémentés
- [x] Tests skippés par défaut (pas d'erreur)
- [x] Activation avec variable d'environnement
- [x] Documentation créée
- [x] Commit + push sur `future`

---

**Commit**: `d157b11`  
**Fichiers créés/modifiés**:
- `tests/reachy_mini_mock.py` (nouveau, 261 lignes)
- `tests/test_reachy_mini_backend.py` (modifié)

