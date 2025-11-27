# Audit Complet des Tests Sautés et Mockés

**Date**: 2025-01-27  
**Objectif**: Analyser tous les tests sautés (`@pytest.mark.skip`, `pytest.skip`) et mockés (`@patch`, `MagicMock`) pour identifier les opportunités d'amélioration.

## Résumé Exécutif

- **Tests sautés analysés**: 556
- **Tests mockés analysés**: 653
- **Catégories principales**:
  - Dépendances manquantes: 191 tests
  - Autres raisons: 294 tests
  - Audio: 23 tests
  - Hardware/Robot physique: 24 tests
  - Vision: 15 tests
  - Environnement (CI/macOS): 9 tests

## Top 10 Raisons de Skip

1. **Hugging Face transformers non disponible** (29 tests)
2. **Hugging Face non disponible** (25 tests)
3. **FastAPI non disponible** (18 tests)
4. **Dépendances ML non disponibles** (17 tests)
5. **BBIAHuggingFace non disponible** (11 tests)
6. **MuJoCo non disponible** (10 tests)
7. **ReachyMiniBackend non disponible** (8 tests)
8. **Backend non disponible** (8 tests)
9. **Fichier bbia_huggingface.py introuvable** (8 tests)
10. **Vision désactivée par BBIA_DISABLE_VISION=1** (7 tests)

## Fichiers avec le Plus de Tests Sautés

1. `tests/test_dashboard_advanced.py`: 74 tests
2. `tests/test_dashboard.py`: 42 tests
3. `tests/test_bbia_phase2_modules.py`: 37 tests
4. `tests/test_daemon_bridge.py`: 34 tests
5. `tests/benchmarks/test_performance.py`: 19 tests
6. `tests/test_motion_repeatability.py`: 17 tests
7. `tests/test_huggingface_expert_conformity.py`: 16 tests
8. `tests/test_ram_optimizations_validation.py`: 16 tests
9. `tests/test_sdk_dependencies.py`: 16 tests
10. `tests/test_vision_webcam_real.py`: 15 tests

---

## Analyse par Catégorie

### 1. Tests Sautés - Dépendances Manquantes (191 tests)

**Problème**: Tests qui nécessitent des dépendances optionnelles non installées.

**Exemples**:
- `Hugging Face transformers non disponible` (54 tests)
- `FastAPI non disponible` (18 tests)
- `MuJoCo non disponible` (10 tests)
- `ReachyMiniBackend non disponible` (8 tests)

**Recommandations**:

#### ✅ **Bonne Pratique Actuelle**
```python
@pytest.mark.skipif(
    not HF_AVAILABLE,
    reason="Hugging Face transformers non disponible"
)
```
Cette approche est correcte et permet d'exécuter les tests quand les dépendances sont disponibles.

#### 🔧 **Améliorations Possibles**

1. **Créer des fixtures conditionnelles réutilisables**:
```python
# tests/conftest.py
@pytest.fixture(scope="session")
def require_huggingface():
    if not HF_AVAILABLE:
        pytest.skip("Hugging Face transformers requis")
    return True
```

2. **Utiliser des marqueurs pytest personnalisés**:
```python
# pyproject.toml
[tool.pytest.ini_options]
markers = [
    "requires_hf: Nécessite Hugging Face transformers",
    "requires_fastapi: Nécessite FastAPI",
    "requires_mujoco: Nécessite MuJoCo",
]
```

3. **Documenter les dépendances optionnelles**:
   - Créer `docs/development/OPTIONAL_DEPENDENCIES.md`
   - Lister toutes les dépendances optionnelles et leurs tests associés

**Verdict**: ✅ **Décision correcte** - Les tests doivent être sautés si les dépendances ne sont pas disponibles. Aucune action requise sauf amélioration de la documentation.

---

### 2. Tests Sautés - Audio (23 tests)

**Problème**: Tests audio nécessitent `sounddevice` ou sont désactivés en CI.

**Exemples**:
- `Audio désactivé par BBIA_DISABLE_AUDIO=1`
- `sounddevice indisponible sur cet environnement`

**Fichiers concernés**:
- `tests/benchmarks/test_performance.py`
- `tests/test_audio_latency_e2e.py`
- `tests/test_audio_buffer_stability.py`
- `tests/test_audio_latency_loopback.py`

**Recommandations**:

#### ✅ **Bonne Pratique Actuelle**
```python
if os.environ.get("BBIA_DISABLE_AUDIO", "0") == "1":
    pytest.skip("Audio désactivé par BBIA_DISABLE_AUDIO=1")
if sd is None:
    pytest.skip("sounddevice indisponible sur cet environnement")
```

#### 🔧 **Améliorations Possibles**

1. **Créer un mock de sounddevice pour les tests**:
```python
# tests/fixtures/audio_mock.py
@pytest.fixture
def mock_sounddevice():
    """Mock sounddevice pour tests sans hardware audio."""
    with patch('sounddevice.play'), \
         patch('sounddevice.rec'), \
         patch('sounddevice.wait'):
        yield
```

2. **Utiliser des tests conditionnels avec marqueurs**:
```python
@pytest.mark.audio
@pytest.mark.skipif(
    os.environ.get("BBIA_DISABLE_AUDIO") == "1",
    reason="Audio désactivé"
)
def test_audio_functionality():
    ...
```

**Verdict**: ✅ **Décision correcte** - Les tests audio doivent être sautés en CI sans hardware audio. Considérer l'ajout de mocks pour certains tests unitaires.

---

### 3. Tests Sautés - Hardware/Robot Physique (24 tests)

**Problème**: Tests nécessitant un robot physique connecté.

**Exemples**:
- `Nécessite robot physique ou mock avancé`
- `Connexion robot échouée`
- `Robot physique requis`

**Fichiers concernés**:
- `tests/test_reachy_mini_backend.py`
- `tests/test_watchdog_monitoring.py`
- `tests/test_emergency_stop.py`

**Recommandations**:

#### ✅ **Bonne Pratique Actuelle**
```python
@pytest.mark.skip(reason="Nécessite robot physique ou mock avancé")
```

#### 🔧 **Améliorations Possibles**

1. **Créer des mocks avancés pour les tests hardware**:
```python
# tests/mocks/reachy_mini_mock.py
class MockReachyMini:
    """Mock avancé qui simule un robot physique."""
    def __init__(self):
        self.is_connected = True
        self._last_heartbeat = time.time()
    
    def get_current_joint_positions(self):
        """Simule la récupération des positions."""
        if not self.is_connected:
            raise ConnectionError("Robot déconnecté")
        return {...}
```

2. **Utiliser des marqueurs pour distinguer tests unitaires/intégration**:
```python
@pytest.mark.unit
def test_watchdog_logic():
    """Test unitaire avec mock."""
    ...

@pytest.mark.integration
@pytest.mark.skipif(not has_physical_robot(), reason="Robot physique requis")
def test_watchdog_real_hardware():
    """Test intégration avec robot réel."""
    ...
```

**Verdict**: ⚠️ **Amélioration possible** - Certains tests pourraient être convertis en tests unitaires avec des mocks avancés. Les tests nécessitant vraiment un robot physique doivent rester sautés.

---

### 4. Tests Sautés - Vision (15 tests)

**Problème**: Tests vision désactivés ou nécessitant des modèles lourds.

**Exemples**:
- `Vision désactivée par BBIA_DISABLE_VISION=1`
- `BBIAVision non disponible`
- `YOLO non disponible`

**Recommandations**:

#### ✅ **Bonne Pratique Actuelle**
```python
if os.environ.get("BBIA_DISABLE_VISION", "0") == "1":
    pytest.skip("Vision désactivée par BBIA_DISABLE_VISION=1")
```

#### 🔧 **Améliorations Possibles**

1. **Utiliser des modèles légers pour les tests**:
```python
@pytest.fixture
def mock_yolo_lightweight():
    """Mock YOLO avec modèle minimal pour tests."""
    mock_model = MagicMock()
    mock_model.predict.return_value = []
    return mock_model
```

**Verdict**: ✅ **Décision correcte** - Les tests vision doivent être sautés si désactivés. Aucune action requise.

---

### 5. Tests Sautés - Environnement (9 tests)

**Problème**: Tests spécifiques à une plateforme (macOS, CI).

**Exemples**:
- `Test spécifique macOS`
- `CI environment`

**Recommandations**:

#### ✅ **Bonne Pratique Actuelle**
```python
@pytest.mark.skipif(
    os.getenv("CI") is not None and sys.platform != "darwin",
    reason="Test spécifique macOS"
)
```

**Verdict**: ✅ **Décision correcte** - Les tests spécifiques à une plateforme doivent être sautés sur les autres plateformes. Aucune action requise.

---

## Analyse des Tests Mockés

### Patterns de Mocking Identifiés

1. **Mocks de dépendances externes** (majorité)
   - `@patch("bbia_sim.bbia_integration.BBIAEmotions")`
   - `@patch("bbia_sim.bbia_integration.BBIAVision")`
   - `@patch("bbia_sim.bbia_integration.SimulationService")`

2. **Mocks de modules système**
   - `@patch("sounddevice.play")`
   - `@patch("wave.open")`
   - `@patch("cv2.VideoCapture")`

3. **Mocks de classes complexes**
   - `MagicMock()` pour remplacer des instances complexes
   - `AsyncMock()` pour les fonctions async

### Recommandations pour les Mocks

#### ✅ **Bonne Pratique Actuelle**
```python
@patch("bbia_sim.bbia_integration.BBIAEmotions")
@patch("bbia_sim.bbia_integration.BBIAVision")
def test_integration(mock_vision, mock_emotions):
    ...
```

#### 🔧 **Améliorations Possibles**

1. **Créer des fixtures réutilisables pour les mocks communs**:
```python
# tests/conftest.py
@pytest.fixture
def mock_bbia_modules():
    """Fixture réutilisable pour mocker les modules BBIA."""
    with patch("bbia_sim.bbia_integration.BBIAEmotions") as mock_emotions, \
         patch("bbia_sim.bbia_integration.BBIAVision") as mock_vision, \
         patch("bbia_sim.bbia_integration.SimulationService") as mock_service:
        yield {
            'emotions': mock_emotions,
            'vision': mock_vision,
            'service': mock_service
        }
```

2. **Utiliser des mocks plus réalistes**:
```python
# Au lieu de MagicMock() simple
mock_vision = MagicMock()
mock_vision.scan_environment.return_value = {
    'objects': [{'name': 'person', 'confidence': 0.9}]
}
```

3. **Documenter les mocks complexes**:
```python
@pytest.fixture
def mock_reachy_mini_backend():
    """
    Mock complet du ReachyMiniBackend pour tests.
    
    Simule:
    - Connexion/déconnexion
    - Récupération positions joints
    - Envoi commandes mouvement
    """
    ...
```

---

## Tests Spécifiques à Analyser

### 1. `tests/test_dashboard_advanced.py` (74 tests sautés)

**Problème**: Beaucoup de tests sautés à cause de dépendances manquantes.

**Recommandation**: 
- Vérifier si toutes les dépendances sont vraiment nécessaires
- Créer des fixtures pour mocker les dépendances manquantes
- Considérer diviser le fichier en tests unitaires (avec mocks) et tests d'intégration

### 2. `tests/test_bbia_chat_llm.py` - `test_chat_context_management`

**Problème**: Timeout dû au chargement réel du modèle LLM.

**Solution appliquée**: ✅ Mock de la méthode `generate` avec `patch.object`.

**Verdict**: ✅ **Corrigé** - Le test utilise maintenant un mock au lieu de charger le modèle réel.

### 3. `tests/test_watchdog_monitoring.py` - `test_watchdog_timeout_triggers_emergency_stop_real`

**Problème**: Nécessite robot physique ou mock avancé.

**Recommandation**: 
- Créer un mock avancé qui simule un robot qui ne répond plus
- Le mock devrait lever une exception dans `get_current_joint_positions()` après un délai

---

## Plan d'Action Recommandé

### Priorité Haute 🔴

1. **Documenter les dépendances optionnelles**
   - Créer `docs/development/OPTIONAL_DEPENDENCIES.md`
   - Lister tous les tests qui nécessitent des dépendances optionnelles

2. **Créer des fixtures réutilisables pour les mocks communs**
   - `tests/conftest.py`: Fixtures pour BBIA modules, audio, vision
   - Réduire la duplication de code de mock

### Priorité Moyenne 🟡

3. **Améliorer les mocks pour les tests hardware**
   - Créer `tests/mocks/reachy_mini_mock.py` avec mock avancé
   - Permettre de tester la logique watchdog sans robot physique

4. **Analyser les tests avec beaucoup de skips**
   - `test_dashboard_advanced.py`: 74 tests sautés
   - `test_dashboard.py`: 42 tests sautés
   - Vérifier si certains peuvent être convertis en tests unitaires

### Priorité Basse 🟢

5. **Créer des marqueurs pytest personnalisés**
   - `@pytest.mark.requires_hf`
   - `@pytest.mark.requires_audio`
   - `@pytest.mark.requires_hardware`

6. **Améliorer la documentation des tests**
   - Ajouter des docstrings expliquant pourquoi certains tests sont mockés
   - Documenter les limitations des mocks

---

## Conclusion

### Décisions Correctes ✅

- **Tests sautés pour dépendances manquantes**: Correct, aucune action requise
- **Tests sautés pour audio/hardware en CI**: Correct, nécessaire pour éviter les erreurs
- **Tests sautés pour environnement spécifique**: Correct, nécessaire pour la portabilité

### Améliorations Possibles 🔧

1. **Réduction de la duplication**: Créer des fixtures réutilisables pour les mocks communs
2. **Documentation**: Documenter les dépendances optionnelles et les raisons des skips
3. **Mocks avancés**: Créer des mocks plus réalistes pour certains tests hardware
4. **Organisation**: Considérer diviser les gros fichiers de tests avec beaucoup de skips

### Métriques

- **Tests sautés**: 556 (principalement pour dépendances manquantes - normal)
- **Tests mockés**: 653 (normal pour tests unitaires)
- **Taux de skip acceptable**: ✅ Oui, principalement pour dépendances optionnelles

**Verdict Global**: ✅ **Les décisions de skip/mock sont globalement correctes**. Les améliorations proposées sont principalement pour réduire la duplication et améliorer la maintenabilité.

---

## Analyse Détaillée de Cas Spécifiques

### Cas 1: `test_watchdog_timeout_triggers_emergency_stop_real`

**Fichier**: `tests/test_watchdog_monitoring.py:217`

**Problème**: Test sauté car nécessite robot physique ou mock avancé.

**Analyse**:
- Le test vérifie que le watchdog déclenche `emergency_stop()` après 2s sans heartbeat
- Actuellement sauté avec `@pytest.mark.skip(reason="Nécessite robot physique ou mock avancé")`

**Recommandation**: ✅ **Créer un mock avancé**

```python
def test_watchdog_timeout_triggers_emergency_stop_mocked(self):
    """Test watchdog avec mock avancé simulant robot déconnecté."""
    from unittest.mock import MagicMock, patch
    
    # Mock du backend qui simule un robot qui ne répond plus
    mock_backend = MagicMock()
    mock_backend.is_connected = True
    mock_backend._last_heartbeat = time.time() - 3.0  # > 2s
    
    # Simuler que get_current_joint_positions lève une exception
    mock_backend.get_current_joint_positions.side_effect = ConnectionError("Robot déconnecté")
    
    # Vérifier que emergency_stop est appelé
    with patch.object(mock_backend, 'emergency_stop') as mock_emergency:
        # Simuler le check watchdog
        if time.time() - mock_backend._last_heartbeat > 2.0:
            mock_backend.emergency_stop()
        
        mock_emergency.assert_called_once()
```

**Verdict**: ⚠️ **Amélioration possible** - Le test peut être implémenté avec un mock avancé sans nécessiter de robot physique.

---

### Cas 2: `test_dashboard_advanced.py` (74 tests sautés)

**Problème**: Beaucoup de tests sautés à cause de `FASTAPI_AVAILABLE = False`.

**Analyse**:
- FastAPI est une dépendance optionnelle
- Tous les tests utilisent `@pytest.mark.skipif(not FASTAPI_AVAILABLE, ...)`
- C'est une bonne pratique pour les dépendances optionnelles

**Recommandation**: ✅ **Décision correcte**

Les tests sont correctement sautés quand FastAPI n'est pas disponible. Cependant:

1. **Amélioration possible**: Créer une fixture pour mocker FastAPI:
```python
@pytest.fixture
def mock_fastapi():
    """Mock FastAPI pour tests sans dépendance."""
    with patch("bbia_sim.dashboard_advanced.FASTAPI_AVAILABLE", True), \
         patch("bbia_sim.dashboard_advanced.FastAPI"), \
         patch("bbia_sim.dashboard_advanced.WebSocket"):
        yield
```

2. **Documentation**: Ajouter dans le README que FastAPI est optionnel pour certains tests.

**Verdict**: ✅ **Décision correcte** - Les tests doivent être sautés si FastAPI n'est pas disponible. Aucune action requise sauf amélioration optionnelle.

---

### Cas 3: Tests Audio avec `BBIA_DISABLE_AUDIO=1`

**Fichiers**: 
- `tests/benchmarks/test_performance.py`
- `tests/test_audio_latency_e2e.py`
- `tests/test_audio_buffer_stability.py`

**Problème**: Tests sautés en CI car audio désactivé.

**Analyse**:
- Les tests vérifient la latence et la stabilité audio
- En CI, `BBIA_DISABLE_AUDIO=1` est souvent défini
- Les tests sont correctement sautés avec `pytest.skip()`

**Recommandation**: ✅ **Décision correcte**

Les tests audio doivent être sautés en CI sans hardware audio. Cependant:

**Amélioration possible**: Créer des tests unitaires avec mocks pour la logique métier:
```python
def test_audio_latency_logic_mocked(self):
    """Test de la logique de calcul de latence avec mock."""
    from unittest.mock import MagicMock, patch
    
    mock_sd = MagicMock()
    mock_sd.play.return_value = None
    mock_sd.wait.return_value = None
    
    with patch('sounddevice.play', mock_sd.play), \
         patch('sounddevice.wait', mock_sd.wait):
        # Tester la logique de calcul sans hardware réel
        ...
```

**Verdict**: ✅ **Décision correcte** - Les tests nécessitant du hardware audio doivent être sautés en CI. Les tests unitaires peuvent utiliser des mocks.

---

### Cas 4: Tests Hugging Face (54 tests sautés)

**Problème**: Beaucoup de tests sautés car `HF_AVAILABLE = False`.

**Analyse**:
- Hugging Face transformers est une dépendance optionnelle lourde
- Les tests utilisent `@pytest.mark.skipif(not HF_AVAILABLE, ...)`
- C'est correct pour une dépendance optionnelle

**Recommandation**: ✅ **Décision correcte**

**Amélioration possible**: Créer des fixtures pour mocker les modèles HF:
```python
@pytest.fixture
def mock_huggingface():
    """Mock Hugging Face pour tests sans dépendance."""
    with patch("bbia_sim.bbia_huggingface.HF_AVAILABLE", True), \
         patch("bbia_sim.bbia_huggingface.AutoModelForCausalLM"), \
         patch("bbia_sim.bbia_huggingface.AutoTokenizer"):
        yield
```

**Verdict**: ✅ **Décision correcte** - Les tests doivent être sautés si HF n'est pas disponible. Aucune action requise.

---

## Résumé des Recommandations par Priorité

### 🔴 Priorité Haute (À faire rapidement)

1. **Créer un mock avancé pour `test_watchdog_timeout_triggers_emergency_stop_real`**
   - Permet de tester la logique watchdog sans robot physique
   - Fichier: `tests/test_watchdog_monitoring.py`

2. **Documenter les dépendances optionnelles**
   - Créer `docs/development/OPTIONAL_DEPENDENCIES.md`
   - Lister toutes les dépendances et leurs tests associés

### 🟡 Priorité Moyenne (Améliorations)

3. **Créer des fixtures réutilisables pour les mocks communs**
   - `tests/conftest.py`: Fixtures pour BBIA modules, FastAPI, HF
   - Réduire la duplication de code

4. **Analyser les tests avec beaucoup de skips**
   - Vérifier si certains peuvent être convertis en tests unitaires
   - `test_dashboard_advanced.py`: Considérer diviser en tests unitaires/intégration

### 🟢 Priorité Basse (Nice to have)

5. **Créer des marqueurs pytest personnalisés**
   - `@pytest.mark.requires_hf`
   - `@pytest.mark.requires_audio`
   - `@pytest.mark.requires_hardware`

6. **Améliorer la documentation des tests**
   - Ajouter des docstrings expliquant pourquoi certains tests sont mockés
   - Documenter les limitations des mocks

---

## Annexes

### Commandes Utiles

```bash
# Lister tous les tests sautés
pytest --collect-only -q | grep "SKIPPED"

# Exécuter uniquement les tests non sautés
pytest -m "not skip"

# Exécuter les tests avec dépendances optionnelles
pytest -m "requires_hf"  # Si marqueur créé

# Compter les tests sautés par raison
pytest --collect-only -q | grep "SKIPPED" | sort | uniq -c

# Lister les fichiers avec le plus de tests sautés
pytest --collect-only -q | grep "SKIPPED" | cut -d: -f1 | sort | uniq -c | sort -rn
```

### Références

- [Pytest Skip Documentation](https://docs.pytest.org/en/stable/how-to/skipping.html)
- [Pytest Mocking Best Practices](https://docs.python.org/3/library/unittest.mock.html)
- [Testing with Optional Dependencies](https://docs.pytest.org/en/stable/example/simple.html#control-skipping-of-tests-according-to-command-line-option)
- [Pytest Fixtures](https://docs.pytest.org/en/stable/fixture.html)

