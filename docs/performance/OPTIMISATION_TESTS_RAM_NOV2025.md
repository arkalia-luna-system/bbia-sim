# ⚡ Optimisation Tests RAM - Novembre 2025

**Date** : Oct 25 / Nov 25
**Objectif** : Réduire consommation RAM des tests (éviter surchauffe)

---

## 🎯 Problème

Les tests consomment trop de RAM car ils chargent de vrais modèles IA (YOLO, Whisper, HuggingFace) au lieu d'utiliser des mocks.

---

## ✅ Solutions Appliquées

### 1. Configuration Environnement Automatique

**Fichier** : `tests/conftest.py`

```python
# Force mode mock pour tests
os.environ.setdefault("BBIA_DISABLE_AUDIO", "1")
os.environ.setdefault("BBIA_DISABLE_VISION_MODELS", "1")
os.environ.setdefault("BBIA_FORCE_MOCK_MODELS", "1")
```

**Résultat** : Tests utilisent mocks par défaut, pas de chargement réel.

---

### 2. Nettoyage Caches Modèles

**Avant tests** :
- Nettoie cache YOLO (`_yolo_model_cache`)
- Nettoie cache MediaPipe (`_mediapipe_face_detection_cache`)
- Nettoie cache Whisper (`_whisper_models_cache`)
- Nettoie cache HuggingFace

**Après chaque test** :
- Force garbage collection Python (`gc.collect()`)

**Résultat** : RAM libérée entre tests.

---

### 3. Marqueurs Pytest pour Tests Lourds

**Fichier** : `pyproject.toml`

```toml
markers = [
    "slow: marque les tests lents",
    "fast: marque les tests rapides (mocks uniquement)",
    "model: marque les tests qui chargent de vrais modèles",
    "heavy: marque les tests très lourds en RAM/CPU",
]

# Par défaut, exclure tests lourds
addopts = [
    "-m", "not slow and not heavy",
]
```

**Résultat** : Tests lourds skip par défaut (sauf si `-m slow` ou `-m heavy`).

---

### 4. Fixtures Partagées Session

**Fichiers** : `tests/conftest.py`

```python
@pytest.fixture(scope="session")
def mock_yolo_detector():
    """Mock YOLO partagé (une seule instance pour toute la session)."""
    return MagicMock()

@pytest.fixture(scope="session")
def mock_whisper_stt():
    """Mock Whisper partagé (une seule instance)."""
    return MagicMock()
```

**Résultat** : Réutilise mêmes mocks, évite création multiple.

---

### 5. Tests Lourds Marqués

**Fichiers modifiés** :
- `tests/test_model_memory_management.py` : `@pytest.mark.heavy` ajouté
- `tests/test_huggingface_latency.py` : `@pytest.mark.heavy` ajouté

**Résultat** : Tests qui chargent vrais modèles skip par défaut.

---

## 🚀 Utilisation

### Tests Rapides (Par Défaut)

```bash
# Lance uniquement tests rapides (mocks)
pytest tests/

# Équivaut à :
pytest tests/ -m "not slow and not heavy"
```

### Tests Complets (Si Besoin)

```bash
# Inclure tests lourds
pytest tests/ -m "slow or heavy"

# Ou tout lancer
pytest tests/ -m ""
```

---

## 📊 Impact

| Aspect | Avant | Après |
|--------|-------|-------|
| **RAM par test** | ~500-2000 MB | ~50-100 MB |
| **Tests rapides** | Tous | Tests sans `@pytest.mark.heavy` |
| **Cache modèles** | Pas nettoyé | Nettoyé avant/après tests |
| **Fixtures** | Créées à chaque test | Session-scoped |

---

## 🎯 Recommandations

1. **Développement quotidien** : Utiliser `pytest tests/` (tests rapides uniquement)
2. **CI/CD** : Peut inclure `-m slow` si nécessaire
3. **Tests spécifiques** : `pytest tests/test_vision_yolo_comprehensive.py` (utilise mocks)
4. **Benchmarks** : Utiliser `-m heavy` explicitement quand nécessaire

---

**Projet optimisé pour réduire consommation RAM !** 🚀

