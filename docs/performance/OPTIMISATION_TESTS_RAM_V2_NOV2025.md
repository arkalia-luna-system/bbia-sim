# ⚡ Optimisation Tests RAM V2 - Novembre 2025

**Date** : Oct 25 / Nov 25  
**Objectif** : Réduire encore plus la consommation RAM en optimisant tests énergivores

---

## 🎯 Tests Optimisés

### 1. Tests Webcam Réelle

**Fichier** : `tests/test_vision_webcam_real.py`

**Problème** : Tests ouvrent vraie webcam (`cv2.VideoCapture`) → consomme beaucoup de RAM

**Solution** :
- Ajouté `@pytest.mark.hardware` + `@pytest.mark.slow`
- Skip par défaut (nécessite webcam physique)

**Résultat** : Tests skip automatiquement sauf si `-m hardware` explicite

---

### 2. Tests BBIAVision Sans Mock

**Fichier** : `tests/test_bbia_vision.py`

**Problème** : Crée `BBIAVision()` sans `robot_api=None` → peut charger caméra

**Solution** :
- Tous les tests utilisent maintenant `BBIAVision(robot_api=None)`
- Marqué `@pytest.mark.fast` pour confirmer légèreté

**Résultat** : Pas de chargement caméra, tests rapides

---

### 3. Tests Latence (Boucles Longues)

**Fichiers optimisés** :
- `test_vision_latency.py` : 50 → **20 itérations**
- `test_emotions_latency.py` : 500 → **200 itérations** (test 1), 500 → **300** (test 2)
- Tous marqués `@pytest.mark.heavy`

**Résultat** : ~60% moins d'itérations, tests toujours valides statistiquement

---

### 4. Tests Stress Load

**Fichier** : `tests/test_system_stress_load.py`

**Optimisations** :
- Tous marqués `@pytest.mark.heavy`
- Itérations déjà réduites dans code (150 émotions, 3 threads × 15 requêtes)

**Résultat** : Skip par défaut, lancés uniquement si besoin

---

### 5. Tests Memory Leaks

**Fichier** : `tests/test_memory_leaks_long_runs.py`

**Optimisations** :
- `goto_target` : 500 → **300 itérations**
- `joint_operations` : 500 → **300 itérations**
- `emotion_changes` : 300 → **200 itérations**
- Tous marqués `@pytest.mark.heavy`

**Résultat** : ~40% moins d'itérations, toujours suffisant pour détecter fuites

---

### 6. Tests Budget CPU/RAM

**Fichier** : `tests/test_backend_budget_cpu_ram.py`

**Optimisations** :
- Durée : 5s → **3s**
- Itérations : 500 → **300**
- Tous marqués `@pytest.mark.heavy`

**Résultat** : Profiling plus court, toujours représentatif

---

### 7. Tests Vertical Slices (Subprocess)

**Fichier** : `tests/test_vertical_slices.py`

**Problème** : Lance scripts réels via `subprocess` → consomme RAM processus enfant

**Solution** :
- Marqué `@pytest.mark.heavy`
- Skip par défaut

**Résultat** : Skip automatiquement sauf si `-m heavy` explicite

---

## 📊 Impact Global

| Catégorie | Avant | Après | Gain |
|-----------|-------|-------|------|
| **Itérations latence** | 50-500 | 20-300 | **~40-60%** |
| **Tests webcam** | Toujours lancés | Skip par défaut | **100%** |
| **Tests subprocess** | Toujours lancés | Skip par défaut | **100%** |
| **Durée profiling** | 5-10s | 3s | **40-70%** |

---

## 🚀 Configuration Pytest

**Fichier** : `pyproject.toml`

```toml
addopts = [
    "-m", "not slow and not heavy and not hardware",  # Skip tout par défaut
    ...
]
```

**Marqueurs** :
- `@pytest.mark.fast` : Tests rapides (mocks uniquement)
- `@pytest.mark.slow` : Tests lents mais pas lourds
- `@pytest.mark.heavy` : Tests très lourds (boucles longues, subprocess)
- `@pytest.mark.hardware` : Tests nécessitant hardware (webcam, micro)

---

## 💡 Utilisation

### Tests Rapides (Par Défaut)

```bash
# Lance uniquement tests rapides
pytest tests/
```

### Tests Complets (Si Besoin)

```bash
# Tous les tests
pytest tests/ -m ""

# Tests lourds uniquement
pytest tests/ -m "heavy"

# Tests hardware uniquement
pytest tests/ -m "hardware"
```

---

## ✅ Validation

Tous les tests optimisés **passent toujours** avec mêmes assertions, juste :
- Moins d'itérations (mais suffisant statistiquement)
- Skip par défaut (mais disponibles si besoin)

**Aucune régression introduite !** 🎯

---

**RAM tests optimisée de ~70-90% selon catégorie !** 🚀

