# 📊 Analyse Détaillée Coverage - Problèmes Identifiés

**Date** : Janvier 2025  
**Objectif** : Identifier pourquoi certains modules ont un coverage faible malgré des tests existants

---

## 🔍 PROBLÈMES IDENTIFIÉS

### 1. **Modules non importés dans les tests** ❌

#### `model_optimizer.py` : 39.13% (14 lignes manquantes)
**Problème** : Coverage warning : `Module src/bbia_sim/model_optimizer was never imported`

**Solution** :
- Le module n'est jamais importé dans les tests
- Il faut créer `tests/test_model_optimizer.py` et importer le module directement
- Tester : `get_cached_model()`, `clear_model_cache()`, `get_cache_size()`, `lazy_load_model()`

**Fichier à créer** : `tests/test_model_optimizer.py`

---

### 2. **Tests existants mais coverage partiel** ⚠️

#### `__main__.py` : 22.50% (124 lignes manquantes sur 160)
**Tests existants** : `tests/test_main.py` (4 tests)

**Problème** : Les tests ne couvrent que :
- ✅ `setup_logging()` (ligne 15-27)
- ✅ `run_awake_sequence()` (ligne 29-42)
- ✅ `run_voice_synthesis()` (ligne 44-57)
- ✅ `run_voice_recognition()` (ligne 59-72)

**Lignes non couvertes** :
- ❌ Fonction `main()` (lignes 29-314) - **FONCTION PRINCIPALE NON TESTÉE**
- ❌ Arguments parser (lignes 31-110)
- ❌ Gestion simulation (lignes 112-177)
- ❌ Gestion erreurs

**Solution** : Ajouter tests pour `main()` avec mocks argparse

---

#### `bbia_awake.py` : 20.0% (12 lignes manquantes sur 15)
**Tests existants** : `tests/test_bbia_awake.py`, `tests/test_bbia_awake_extended.py`

**Problème** : Tests utilisent `subprocess` (exécution externe) au lieu d'importer directement le module

**Solution** : Importer directement `bbia_awake` dans les tests pour que coverage le détecte

---

#### `bbia_integration.py` : 20.1% (199 lignes manquantes sur 249)
**Tests existants** : `tests/test_bbia_integration.py`, `tests/test_bbia_integration_rapid.py`

**Problème** : Tests vérifient seulement l'existence des classes/méthodes, pas leur exécution

**Solution** : Ajouter tests qui appellent réellement les méthodes avec mocks

---

### 3. **Modules sans tests** ❌

#### `daemon/app/__main__.py` : 0.0% (8 lignes)
**Problème** : Aucun test

**Solution** : Créer `tests/test_daemon_app_main.py` pour tester le point d'entrée

---

#### `daemon/app/routers/sanity.py` : 32.43% (25 lignes manquantes sur 37)
**Problème** : Aucun test

**Solution** : Créer `tests/test_sanity_router.py` pour tester les routes FastAPI :
- `GET /api/sanity/status`
- `POST /api/sanity/emergency_stop`

---

#### `face_recognition.py` : 25.18% (104 lignes manquantes sur 139)
**Problème** : Aucun test dédié

**Solution** : Créer `tests/test_face_recognition.py`

---

#### `dashboard.py` : 31.29% (101 lignes manquantes sur 147)
**Problème** : Il y a `test_dashboard_advanced.py` mais pas de test pour `dashboard.py` (module différent)

**Solution** : Créer `tests/test_dashboard.py` pour tester le module `dashboard.py`

---

#### `backends/reachy_backend.py` : 30.8% (135 lignes manquantes sur 195)
**Problème** : Tests existants mais coverage faible

**Solution** : Vérifier pourquoi les tests ne couvrent pas tout (peut-être imports conditionnels)

---

## 📋 PLAN D'ACTION

### Priorité Haute (Modules < 30%)

1. **`model_optimizer.py`** (39.13% → 70%+)
   - ✅ Créer `tests/test_model_optimizer.py`
   - ✅ Importer directement le module
   - ✅ Tester toutes les fonctions

2. **`daemon/app/__main__.py`** (0% → 70%+)
   - ✅ Créer `tests/test_daemon_app_main.py`
   - ✅ Tester le point d'entrée avec mocks uvicorn

3. **`daemon/app/routers/sanity.py`** (32.43% → 70%+)
   - ✅ Créer `tests/test_sanity_router.py`
   - ✅ Tester routes FastAPI avec TestClient

4. **`__main__.py`** (22.50% → 70%+)
   - ✅ Améliorer `tests/test_main.py`
   - ✅ Ajouter tests pour fonction `main()` avec mocks argparse

5. **`bbia_awake.py`** (20.0% → 70%+)
   - ✅ Modifier tests pour importer directement le module
   - ✅ Ajouter tests unitaires (pas seulement subprocess)

6. **`bbia_integration.py`** (20.1% → 70%+)
   - ✅ Améliorer tests existants pour appeler réellement les méthodes
   - ✅ Ajouter tests avec mocks complets

7. **`face_recognition.py`** (25.18% → 70%+)
   - ✅ Créer `tests/test_face_recognition.py`

8. **`dashboard.py`** (31.29% → 70%+)
   - ✅ Créer `tests/test_dashboard.py`

9. **`backends/reachy_backend.py`** (30.8% → 70%+)
   - ✅ Analyser pourquoi coverage faible malgré tests
   - ✅ Améliorer tests existants

---

## ✅ RÉSUMÉ

**Problèmes principaux** :
1. Modules non importés dans les tests → coverage ne les détecte pas
2. Tests utilisent subprocess au lieu d'imports directs
3. Tests vérifient seulement l'existence, pas l'exécution
4. Modules sans tests dédiés

**Solution générale** :
- Importer directement les modules dans les tests (pas subprocess)
- Tester réellement l'exécution (pas seulement l'existence)
- Créer tests manquants pour modules sans coverage

---

**Dernière mise à jour** : Janvier 2025

