# 📊 Analyse Détaillée Coverage - Problèmes Identifiés

**Date** : Janvier 2025  
**Objectif** : Identifier pourquoi certains modules ont un coverage faible malgré des tests existants

---

## 🔍 PROBLÈMES IDENTIFIÉS

### 1. **Modules non importés dans les tests** ❌

#### `model_optimizer.py` : ✅ **100%** (corrigé - Nov 2025)
**Problème** : Coverage warning : `Module src/bbia_sim/model_optimizer was never imported`

**Solution appliquée** :
- ✅ Créé `tests/test_model_optimizer.py` avec import direct du module
- ✅ Tous les tests passent (9 tests)
- ✅ Coverage : **100%** (toutes les fonctions testées)

**Fichier créé** : `tests/test_model_optimizer.py` ✅

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

#### `daemon/app/__main__.py` : ✅ **87.50%** (corrigé - Nov 2025)
**Problème** : Aucun test

**Solution appliquée** :
- ✅ Coverage amélioré via tests existants et imports directs
- ✅ Coverage : **87.50%** (7 lignes sur 8 couvertes)

---

#### `daemon/app/routers/sanity.py` : ✅ **89.19%** (corrigé - Nov 2025)
**Problème** : Aucun test

**Solution appliquée** :
- ✅ Créé `tests/test_sanity_router.py` avec 7 tests complets
- ✅ Toutes les routes FastAPI testées : `GET /api/sanity/status`, `POST /api/sanity/emergency_stop`
- ✅ Coverage : **89.19%** (33 lignes sur 37 couvertes)

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

1. **`model_optimizer.py`** ✅ **TERMINÉ** (39.13% → **100%** - Nov 2025)
   - ✅ Créé `tests/test_model_optimizer.py` (9 tests)
   - ✅ Import direct du module
   - ✅ Toutes les fonctions testées

2. **`daemon/app/__main__.py`** ✅ **TERMINÉ** (0% → **87.50%** - Nov 2025)
   - ✅ Coverage amélioré via tests et imports directs
   - ✅ 7 lignes sur 8 couvertes

3. **`daemon/app/routers/sanity.py`** ✅ **TERMINÉ** (32.43% → **89.19%** - Nov 2025)
   - ✅ Créé `tests/test_sanity_router.py` (7 tests)
   - ✅ Routes FastAPI testées avec mocks

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

**Dernière mise à jour** : Novembre 2025

## ✅ CORRECTIONS APPLIQUÉES (Nov 2025)

### Modules corrigés
- ✅ `model_optimizer.py` : **100%** coverage (9 tests créés)
- ✅ `daemon/app/routers/sanity.py` : **89.19%** coverage (7 tests créés)
- ✅ `daemon/app/__main__.py` : **87.50%** coverage (amélioré via imports directs)

### Tests créés
- ✅ `tests/test_model_optimizer.py` - 9 tests, coverage 100%
- ✅ `tests/test_sanity_router.py` - 7 tests, coverage 89.19%

### Qualité code
- ✅ Black : formatage OK
- ✅ Ruff : 0 erreurs
- ✅ MyPy : 0 erreurs
- ✅ Bandit : 0 erreurs

