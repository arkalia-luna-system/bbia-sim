# 🎯 PROMPT D'AUDIT EXHAUSTIF - BBIA-SIM ↔ REACHY MINI SDK

## [Vérification stricte et automatisée, venv actif]

---

## 📋 CONTEXTE PRÉ-EXISTANT (À UTILISER)

### ✅ Informations Déjà Découvertes

**Chemins locaux**:
- **BBIA-SIM**: `/Volumes/T7/bbia-reachy-sim` (branche `future`)
- **Repo Officiel**: `/Volumes/T7/reachy_mini` (branche `develop`)

**Outils existants**:
- **Script comparaison principal**: `scripts/compare_with_official_exhaustive.py` ✅ (consolidé)
  - Usage: `python scripts/compare_with_official_exhaustive.py --official-root /Volumes/T7/reachy_mini`
  - Génère: `logs/comparison_official_results.json` et `logs/comparison_official_report.md`
  - **Améliorations** : Intègre fonctionnalités de scripts archivés (méthodes, backend, assets, docstrings)
- **Script alignement MJCF**: `scripts/check_official_alignment.py` ✅
  - Usage: `python scripts/check_official_alignment.py`
- **Script génération rapports**: `scripts/generate_conformity_report_reachy_mini.py` ✅
  - Usage: `python scripts/generate_conformity_report_reachy_mini.py`
 
**Note** : `audit_systematique_exhaustif.py` archivé dans `scripts/_archived/comparison_audit/` (fonctionnalités intégrées dans script principal)
- **Checklists finales**:
  - `docs/conformite/CHECKLIST_FINALE_COMPARAISON_OFFICIELLE.md` (177 différences détectées)
  - `docs/conformite/CHECKLIST_AUDIT_EXHAUSTIF.md` (audit systématique joints MuJoCo, tests, assets STL)
- **Rapports précédents**: `logs/comparison_official_results.json` (177 différences détectées)

**Endpoints déjà corrigés** (à vérifier conformité):
- ✅ `GET /api/move/recorded-move-datasets/list/{dataset_name:path}` → `src/bbia_sim/daemon/app/routers/move.py:189`
- ✅ `POST /api/move/play/recorded-move-dataset/{dataset_name:path}/{move_name}` → `src/bbia_sim/daemon/app/routers/move.py:207`
- ✅ `GET /api/kinematics/stl/{filename}` → `src/bbia_sim/daemon/app/routers/kinematics.py:119` (utilise `{filename:path}` - compatible)
- ✅ **Router `/api/move/*` complet** : `POST /goto`, `GET /running`, `POST /stop`, `POST /play/wake_up`, `POST /play/goto_sleep`, `POST /set_target`, `WebSocket /ws/updates`, `WebSocket /ws/set_target` → `src/bbia_sim/daemon/app/routers/move.py`
- ✅ **Router `/api/state/*` amélioré** : `GET /full` (11 paramètres), `GET /present_head_pose`, `GET /present_body_yaw`, `GET /present_antenna_joint_positions`, `WebSocket /ws/full` → `src/bbia_sim/daemon/app/routers/state.py`
- ✅ **BackendAdapter créé** : Adapte `RobotAPI` (BBIA) vers `Backend` (SDK) → `src/bbia_sim/daemon/app/backend_adapter.py`

**Structure routers officiels**:
- `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/`:
  - `move.py`, `state.py`, `motors.py`, `daemon.py`, `kinematics.py`, `apps.py`

**Structure routers BBIA**:
- `src/bbia_sim/daemon/app/routers/`:
  - `move.py`, `state.py`, `motion.py`, `motors.py`, `daemon.py`, `kinematics.py`, `apps.py`, `ecosystem.py`, `sanity.py`

**Tests existants BBIA** (168 tests):
- `tests/test_reachy_mini_backend.py` (24 tests)
- `tests/test_reachy_mini_full_conformity_official.py` (37 tests)
- `tests/test_reachy_mini_strict_conformity.py`
- Et 165+ autres...

**Modèles MuJoCo**:
- Officiel: `/Volumes/T7/reachy_mini/src/reachy_mini/descriptions/reachy_mini/mjcf/reachy_mini.xml`
- BBIA: `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`
- Assets STL: `src/bbia_sim/sim/assets/reachy_official/` (41 fichiers STL)

**Backend SDK**:
- BBIA: `src/bbia_sim/backends/reachy_mini_backend.py`
- Utilise: `from reachy_mini import ReachyMini` (SDK officiel)
- Méthodes critiques: `goto_target()`, `look_at_world()`, `play_move()`, `async_play_move()`

---

## 🚫 CONTRAINTES STRICTES

1. **NE JAMAIS re-cloner ni télécharger** le repo officiel (déjà présent à `/Volumes/T7/reachy_mini`)
2. **NE JAMAIS créer de nouveaux fichiers Markdown** sans vérifier d'abord les MD existants
3. **RÉUTILISER/MODIFIER** les MD existants dans `docs/conformite/`, `docs/audit/`, `docs/guides/`
4. **NE PAS exécuter tous les tests en parallèle** (PC peu puissant) - UNIQUEMENT les tests liés à chaque correction
5. **Valider immédiatement** chaque correction: black/ruff/mypy/bandit + test spécifique
6. **NE JAMAIS modifier les dates** (logs/commits/timestamps existants)

---

## 🎯 TÂCHE PRINCIPALE

> Pour chaque **module/fichier/fonction** de BBIA-SIM (`src/bbia_sim`, `scripts/`, `tests/`, `docs/`, `assets/`), compare **absolument TOUS** les endpoints REST, classes, scripts, outils d'intégration, modèles MuJoCo, helpers, guides et tests d'intégration à leur équivalent dans le repo officiel (`/Volumes/T7/reachy_mini/src/reachy_mini`, `tests/`, `docs/`, etc.).

> **Objectif**: Chaque action du SDK ou daemon officiel doit être **strictement reproductible** avec BBIA-SIM et **immédiatement compatible** pour developer/tester le Reachy Mini **PHYSIQUE**, sans bugs ni comportements non conformes.

---

## 🔍 CHECKLIST DE DÉTECTION (Pour chaque différence)

### 1. **Incohérences d'API REST**
- **Chemins**: Comparer `/api/move/goto` vs `/api/motion/goto_pose`
- **Noms**: Comparer `GotoModelRequest` vs `Pose` model
- **Arguments**: Comparer types, valeurs par défaut, required/optional
- **Retours**: Comparer structures de réponse, codes HTTP
- **Erreurs**: Comparer exceptions, messages d'erreur, codes de statut
- **Files**: `src/bbia_sim/daemon/app/routers/*.py` vs `/Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/*.py`

### 2. **Divergences de comportement**
- **Ranges**: Comparer limites joints (`joint_limits` dans backends)
- **Clamps**: Comparer logique de clamp (min/max values)
- **Logs**: Comparer niveaux, formats, messages
- **Exceptions**: Comparer types d'exceptions, chaînage (`raise ... from`)
- **Formats**: Comparer formats de valeurs (float vs int, radians vs degrees)
- **Synchronisation**: Comparer comportement async/sync

### 3. **Modules/Endpoints/Helpers manquants**
- **BBIA vs SDK**: Endpoints officiels absents dans BBIA
- **SDK vs BBIA**: Extensions BBIA légitimes (marquer comme INFO)
- **Helpers**: Comparer `src/bbia_sim/utils/` vs `/Volumes/T7/reachy_mini/src/reachy_mini/utils/`
- **Scripts**: Comparer `scripts/` BBIA vs exemples officiels

### 4. **Incohérences modèles MuJoCo/STL**
- **Joints**: Comparer noms, ranges, types dans XML
- **Assets**: Comparer fichiers STL (41 fichiers attendus)
- **Structure**: Comparer hiérarchie des corps, géométries, actuateurs
- **Orientation**: Comparer quaternions, matrices de transformation
- **Files**:
  - BBIA: `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`
  - Officiel: `/Volumes/T7/reachy_mini/src/reachy_mini/descriptions/reachy_mini/mjcf/reachy_mini.xml`

### 5. **Disparités tests/couverture**
- **Tests critiques manquants**: `test_daemon.py`, `test_collision.py`, `test_analytical_kinematics.py`
- **Tests existants**: Comparer 168 tests BBIA vs 22 tests officiels
- **Couverture**: Vérifier connexion, motion, head, camera, motors
- **Files**: `tests/test_reachy_mini_*.py` vs `/Volumes/T7/reachy_mini/tests/test_*.py`

### 6. **Documentation**
- **Warnings**: Comparer sections sécurité, onboarding
- **README**: Comparer sections Usage, Installation, API
- **Troubleshooting**: Comparer guides de résolution problèmes
- **Files**: `README.md`, `docs/guides/`, `docs/conformite/`

### 7. **Exemples/Scripts/Démos**
- **Exemples**: Comparer `examples/` BBIA vs `/Volumes/T7/reachy_mini/examples/`
- **Scripts**: Vérifier utilitaires, helpers, ML tools
- **Démos**: Comparer démos fonctionnelles

### 8. **Code style & qualité**
- **Black**: `black --check src/bbia_sim/daemon/app/routers/move.py`
- **Ruff**: `ruff check src/bbia_sim/daemon/app/routers/move.py`
- **Mypy**: `mypy src/bbia_sim/daemon/app/routers/move.py` (ignorer imports conditionnels OK)
- **Bandit**: `bandit -r src/bbia_sim/daemon/app/routers/move.py --skip B101`

### 9. **Doublons/Fichiers obsolètes**
- **Doublons**: Identifier fichiers similaires (`test_*.py` multiples)
- **Obsolètes**: Identifier fichiers non utilisés, imports morts
- **Structure**: Nettoyer structure, supprimer fichiers inutiles

### 10. **Dates (NE PAS MODIFIER)**
- **Logs**: Ne pas modifier timestamps dans logs/
- **Commits**: Ne pas modifier dates de commits Git
- **Files**: Ne pas modifier mtime des fichiers

---

## 🔂 PROCÉDURE DÉTAILLÉE (Pour chaque écart détecté)

### Étape 1: Détection
1. **Utiliser les scripts existants**:
   ```bash
   # Comparaison exhaustive endpoints/classes
   python scripts/compare_with_official_exhaustive.py \
     --bbia-root /Volumes/T7/bbia-reachy-sim \
     --official-root /Volumes/T7/reachy_mini \
     --output-dir logs
   
   # Vérifier alignement MJCF/STL
   python scripts/check_official_alignment.py
   
   # Générer rapport conformité depuis tests
   python scripts/generate_conformity_report_reachy_mini.py
   ```
2. **Lire les résultats**:
   - `logs/comparison_official_results.json` (endpoints/classes)
   - `docs/conformite/CHECKLIST_AUDIT_EXHAUSTIF.md` (joints/tests/assets)
3. **Consulter les checklists**:
   - `docs/conformite/CHECKLIST_FINALE_COMPARAISON_OFFICIELLE.md` (endpoints REST)
   - `docs/conformite/CHECKLIST_AUDIT_EXHAUSTIF.md` (modèles MuJoCo, tests, assets)

### Étape 2: Documentation
- **Fichier**: Chemin relatif depuis racine BBIA
- **Ligne**: Numéro de ligne exact (si applicable)
- **Endpoint**: Méthode HTTP + chemin complet
- **Différence**: Description précise
- **Correction proposée**: Code ou action spécifique

### Étape 3: Correction
1. **Éditer le fichier** concerné
2. **Implémenter** la correction conforme SDK
3. **Vérifier imports**: Ajouter imports nécessaires (`from reachy_mini.motion.recorded_move import RecordedMoves`)
4. **Gérer exceptions**: Utiliser `raise ... from e` ou `raise ... from None`

### Étape 4: Validation immédiate
```bash
# 1. Formatage
black src/bbia_sim/daemon/app/routers/[fichier].py

# 2. Linting
ruff check src/bbia_sim/daemon/app/routers/[fichier].py

# 3. Type checking (si nécessaire)
mypy src/bbia_sim/daemon/app/routers/[fichier].py --ignore-missing-imports

# 4. Security (optionnel)
bandit -r src/bbia_sim/daemon/app/routers/[fichier].py --skip B101
```

### Étape 5: Test spécifique
```bash
# Exécuter UNIQUEMENT le test lié à la correction
pytest tests/test_[nom_test].py::test_[fonction_specifique] -v

# Si test n'existe pas, créer test minimal
# File: tests/test_api_[endpoint].py
```

### Étape 6: Mise à jour checklist
- **Ouvrir checklist appropriée**:
  - **API/Classes**: `docs/conformite/CHECKLIST_FINALE_COMPARAISON_OFFICIELLE.md`
  - **Modèles/Tests/Assets**: `docs/conformite/CHECKLIST_AUDIT_EXHAUSTIF.md`
- **Ajouter entrée**:
  ```markdown
  - [x] **API** - Endpoint `GET /api/move/recorded-move-datasets/list/{dataset_name:path}`
    - Fichier: `src/bbia_sim/daemon/app/routers/move.py:184`
    - Correction: ✅ Implémenté avec RecordedMoves SDK
    - Test: ⚠️ À tester avec dataset réel
    - Statut: ✅ CORRIGÉ
  ```

---

## 🧠 OPTIMISATIONS & BONNES PRATIQUES

### Performance
- **Modulariser**: Traiter un endpoint/classe à la fois
- **Batcher**: Grouper corrections similaires (tous les endpoints `move` ensemble)
- **RAM/CPU**: Ne pas charger tous les fichiers en mémoire d'un coup

### Structure
- **Ranger**: Tous les exemples/démos dans `examples/`
- **Tester**: Chaque script doit être exécutable et testé
- **Nettoyer**: Supprimer fichiers inutilisés, doublons

### Documentation
- **Réutiliser**: Modifier MD existants dans `docs/conformite/`, `docs/audit/`
- **NE PAS créer doublons**: Vérifier avant de créer nouveau MD
- **Mettre à jour**: Checklist finale au lieu de créer nouveaux rapports

### Tests
- **Granulaire**: UN test à la fois, pas toute la suite
- **Spécifique**: Test directement lié à la correction
- **Minimal**: Créer tests minimaux pour nouvelles fonctionnalités

---

## 🟢 CHECKLIST EXPORT FINALE

Pour **CHAQUE** erreur/correction/évolution détectée, documenter dans la checklist appropriée:
- **API/Classes/Endpoints**: `docs/conformite/CHECKLIST_FINALE_COMPARAISON_OFFICIELLE.md`
- **Modèles MuJoCo/Tests/Assets**: `docs/conformite/CHECKLIST_AUDIT_EXHAUSTIF.md`

```markdown
### [Priorité] - [Nature]

- [ ] **[Nature]** - [Description précise]
  - **Fichier**: `chemin/relatif/fichier.py:ligne`
  - **Endpoint**: `METHOD /api/path` (si applicable)
  - **Différence**: [Description]
  - **BBIA**: [Valeur/état actuel]
  - **Officiel**: [Valeur/état attendu]
  - **Correction**: [Code ou action]
  - **Test**: [Test associé ou "À créer"]
  - **Statut**: ✅ CORRIGÉ / ⚠️ PENDING / ❌ SKIPPÉ (raison)
```

**Natures possibles**:
- `API` - Endpoint REST
- `Class` - Classe Python
- `Model` - Modèle MuJoCo/STL
- `Test` - Test unitaire/intégration
- `Doc` - Documentation
- `Security` - Sécurité
- `Script` - Script utilitaire
- `Helper` - Helper function
- `Config` - Configuration

**Priorités**:
- `CRITICAL` - Bloque compatibilité robot réel
- `HIGH` - Important pour conformité SDK
- `MEDIUM` - À évaluer selon besoins
- `LOW` - Amélioration non critique
- `INFO` - Information (extensions BBIA légitimes)

---

## 📊 EXEMPLE DE WORKFLOW COMPLET

### Cas: Endpoint recorded-move manquant

1. **Détection** (script automatique):
   ```json
   {
     "category": "API",
     "endpoint": "GET /recorded-move-datasets/list/{dataset_name:path}",
     "severity": "HIGH",
     "status": "pending"
   }
   ```

2. **Recherche équivalent officiel**:
   ```bash
   # Lire fichier officiel
   cat /Volumes/T7/reachy_mini/src/reachy_mini/daemon/app/routers/move.py | grep -A 10 "recorded-move-datasets"
   ```

3. **Implémentation**:
   ```python
   # src/bbia_sim/daemon/app/routers/move.py
   @router.get("/recorded-move-datasets/list/{dataset_name:path}")
   async def list_recorded_move_dataset(dataset_name: str) -> list[str]:
       """Conforme SDK officiel."""
       try:
           from reachy_mini.motion.recorded_move import RecordedMoves
           moves = RecordedMoves(dataset_name)
           return moves.list_moves()
       except ImportError:
           raise HTTPException(status_code=501, detail="RecordedMoves non disponible") from None
       except RepositoryNotFoundError as e:
           raise HTTPException(status_code=404, detail=str(e)) from e
   ```

4. **Validation**:
   ```bash
   black src/bbia_sim/daemon/app/routers/move.py
   ruff check src/bbia_sim/daemon/app/routers/move.py
   ```

5. **Test** (à créer):
   ```python
   # tests/test_api_recorded_moves.py
   def test_list_recorded_move_dataset():
       # Test avec dataset mock
   ```

6. **Checklist**:
   ```markdown
   - [x] **API** - Endpoint `GET /api/move/recorded-move-datasets/list/{dataset_name:path}`
     - Fichier: `src/bbia_sim/daemon/app/routers/move.py:184`
     - Correction: ✅ Implémenté
     - Test: ⚠️ À tester avec dataset réel
     - Statut: ✅ CORRIGÉ
   ```

---

## 🎯 COMMANDES RAPIDES DE RÉFÉRENCE

```bash
# Comparaison exhaustive
python scripts/compare_with_official_exhaustive.py

# Formatage
black src/bbia_sim/daemon/app/routers/*.py

# Linting
ruff check src/bbia_sim/daemon/app/routers/*.py

# Test spécifique
pytest tests/test_reachy_mini_backend.py::test_specific_function -v

# Vérifier endpoint
curl -X GET http://localhost:8000/api/move/recorded-move-datasets/list/dataset_name

# Lire rapport
cat logs/comparison_official_report.md

# Checklists
cat docs/conformite/CHECKLIST_FINALE_COMPARAISON_OFFICIELLE.md  # Endpoints/API
cat docs/conformite/CHECKLIST_AUDIT_EXHAUSTIF.md  # Joints/Tests/Assets
```

---

## ✅ VALIDATION FINALE

Avant de considérer l'audit complet:

1. ✅ **Tous les endpoints HIGH corrigés** (0 pending)
2. ✅ **Code formaté** (black pass)
3. ✅ **Linting OK** (ruff pass)
4. ✅ **Tests critiques passent** (pytest)
5. ✅ **Checklist mise à jour** (MD existant modifié)
6. ✅ **Pas de doublons MD créés**
7. ✅ **Structure propre** (fichiers obsolètes supprimés)

---

**Version**: 2.0
**Date**: Oct / Nov. 2025
**Basé sur**: Script `compare_with_official_exhaustive.py` (consolidé) et checklist existante

