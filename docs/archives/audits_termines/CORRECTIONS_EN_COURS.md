---
**⚠️ ARCHIVE - DOCUMENT HISTORIQUE ⚠️**

Ce document a été archivé car il est devenu obsolète ou a été remplacé par une version plus récente.
Il est conservé à des fins de référence historique uniquement.

**Date d'archivage** : Oct / No2025025025025025
**Raison** : Document terminé/obsolète/remplacé
---

# 🔧 Corrections En Cours - Tests Échoués

**Date**: Oct / No2025025025025025  
**Stratégie**: Méthodique, sans régressions

---

## ✅ Phase 1: Tests Mapping (SIMPLE) - EN COURS

### ✅ 1. `test_global_config.py::test_validate_joint_forbidden`
- **Problème**: Test utilisait `left_antenna` qui n'est plus interdit
- **Solution**: Utiliser `passive_1` qui est réellement interdit
- **Statut**: ✅ CORRIGÉ ET VALIDÉ

### 🔄 2. `test_mapping_reachy_complete.py::test_05_forbidden_joints_complete`
- **À vérifier**: Utilise-t-il les bons joints ?
- **Action**: Vérifier que test utilise `passive_*` et non `left_antenna`

### 🔄 3. `test_mapping_reachy_complete.py::test_11_get_joint_info_forbidden`
- **À vérifier**: Test avec joint interdit
- **Action**: Vérifier comportement attendu

### 🔄 4-6. Autres tests mapping/conformity
- **Action**: Vérifier alignement avec FORBIDDEN_JOINTS

---

## 🔄 Phase 2: Tests Vision (MOYEN) - EN COURS

### ✅ 1. `test_bbia_vision.py::test_track_untrack_object`
- **Problème**: `track_object("livre")` échoue car aucun objet détecté
- **Solution**: Mock `objects_detected` avec objet "livre"
- **Statut**: ✅ CORRIGÉ

### 🔄 2. `test_bbia_vision_extended.py::test_init_specs`
- **Problème**: Spécifications ne matchent pas
- **Action**: Vérifier structure `vision.specs` actuelle

### 🔄 3-14. Autres tests vision
- **Action**: Analyser chaque échec individuellement

---

## ⏳ Phase 3: Tests Performance (SIMPLE) - À FAIRE

### Tests à corriger:
1. `test_runtime_budget_simulation_10s`
2. `test_vision_fps_10s_simulated`
3. `test_vision_budget_cpu_ram_10s`
4. `test_vision_pipeline_latency_simulated`

**Solution**: Ajouter détection CI et seuils adaptatifs

---

## 📊 Suivi Progression

- ✅ Tests mapping corrigés: 1/6
- ✅ Tests vision corrigés: 1/14
- ⏳ Tests performance: 0/4

**Total corrigé**: 2/28 (7%)

