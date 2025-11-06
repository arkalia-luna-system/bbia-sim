# 📁 Scripts d'Audit/Comparaison Archivés

**Date archivage** : Oct / No2025025025025025  
**Raison** : Consolidation des scripts redondants

---

## 📋 Scripts Archivés

### 🔴 Scripts Obsolètes (Versions Antérieures)

1. **`audit_systematique_exhaustif.py`** (458 lignes)
   - **Raison** : Redondant avec `compare_with_official_exhaustive.py`
   - **Fonction** : Audit exhaustif (similaire au script principal)
   - **Statut** : Fonctionnalités intégrées dans `compare_with_official_exhaustive.py`

2. **`audit_conformite_complete.py`** (376 lignes)
   - **Raison** : Version antérieure obsolète
   - **Fonction** : Conformité complète (ancienne version)
   - **Statut** : Remplacé par `compare_with_official_exhaustive.py`

3. **`audit_systematique_complet.py`** (363 lignes)
   - **Raison** : Version antérieure (précède `audit_systematique_exhaustif.py`)
   - **Fonction** : Audit systématique (ancienne version)
   - **Statut** : Remplacé par versions plus récentes

### 🔄 Scripts Fusionnés (Fonctionnalités Intégrées)

4. **`compare_sdk_methods.py`** (73 lignes)
   - **Raison** : Fonctionnalité simple intégrée dans script principal
   - **Fonction** : Compare méthodes SDK (simple)
   - **Statut** : Logique intégrée dans `compare_with_official_exhaustive.py`

5. **`audit_methodes_backend.py`** (171 lignes)
   - **Raison** : Fonctionnalité spécifique intégrée dans script principal
   - **Fonction** : Audit méthodes backend spécifiquement
   - **Statut** : Logique intégrée dans `compare_with_official_exhaustive.py`

6. **`audit_exhaustif_details.py`** (199 lignes)
   - **Raison** : Détails spécifiques intégrés dans script principal
   - **Fonction** : Détails spécifiques (doc, tests, assets)
   - **Statut** : Vérifications intégrées dans `compare_with_official_exhaustive.py`

### ⚠️ Scripts Supplémentaires (Non Utilisés)

7. **`verifier_et_corriger_audits.py`** (142 lignes)
   - **Raison** : Fonctionnalité peut-être redondante ou non utilisée
   - **Fonction** : Vérifie corrections mentionnées dans MD
   - **Statut** : À analyser si besoin

8. **`audit_reachy_integration.py`** (478 lignes)
   - **Raison** : Audit intégration spécifique, peut-être non pertinent
   - **Fonction** : Audit complet BBIA → Reachy Integration (JSONL + MD)
   - **Statut** : Peut être réactivé si nécessaire pour audits d'intégration spécifiques

---

## ✅ Scripts Conservés (Actifs)

### Scripts Principaux

1. **`compare_with_official_exhaustive.py`** ✅
   - **Localisation** : `scripts/compare_with_official_exhaustive.py`
   - **Fonction** : Script principal de comparaison exhaustive
   - **Compare** : API, classes, modèles, tests, docs, scripts
   - **Utilisation** : `python scripts/compare_with_official_exhaustive.py`

2. **`check_official_alignment.py`** ✅
   - **Localisation** : `scripts/check_official_alignment.py`
   - **Fonction** : Vérifie alignement MJCF/STL avec repo officiel
   - **Utilisation** : `python scripts/check_official_alignment.py`

3. **`generate_conformity_report_reachy_mini.py`** ✅
   - **Localisation** : `scripts/generate_conformity_report_reachy_mini.py`
   - **Fonction** : Génère rapports de conformité depuis tests
   - **Utilisation** : `python scripts/generate_conformity_report_reachy_mini.py`

---

## 🔄 Migration

Les fonctionnalités des scripts archivés ont été intégrées dans `compare_with_official_exhaustive.py` :

- ✅ Comparaison méthodes SDK → `compare_python_classes()` amélioré
- ✅ Audit méthodes backend → `compare_python_classes()` avec focus backend
- ✅ Détails spécifiques (doc, tests, assets) → `compare_documentation()`, `compare_tests()`, etc.

---

## 📝 Notes

- Les scripts archivés sont conservés pour référence historique
- Pour réutiliser un script archivé, le copier depuis `_archived/comparison_audit/`
- Le script principal `compare_with_official_exhaustive.py` est maintenu et amélioré

---

**Dernière mise à jour** : Oct / No2025025025025025
