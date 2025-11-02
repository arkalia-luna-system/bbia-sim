# 📋 RÉORGANISATION DES FICHIERS .MD

**Date** : Oct / Nov. 2025
**Objectif**: Organiser tous les fichiers .md à leur bonne place

---

## 🔍 FICHIERS À DÉPLACER

### 1. ⚠️ **`logs/conformity_report_reachy_mini.md`**
**Actuel**: `logs/conformity_report_reachy_mini.md`
**Recommandé**: `docs/conformite/conformity_report_reachy_mini.md`
**Raison**: Rapport de conformité, doit être avec les autres docs de conformité

### 2. ⚠️ **`assets/images/REACHY_MINI_REFERENCE.md`**
**Actuel**: `assets/images/REACHY_MINI_REFERENCE.md`
**Recommandé**: `docs/reachy/REACHY_MINI_REFERENCE.md`
**Raison**: Référence visuelle du robot, doit être avec la documentation Reachy

### 3. ⚠️ **Fichiers d'audit dans `artifacts/`**
Ces fichiers sont des rapports d'audit et devraient être dans `docs/audit/` ou `docs/archives/audits/`:

- `artifacts/AUDIT_100_PERCENT_COMPLET.md` → `docs/audit/AUDIT_100_PERCENT_COMPLET.md`
- `artifacts/AUDIT_REACHY_COMPLET_FINAL.md` → `docs/audit/AUDIT_REACHY_COMPLET_FINAL.md`
- `artifacts/AUDIT_REACHY_SYNTHESE.md` → `docs/audit/AUDIT_REACHY_SYNTHESE.md`
- `artifacts/STATUT_FINAL_AUDIT.md` → `docs/audit/STATUT_FINAL_AUDIT.md`
- `artifacts/SYNTHESE_FINALE_AUDIT.md` → `docs/audit/SYNTHESE_FINALE_AUDIT.md`
- `artifacts/AMELIORATIONS_SUITE_AUDIT.md` → `docs/audit/AMELIORATIONS_SUITE_AUDIT.md`
- `artifacts/AMELIORATIONS_FINALES.md` → `docs/ameliorations/AMELIORATIONS_FINALES.md`
- `artifacts/CORRECTIONS_APPLIQUEES.md` → `docs/corrections/CORRECTIONS_APPLIQUEES.md`
- `artifacts/PROCHAINES_ETAPES.md` → `docs/audit/PROCHAINES_ETAPES_AUDIT.md` (pour éviter doublon)
- `artifacts/WATCHDOG_IMPLEMENTATION.md` → `docs/performance/WATCHDOG_IMPLEMENTATION.md`

**Raison**: Les `artifacts/` sont normalement pour les fichiers générés (logs, résultats tests), pas la documentation

---

## 📁 FICHIERS BONS (À CONSERVER)

Ces fichiers sont bien placés :
- ✅ `README.md` (racine)
- ✅ `CHANGELOG.md` (racine)
- ✅ `CONTRIBUTING.md` (racine)
- ✅ `CODE_OF_CONDUCT.md` (racine)
- ✅ `BADGES.md` (racine)
- ✅ `docs/README.md`
- ✅ `examples/README.md`
- ✅ `assets/README.md`
- ✅ `tests/README.md`
- ✅ `presentation/*.md` (présentations)

---

## 🗑️ FICHIERS À SUPPRIMER (Métadonnées macOS)

Les fichiers `._*.md` sont des métadonnées macOS et peuvent être supprimés :
- `artifacts/._*.md` (tous)

**Commande pour supprimer**:
```bash
find artifacts -name "._*.md" -delete
find docs -name "._*.md" -delete
```

---

## 📝 COMMANDES POUR EXÉCUTER LES DÉPLACEMENTS

```bash
# 1. Déplacer rapport conformité
mv logs/conformity_report_reachy_mini.md docs/conformite/

# 2. Déplacer référence visuelle Reachy
mv assets/images/REACHY_MINI_REFERENCE.md docs/reachy/

# 3. Déplacer audits
mv artifacts/AUDIT_100_PERCENT_COMPLET.md docs/audit/
mv artifacts/AUDIT_REACHY_COMPLET_FINAL.md docs/audit/
mv artifacts/AUDIT_REACHY_SYNTHESE.md docs/audit/
mv artifacts/STATUT_FINAL_AUDIT.md docs/audit/
mv artifacts/SYNTHESE_FINALE_AUDIT.md docs/audit/
mv artifacts/AMELIORATIONS_SUITE_AUDIT.md docs/audit/

# 4. Déplacer améliorations et corrections
mv artifacts/AMELIORATIONS_FINALES.md docs/ameliorations/
mv artifacts/CORRECTIONS_APPLIQUEES.md docs/corrections/

# 5. Déplacer PROCHAINES_ETAPES (éviter doublon)
mv artifacts/PROCHAINES_ETAPES.md docs/audit/PROCHAINES_ETAPES_AUDIT.md

# 6. Déplacer WATCHDOG
mv artifacts/WATCHDOG_IMPLEMENTATION.md docs/performance/

# 7. Supprimer métadonnées macOS
find artifacts -name "._*.md" -delete
find docs -name "._*.md" -delete
```

---

## ✅ VÉRIFICATION POST-DÉPLACEMENT

Après déplacement, vérifier :
1. Tous les liens dans les fichiers .md sont toujours valides
2. Aucun fichier important n'a été supprimé
3. La structure `docs/` est cohérente

