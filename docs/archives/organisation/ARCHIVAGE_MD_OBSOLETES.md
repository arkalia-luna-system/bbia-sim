# 📁 Archivage MD Obsolètes - Oct / Nov. 2025

**Date** : Oct / Nov. 2025  
**Objectif** : Archiver les MD qui mentionnent des tâches déjà terminées ou obsolètes

---

## ✅ Critères d'Archivage

Un MD doit être archivé si :
1. Il mentionne des tâches **déjà terminées** et vérifiées dans le code
2. Il contient des informations **remplacées** par des documents plus récents
3. Il est **redondant** avec d'autres documents

---

## 📋 MD à Archiver

### Groupe 1 : Corrections Démos (Déjà Appliquées)

Ces MD mentionnent des corrections qui sont **déjà faites** :

1. **`docs/corrections/CORRECTIONS_DEMOS_REACHY.md`** ✅
   - Mentionne corrections `demo_behavior_ok.py` et `demo_emotion_ok.py`
   - **État réel** : Toutes corrections appliquées (vérifié dans code)
   - **Destination** : `docs/archives/corrections_terminees/`

2. **`docs/corrections/CORRECTIONS_MODULES_NON_PRIORITAIRES_2025.md`** ✅
   - Mentionne corrections `surprise_3d_mujoco_viewer.py` et `demo_chat_bbia_3d.py`
   - **État réel** : Corrections appliquées (vérifié dans code)
   - **Destination** : `docs/archives/corrections_terminees/`

### Groupe 2 : Tests Dashboard (Déjà Créé)

3. **MD mentionnant "créer test_dashboard_advanced.py"** ✅
   - **État réel** : `tests/test_dashboard_advanced.py` existe avec 26 tests (555 lignes)
   - **Action** : Identifier MD mentionnant cette tâche et marquer comme terminé
   - **Destination** : Marquer comme obsolète dans index

### Groupe 3 : Résumés Redondants

4. **`docs/audit/SYNTHESE_FINALE_TOUTES_CORRECTIONS.md`** ⚠️
   - Synthèse de corrections déjà appliquées
   - **Destination** : `docs/archives/audits_termines/` si redondant avec autres synthèses

5. **`docs/RAPPORT_CORRECTION_MD_AUDIT_NOV2025.md`** ⚠️
   - Vérifie cohérence MD vs Code (fait)
   - **Destination** : `docs/archives/audits_termines/` si redondant

---

## 📂 Structure d'Archivage

### Destination : `docs/archives/`

```
docs/archives/
├── corrections_terminees/
│   ├── CORRECTIONS_DEMOS_REACHY.md
│   └── CORRECTIONS_MODULES_NON_PRIORITAIRES_2025.md
├── audits_termines/
│   ├── SYNTHESE_FINALE_TOUTES_CORRECTIONS.md
│   └── RAPPORT_CORRECTION_MD_AUDIT_NOV2025.md
```

---

## ✅ MD à Conserver (Référence Actuelle)

### Documents Actifs (Ne Pas Archiver)

1. **`docs/TACHES_A_FAIRE_CONSOLIDEES.md`** ✅
   - Document principal consolidé (mis à jour avec état réel)

2. **`docs/corrections/CORRECTIONS_APPLIQUEES.md`** ✅
   - Historique des corrections (utile comme référence)

3. **`docs/ameliorations/AMELIORATIONS_FINALES.md`** ✅
   - Liste des améliorations (référence actuelle)

4. **`docs/analyses/RESUME_ANALYSE_EXPERT_FINALE_2025.md`** ✅
   - Synthèse d'analyse (référence actuelle)

---

## 🎯 Plan d'Action

1. ✅ Créer ce document d'archivage
2. ⏳ Vérifier chaque MD avant archivage
3. ⏳ Déplacer MD obsolètes vers archives
4. ⏳ Mettre à jour index si nécessaire

---

**Note** : Ne pas supprimer les MD, seulement les archiver pour garder l'historique.

