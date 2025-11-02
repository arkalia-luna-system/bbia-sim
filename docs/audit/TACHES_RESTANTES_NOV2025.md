# 📋 Tâches Restantes Identifiées - Novembre 2025

**Date** : Oct 25 / Nov 25  
**Audit** : Recherche exhaustive de ce qui n'a pas encore été fait

---

## 🔴 Priorité Haute - À Faire Immédiatement

### 1. ✅ Mettre à jour tests/README.md avec chiffres réels - TERMINÉ

**Problème** : Le fichier mentionnait "441 tests passent" alors qu'il y a maintenant **1210 tests**

**Fichier** : `tests/README.md`

**Action** : ✅ CORRIGÉ
```markdown
- **Coverage total** : **68.86%** (excellent)
- **1210 tests collectés** (pytest --collect-only)
- **Tests passent** : Voir résultats pytest récents
```

**Statut** : ✅ Complété

---

### 2. ✅ Implémenter TODOs dans ecosystem.py - PARTIELLEMENT TERMINÉ

**Fichier** : `src/bbia_sim/daemon/app/routers/ecosystem.py`

**TODOs traités** :

1. ✅ **Ligne ~121** : `uptime="00:00:00"` → **TERMINÉ**
   - **Action** : Calculer uptime réel depuis démarrage du service
   - **Implémenté** : Fonctions `get_app_start_time()` et `format_uptime()`
   - **Statut** : ✅ Complété

2. ⚠️ **Ligne ~124** : `active_connections=0` → **EN COURS**
   - **Action** : Infrastructure préparée avec `get_active_connections()`
   - **Restant** : Implémenter tracking réel via gestionnaire WebSocket
   - **Statut** : ⏳ En cours (nécessite intégration avec gestionnaire WS)

3. ⏳ **Ligne ~408** : `# TODO: Implémenter la logique de démarrage de démo`
   - **Action** : Logique pour démarrer une démo automatiquement
   - **Complexité** : Moyenne
   - **Statut** : ⏳ À faire

---

## 🟡 Priorité Moyenne - Améliorations Code

### 3. 📊 Améliorer Coverage Modules Critiques

**Modules avec coverage < 50%** :

1. **`vision_yolo.py`** : 27.74% coverage (99 lignes non couvertes)
   - **Action** : Créer `tests/test_vision_yolo_extended.py`
   - **Focus** : Détection objets, classification, gestion images
   - **Estimation** : 2-3 heures

2. **`voice_whisper.py`** : 33.33% coverage (76 lignes non couvertes)
   - **Action** : Créer tests pour Whisper ASR, transcription, gestion erreurs
   - **Estimation** : 2-3 heures

3. **`dashboard_advanced.py`** : 0% coverage (288 lignes non couvertes)
   - **Action** : Tests pour dashboard avancé
   - **Estimation** : 2-3 heures

4. **`daemon/bridge.py`** : 0% coverage (283 lignes non couvertes)
   - **Action** : Tests pour bridge daemon
   - **Estimation** : 2-3 heures

---

### 4. ✅ Réorganiser Fichiers MD Mal Placés - TERMINÉ

**Fichiers traités** :

1. ✅ **`logs/comparison_official_report.md`** 
   - **Avant** : `logs/`
   - **Après** : `docs/conformite/comparison_official_report.md`
   - **Raison** : Rapport de conformité, maintenant avec autres docs conformité
   - **Statut** : ✅ Déplacé

2. ⚠️ **Fichiers MD dans `artifacts/`** 
   - **Fichier trouvé** : `artifacts/golden/schema.md` (peut rester, fait partie de golden)
   - **Statut** : Vérifié, pas d'action nécessaire

**Statut** : ✅ Complété

---

## 🟢 Priorité Basse - Nettoyage et Optimisation

### 5. 🔗 Vérifier Liens Internes Cassés

**Fichiers à vérifier** :
- `docs/status.md` - Nombreux liens, nécessite vérification complète
- `docs/INDEX_FINAL.md` - Liens vers fichiers archives à vérifier

**Action** : Script automatique pour vérifier tous les liens markdown

**Estimation** : 1 heure

---

### 6. 📝 Consolider Documents Redondants

**Groupes identifiés** :

**Groupe A - Résumés d'audit** (docs/audit/) :
- Plusieurs fichiers "FINAL", "COMPLET", "VERIFICATION"
- **Action** : Identifier les plus récents et archiver les autres

**Groupe B - Corrections** :
- Plusieurs fichiers de corrections similaires
- **Action** : Consolider dans un document unique avec références

**Estimation** : 2-3 heures

---

### 7. 🔧 TODOs Robot Réel (Nécessite Hardware)

**Fichier** : `src/bbia_sim/backends/reachy_backend.py`

**TODOs** :
- Ligne ~52: `# TODO: Implémenter la vraie connexion Reachy`
- Ligne ~71: `# TODO: Implémenter la vraie déconnexion Reachy`
- Ligne ~104: `# TODO: Envoyer la commande au robot réel`
- Ligne ~127: `# TODO: Synchroniser avec le robot réel`
- Ligne ~165: `# TODO: Implémenter l'envoi de commandes réelles`

**Statut** : ⏳ En attente de réception robot physique

**Action** : Implémenter quand robot reçu (nécessite accès hardware)

**Estimation** : 3-4 heures (quand robot disponible)

---

## 📊 Résumé Par Priorité

| Priorité | Tâches | Estimation | Statut |
|----------|-------|------------|--------|
| 🔴 Haute | Mettre à jour tests/README.md | 15 min | ✅ TERMINÉ |
| 🔴 Haute | TODOs ecosystem.py | 1-2h | ⚠️ PARTIEL (uptime ✅, WS ⏳) |
| 🟡 Moyenne | Coverage modules critiques | 8-12h | ⏳ À faire |
| 🟡 Moyenne | Réorganiser fichiers MD | 30 min | ✅ TERMINÉ |
| 🟢 Basse | Vérifier liens cassés | 1h | ⏳ À faire |
| 🟢 Basse | Consolider documents | 2-3h | ⏳ À faire |
| 🔵 Hardware | TODOs robot réel | 3-4h | ⏳ En attente |

---

## 🎯 Recommandations Immédiates

**Actions rapides (< 1h)** :
1. ✅ Mettre à jour `tests/README.md` avec 1210 tests
2. ✅ Déplacer `logs/conformity_report_reachy_mini.md` vers `docs/conformite/`
3. ✅ Implémenter calcul uptime réel dans ecosystem.py (15 min)

**Actions moyennes (1-3h)** :
1. ✅ Implémenter comptage connexions WebSocket
2. ✅ Améliorer coverage `vision_yolo.py` et `voice_whisper.py`
3. ✅ Vérifier liens markdown cassés

---

**Date création** : Oct 25 / Nov 25  
**Prochaine révision** : Décembre 2025

