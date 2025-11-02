# 📋 Liste Complète des Tâches Restantes - Oct / Nov. 2025

**Date** : Oct / Nov. 2025
**Source** : Audit exhaustif code + documentation
**Dernière vérification** : Après corrections warnings et vérifications TODOs

---

## ✅ CE QUI EST DÉJÀ FAIT (mais mentionné comme "à faire")

Les éléments suivants sont **déjà implémentés** et ne nécessitent **aucune action** :

1. ✅ **uptime** : Calcul implémenté dans `ecosystem.py` (lignes 27-40)
2. ✅ **active_connections** : Fonction `get_active_connections()` implémentée (ligne 57)
3. ✅ **logique démarrage démo** : Endpoint `/demo/start` complètement implémenté (lignes 434-549)
4. ✅ **SmolVLM2** : Intégré dans `bbia_huggingface.py`
5. ✅ **VAD** : Implémenté dans `voice_whisper.py`
6. ✅ **NER extraction** : Méthodes `_extract_angle()` et `_extract_intensity()` implémentées
7. ✅ **Whisper streaming** : Implémenté avec VAD
8. ✅ **Optimisations performance** : Simulation 1000Hz → 60Hz, get_bbia_voice optimisé (10 boucles → 1 passe), regex compilées en cache
9. ✅ **Tests prioritaires** : `test_ecosystem_priority_high.py` créé et tous les tests passent

---

## ⏳ VRAIES TÂCHES À FAIRE

### 🔴 Priorité Haute

#### 1. 📊 Améliorer Coverage Tests Modules Critiques

**Modules avec coverage < 50%** :

| Module | Coverage Actuel | Lignes Non Couvertes | Fichiers Tests Existant | Action Nécessaire |
|--------|----------------|---------------------|----------------------|-------------------|
| `vision_yolo.py` | 27.74% | 99 lignes | `test_vision_yolo_comprehensive.py` (existe) | ⚠️ **AMÉLIORER** tests existants |
| `voice_whisper.py` | 33.33% | 76 lignes | `test_vad_streaming.py`, `test_ia_modules.py` | ⚠️ **AMÉLIORER** tests existants |
| `dashboard_advanced.py` | **76.71%** ✅ | ~75 lignes | ✅ **EXISTE** : `tests/test_dashboard_advanced.py` (47+ tests, 1169 lignes) | ✅ **TERMINÉ** (+38% depuis 38.82%) |
| `daemon/bridge.py` | 0% | 283 lignes | `test_daemon_bridge.py` (existe partiellement) | ⚠️ **AMÉLIORER** tests existants |

**Actions** :
- ✅ **Dashboard Advanced** : **TERMINÉ** - `tests/test_dashboard_advanced.py` créé et amélioré (47+ tests, 76.71% coverage) ✅
- ⚠️ **Vision YOLO** : Étendre `tests/test_vision_yolo_comprehensive.py` (priorité 1 maintenant)
- ⚠️ **Voice Whisper** : Étendre tests existants (priorité 1 maintenant)
- ⚠️ **Bridge Daemon** : Étendre `tests/test_daemon_bridge.py` (priorité 2)

**Estimation** : 8-12 heures

---

#### 2. 🔗 Vérifier et Corriger Liens Markdown Cassés

**Fichiers à vérifier** :
- `docs/status.md` - Nombreux liens internes
- `docs/INDEX_FINAL.md` - Liens vers archives
- Tous les fichiers MD dans `docs/audit/` (liens croisés)

**Action** : Créer script automatique `scripts/verify_md_links.py`

**Fonctionnalités du script** :
- Vérifier tous les liens dans les MD (utiliser `scripts/verify_docs_complete.py`)
- Vérifier liens internes (`#ancre`)
- Vérifier liens vers fichiers (existe/n'existe pas)
- Générer rapport des liens cassés

**Estimation** : 1 heure (création script) + 30 min (corrections)

---

### 🟡 Priorité Moyenne

#### 3. 📝 Consolider Documents Redondants dans `docs/audit/`

**Groupes identifiés** :

**Groupe A - Résumés d'audit multiples** :
- `AUDIT_100_PERCENT_COMPLET.md`
- `AUDIT_COMPLET_ETAT_REEL_2025.md`
- `BILAN_COMPLET_MARKDOWN_CONFORMITE_2025.md`
- `SYNTHESE_FINALE_AUDIT.md`
- `SYNTHESE_FINALE_TOUTES_CORRECTIONS.md`

**Action** :
1. Identifier le document le plus récent et complet
2. Créer `docs/audit/INDEX_AUDITS.md` qui référence tous les audits
3. Archiver les anciens vers `docs/archives/audits_termines/`
4. Garder seulement les audits uniques (différents objectifs)

**Estimation** : 2-3 heures

---

#### 4. 🔧 TODOs Code Non-Bloquants

**Fichiers avec TODOs** :

| Fichier | TODO | Priorité | Statut |
|---------|------|----------|--------|
| `src/bbia_sim/bbia_tools.py` | Ligne 378: Intégration VisionTrackingBehavior | 🟡 Moyenne | ⏳ À faire |
| `src/bbia_sim/bbia_tools.py` | Ligne 439: Arrêt réel mouvement | 🟡 Moyenne | ⏳ À faire |
| `src/bbia_sim/daemon/app/main.py` | Ligne 241: Auth WebSocket | 🟢 Basse | ⏳ Optionnel |
| `src/bbia_sim/robot_api.py` | Ligne 280: Migration imports | 🟢 Basse | ⏳ Refactoring futur |
| `tests/test_watchdog_monitoring.py` | Ligne 204: Robot physique | 🔵 Hardware | ⏳ Attente hardware |

**Actions** :
1. **bbia_tools.py** : Implémenter intégration VisionTrackingBehavior (1-2h)
2. **bbia_tools.py** : Implémenter arrêt réel mouvement (1h)
3. **main.py** : Auth WebSocket optionnelle (peut attendre)
4. **robot_api.py** : Refactoring futur (non urgent)

**Estimation** : 2-3 heures (sans hardware)

---

### 🟢 Priorité Basse

#### 5. 📚 Documentation Supplémentaire

**Actions** :
- Mettre à jour `docs/guides_techniques/FAQ_TROUBLESHOOTING.md` avec nouvelles fonctionnalités
- Créer guide pour `dashboard_advanced.py`
- Documenter tests coverage dans `tests/README.md`

**Estimation** : 1-2 heures

---

### 🔵 Hardware (En Attente Robot Physique)

#### 6. 🤖 TODOs Robot Réel

**Fichier** : `src/bbia_sim/backends/reachy_backend.py`

**TODOs** :
- Ligne 52: Connexion réelle Reachy
- Ligne 71: Déconnexion réelle
- Ligne 104: Envoi commandes réelles
- Ligne 127: Synchronisation état
- Ligne 143: Arrêt réel via API robot
- Ligne 183: Envoi commandes réelles

**Statut** : ⏳ En attente réception robot physique

**Action** : Implémenter quand robot reçu (nécessite accès hardware)

**Estimation** : 3-4 heures (quand robot disponible)

---

## 📊 Résumé Par Priorité

| Priorité | Tâches | Estimation | Statut |
|----------|-------|------------|--------|
| 🔴 Haute | Coverage tests (4 modules) | 8-12h | ⏳ À faire |
| 🔴 Haute | Vérifier liens MD cassés | 1.5h | ⏳ À faire |
| 🔴 Haute | Optimisations performance | 2-3h | ✅ **TERMINÉ** (simulation 60Hz ✅, voix ✅, regex ✅) |
| 🟡 Moyenne | Consolider documents | 2-3h | ⏳ À faire |
| 🟡 Moyenne | TODOs bbia_tools.py | 2-3h | ⏳ À faire |
| 🟢 Basse | Documentation supplémentaire | 1-2h | ⏳ À faire |
| 🔵 Hardware | TODOs robot réel | 3-4h | ⏳ En attente |

**Total (sans hardware)** : **~15-22 heures** de travail

---

## 🎯 Plan d'Action Recommandé

### Phase 1 : Actions Rapides (< 2h)
1. ✅ Créer script `scripts/verify_md_links.py` (1h)
2. ✅ Vérifier liens MD et corriger (30 min)

### Phase 2 : Coverage Tests (8-12h)
1. ✅ Créer `tests/test_dashboard_advanced.py` (3h) - **PRIORITÉ 1**
2. ⚠️ Étendre `tests/test_vision_yolo_comprehensive.py` (2-3h)
3. ⚠️ Étendre tests `voice_whisper.py` (2-3h)
4. ⚠️ Étendre `tests/test_daemon_bridge.py` (1-2h)

### Phase 3 : Nettoyage Documentation (2-3h)
1. ✅ Consolider documents redondants
2. ✅ Créer index des audits

### Phase 4 : TODOs Code (2-3h)
1. ✅ Implémenter VisionTrackingBehavior intégration
2. ✅ Implémenter arrêt réel mouvement

---

## ✅ Validation Actuelle

**Ce qui est DÉJÀ TERMINÉ** :
- ✅ TODOs `ecosystem.py` : **100% implémenté** (uptime, active_connections, demo start)
- ✅ Qualité code : Warnings corrigés, Black + Ruff OK
- ✅ Scripts : Fusionnés et optimisés
- ✅ Conformité SDK : Validée et testée

**Ce qui reste VRAIMENT à faire** :
1. 📊 Coverage tests (principal bloc de travail)
2. 🔗 Liens MD cassés (rapide)
3. 📝 Nettoyage documentation (moyen)
4. 🔧 TODOs code optionnels (faible priorité)

---

**Dernière mise à jour** : Oct / Nov. 2025
**Prochaine révision** : Après implémentation coverage tests

