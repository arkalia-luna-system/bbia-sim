# 📋 Tâches à Faire - Document Consolidé

**Date** : Oct / Nov. 2025  
**Source** : Analyse exhaustive de tous les MD de corrections, améliorations, audits  
**Dernière mise à jour** : Après lecture complète des fichiers MD

---

## ✅ CE QUI EST DÉJÀ FAIT (Vérifié)

Les éléments suivants sont **déjà implémentés** et validés :

1. ✅ **Emergency Stop** - Implémenté dans tous les backends
2. ✅ **Audio SDK Alignment** (16kHz) - Aligné SDK Reachy Mini
3. ✅ **Validation Émotions SDK** - Intensité [0.0, 1.0] validée
4. ✅ **Tests Sécurité Limites** - 5 tests créés et validés
5. ✅ **Support BBIA_DISABLE_AUDIO** - Respecté dans TTS/Audio
6. ✅ **Endpoint /stop avec Emergency Stop** - Implémenté
7. ✅ **Module Media SDK** - Intégré (robot.media.camera, microphone, speaker)
8. ✅ **Interpolation Adaptative** - Mapping émotion → technique implémenté
9. ✅ **Enregistrement/Replay** - Implémenté dans bbia_behavior.py
10. ✅ **Sécurité JSON** - Validation payload implémentée
11. ✅ **Améliorations Intelligence** - Réponses variées, langage naturel
12. ✅ **Uptime & Active Connections** - Implémentés dans ecosystem.py
13. ✅ **Logique démarrage démo** - Endpoint /demo/start implémenté
14. ✅ **SmolVLM2, VAD, NER, Whisper streaming** - Tous implémentés
15. ✅ **Optimisations performance** - Simulation 60Hz, voix, regex optimisées

---

## 🔴 PRIORITÉ HAUTE - Tâches Restantes

### 1. 📊 Améliorer Coverage Tests Modules Critiques

**Modules avec coverage < 50%** :

| Module | Coverage Actuel | Lignes Non Couvertes | Fichiers Tests | Action | État Réel |
|--------|------------------|---------------------|----------------|--------|-----------|
| `vision_yolo.py` | 27.74% | 99 lignes | `test_vision_yolo_comprehensive.py` (existe) | ⚠️ **AMÉLIORER** tests existants | ⏳ À améliorer |
| `voice_whisper.py` | 33.33% | 76 lignes | `test_vad_streaming.py`, `test_ia_modules.py` | ⚠️ **AMÉLIORER** tests existants | ⏳ À améliorer |
| `dashboard_advanced.py` | **76.71%** ⬆️⬆️ | ~75 lignes | ✅ **EXISTE** : `tests/test_dashboard_advanced.py` (47+ tests, 1169 lignes) | ✅ **AMÉLIORÉ** (+38% depuis 38.82%) | ✅ **TERMINÉ** |
| `daemon/bridge.py` | 0% | 283 lignes | `test_daemon_bridge.py` (partiel) | ⚠️ **AMÉLIORER** tests existants | ⏳ À améliorer |

**Estimation** : 3-6 heures (réduit car dashboard_advanced amélioré)

**Plan d'action** :
1. ✅ ~~Créer `tests/test_dashboard_advanced.py`~~ - **TERMINÉ** (47+ tests, 1169 lignes)
2. ✅ ~~Améliorer coverage de `tests/test_dashboard_advanced.py`~~ - **TERMINÉ** (38.82% → **76.71%**, +38%, objectif 70%+ dépassé ✅)
3. ⚠️ **Étendre `tests/test_vision_yolo_comprehensive.py`** (priorité 1 maintenant, 2-3h)
4. ⚠️ **Étendre tests `voice_whisper.py`** (priorité 1 maintenant, 2-3h)
5. ⚠️ **Étendre `tests/test_daemon_bridge.py`** (priorité 2, 1-2h)

---

### 2. 🔗 Vérifier et Corriger Liens MD Cassés

**Fichier** : `scripts/verify_md_links.py` (existe déjà)

**Action** :
- ✅ Exécuter le script pour détecter liens cassés
- ✅ Corriger 112 liens dans fichiers actifs (Oct / Nov. 2025)
  - `.github/ISSUES_TO_CREATE.md` : corrigé
  - `docs/FAQ.md` : corrigé (6 liens)
  - `docs/STYLE_GUIDE_MD.md` : corrigé
  - `docs/references/INDEX.md` : corrigé (15+ liens)
- ⏳ Reste : ~139 liens (majoritairement dans archives - non prioritaire)

**Progrès** : 251 → 139 liens cassés (-45% dans fichiers actifs)
**Estimation restante** : ~30 min (archives optionnelles)

---

## 🟡 PRIORITÉ MOYENNE - Améliorations

### 3. 🔧 TODOs bbia_tools.py

**Fichier** : `src/bbia_sim/bbia_tools.py`

**TODOs identifiés** :
- ✅ ~~Ligne ~378: Intégration VisionTrackingBehavior~~ - **TERMINÉ** (Oct / Nov. 2025)
  - Action : Implémenter intégration complète VisionTrackingBehavior
  - **État** : Intégré avec VisionTrackingBehavior.execute() si vision et robot_api disponibles
- ✅ ~~Ligne ~439: Arrêt réel mouvement~~ - **TERMINÉ** (Oct / Nov. 2025)
  - Action : Implémenter arrêt réel mouvement (au-delà de emergency_stop)
  - **État** : Utilise robot_api.emergency_stop() pour arrêt immédiat et sécurisé

**Estimation** : ✅ **TERMINÉ**

---

### 4. 📝 Consolider Documentation

**Objectif** : Réorganiser et consolider fichiers MD redondants

**Actions** :
1. Identifier documents les plus récents et complets
2. Archiver anciens vers `docs/archives/audits_termines/`
3. Créer index consolidé (déjà fait : `INDEX_AUDITS_CONSOLIDES.md`)
4. Réduire doublons (~30% fichiers MD)

**Groupes identifiés** :
- **Groupe A - Résumés d'audit** (docs/audit/) : Plusieurs fichiers "FINAL", "COMPLET", "VERIFICATION"
- **Groupe B - Corrections** : Plusieurs fichiers de corrections similaires

**Estimation** : 2-3 heures

---

### 5. 🎯 Corriger Démos Reachy Mini

**État réel vérifié** : ✅ **TOUTES LES CORRECTIONS DÉJÀ APPLIQUÉES**

1. **`examples/demo_behavior_ok.py`** ✅ **DÉJÀ CORRIGÉ**
   - Amplitudes conformes : max 0.234 rad (< 0.3 rad) ✅
   - Utilise méthodes SDK appropriées ✅
   - Commentaires SDK explicites présents ✅

2. **`examples/demo_emotion_ok.py`** ✅ **DÉJÀ CORRIGÉ**
   - Amplitudes conformes : max 0.22 rad (< 0.3 rad) ✅
   - Patterns émotionnels optimisés ✅

3. **`examples/demo_reachy_mini_corrigee.py`** ✅ **DÉJÀ CORRIGÉ**
   - Utilise `goto_target()` avec `create_head_pose()` ✅
   - Interpolation adaptative par émotion ✅
   - Conforme SDK officiel ✅

**Conclusion** : ✅ Toutes les corrections démos sont déjà appliquées. Les MD mentionnant ces corrections peuvent être archivés.

---

## 🟢 PRIORITÉ BASSE - Améliorations Optionnelles

### 6. 📚 Documentation Supplémentaire

**Actions** :
1. Mettre à jour `docs/guides_techniques/FAQ_TROUBLESHOOTING.md` avec nouvelles fonctionnalités
2. Créer guide pour `dashboard_advanced.py`
3. Documenter tests coverage dans `tests/README.md`

**Estimation** : 1-2 heures

---

### 7. 🔧 TODOs Optionnels Code

**Fichiers avec TODOs non-bloquants** :

| Fichier | TODO | Priorité |
|---------|------|----------|
| `src/bbia_sim/daemon/app/main.py` | Ligne 241: Auth WebSocket | 🟢 Basse (Optionnel) |

**Estimation** : 1 heure

---

## 🔵 HARDWARE - En Attente Robot Physique

### 8. 🤖 TODOs Robot Réel (Nécessite Hardware)

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

### 9. ⚠️ Module IO SDK (Non Utilisé)

**Status** : Disponible dans `ReachyMiniBackend.io` mais NON UTILISÉ

**Capacités disponibles** :
```python
robot.io.get_camera_stream()  # Stream vidéo temps réel
robot.io.get_audio_stream()   # Stream audio temps réel
robot.io.set_leds()            # Contrôle LEDs (si disponibles)
```

**Opportunités** :
- Vision temps réel au lieu de scan périodique
- Audio streaming pour reconnaissance vocale temps réel
- Feedback visuel via LEDs

**Priorité** : **Basse** (fonctionnalités actuelles suffisantes)

---

## 📊 Résumé Par Priorité

| Priorité | Tâches | Estimation | Statut |
|----------|--------|------------|--------|
| 🔴 Haute | Coverage tests (3 modules restants) | 3-6h | ✅ **EN PROGRÈS** (dashboard: TERMINÉ ✅, reste: vision_yolo, voice_whisper, bridge) |
| 🔴 Haute | Vérifier liens MD cassés | - | ✅ **EN PROGRÈS** (112/251 corrigés, -45%) |
| 🟡 Moyenne | TODOs bbia_tools.py | - | ✅ **TERMINÉ** (Oct / Nov. 2025) |
| 🟡 Moyenne | Corriger démos Reachy | - | ✅ **DÉJÀ FAIT** |
| 🟡 Moyenne | Consolider documents | - | ✅ **EN PROGRÈS** (2 MD archivés) |
| 🟢 Basse | Documentation supplémentaire | 1-2h | ⏳ À faire |
| 🟢 Basse | TODOs optionnels code | 1h | ⏳ À faire |
| 🔵 Hardware | TODOs robot réel | 3-4h | ⏳ En attente |
| 🔵 Hardware | Module IO SDK | - | ⏳ Optionnel |

**Total (sans hardware)** : **~10-16 heures** de travail (réduit car démos corrigées)

---

## 🎯 Plan d'Action Recommandé

### Phase 1 : Actions Rapides (< 3h)
1. ✅ Vérifier liens MD cassés avec script existant (1.5h)
2. ✅ Corriger liens détectés (1h)

### Phase 2 : Coverage Tests (3-6h) - PRIORITÉ 1
1. ✅ ~~Créer `tests/test_dashboard_advanced.py`~~ - **TERMINÉ** (47+ tests, 1169 lignes)
2. ✅ ~~Améliorer coverage de `tests/test_dashboard_advanced.py`~~ - **TERMINÉ** (76.71%, objectif 70%+ dépassé ✅)
3. ⚠️ **PRIORITÉ 1** : Étendre `tests/test_vision_yolo_comprehensive.py` (2-3h)
4. ⚠️ **PRIORITÉ 1** : Étendre tests `voice_whisper.py` (2-3h)
5. ⚠️ **PRIORITÉ 2** : Étendre `tests/test_daemon_bridge.py` (1-2h)

### Phase 3 : Corrections Démos
1. ✅ ~~Corriger `demo_behavior_ok.py`~~ - **DÉJÀ FAIT** (amplitudes conformes)
2. ✅ ~~Corriger `demo_emotion_ok.py`~~ - **DÉJÀ FAIT** (amplitudes conformes)
3. ✅ ~~Corriger `demo_reachy_mini_corrigee.py`~~ - **DÉJÀ FAIT** (utilise goto_target)

### Phase 4 : Nettoyage & TODOs (4-6h)
1. ✅ Consolider documents redondants
2. ✅ Implémenter VisionTrackingBehavior intégration
3. ✅ Implémenter arrêt réel mouvement
4. ✅ Documentation supplémentaire

### Phase 5 : Hardware (Quand disponible)
1. ✅ Implémenter connexion robot réel
2. ✅ Implémenter envoi commandes réelles
3. ✅ Synchronisation état robot

---

## 📝 Notes Importantes

### Corrections Démos - Règles SDK
- **Limite amplitude** : 0.3 rad max (GLOBAL_SAFETY_LIMIT)
- **Méthodes SDK** : Utiliser `goto_target()` avec `create_head_pose()`
- **Interpolation** : Utiliser méthodes adaptées (minjerk, cartoon, ease_in_out)
- **Joints Stewart** : Ne jamais contrôler individuellement (IK requise)

### Coverage Tests - Objectifs
- **dashboard_advanced.py** : 0% → objectif 70%+
- **vision_yolo.py** : 27.74% → objectif 70%+
- **voice_whisper.py** : 33.33% → objectif 70%+
- **daemon/bridge.py** : 0% → objectif 70%+

---

## ✅ Validation Finale - État Réel Vérifié

**Ce qui reste VRAIMENT à faire** :
1. 📊 Coverage tests (principal bloc de travail) - **PRIORITÉ 1**
   - ✅ `dashboard_advanced.py` : 38.82% → **76.71%** (+38%) - **TERMINÉ** ✅
   - ⏳ Objectif : Atteindre 70%+ pour tous les modules
   - ⏳ `vision_yolo.py` : 27.74% → 70%+ (à améliorer)
   - ⏳ `voice_whisper.py` : 33.33% → 70%+ (à améliorer)
   - ⏳ `daemon/bridge.py` : 0% → 70%+ (à améliorer)
2. 🔗 Liens MD cassés (rapide) - **EN PROGRÈS** (112/251 corrigés, -45%)
3. 📝 Nettoyage documentation (moyen) - **EN PROGRÈS** (2 MD archivés)

**Ce qui est DÉJÀ FAIT** (session Oct / Nov. 2025) :
- ✅ TODOs `bbia_tools.py` terminés (VisionTrackingBehavior + arrêt réel)
- ✅ `test_dashboard_advanced.py` : **47+ tests créés** (1169 lignes)
- ✅ Coverage `dashboard_advanced.py` : **+38%** (38.82% → **76.71%**) - Objectif 70%+ dépassé !
- ✅ 112 liens MD corrigés dans fichiers actifs (-45%)
- ✅ 2 MD obsolètes archivés
- ✅ Toutes les corrections démos appliquées (amplitudes conformes, goto_target utilisé)
- ✅ Optimisations performance terminées
- ✅ Emergency stop, audio SDK, validation émotions, etc.

---

**Dernière mise à jour** : Oct / Nov. 2025  
**Source** : Analyse complète de tous les MD + vérification état réel du code  
**Vérification** : Code inspecté directement (fichiers, tests, démos)  
**Progrès session** : Coverage dashboard **+38%** (76.71%), TODOs terminés, liens MD -45%, **11 nouveaux tests handle_advanced_robot_command** ajoutés

**Voir** : `docs/PROGRES_DECEMBRE_2025.md` pour détails complets de la session

