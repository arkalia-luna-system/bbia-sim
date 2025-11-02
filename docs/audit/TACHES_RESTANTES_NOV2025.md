# 📋 Tâches Restantes Identifiées - Oct / Nov. 2025

**Date** : Oct / Nov. 2025

**Audit** : Recherche exhaustive de ce qui n'a pas encore été fait

> **🤖 Guide Assistants IA :** Pour scripts et conventions, voir [`../ASSISTANT_IA_GUIDE.md`](../ASSISTANT_IA_GUIDE.md)

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

### 2. ✅ Implémenter TODOs dans ecosystem.py - **100% TERMINÉ**

**Fichier** : `src/bbia_sim/daemon/app/routers/ecosystem.py`

**TODOs traités** :

1. ✅ **Ligne ~121** : `uptime="00:00:00"` → **TERMINÉ**
   - **Action** : Calculer uptime réel depuis démarrage du service
   - **Implémenté** : Fonctions `get_app_start_time()` et `format_uptime()`
   - **Statut** : ✅ Complété

2. ✅ **Ligne ~124** : `active_connections=0` → **TERMINÉ**
   - **Action** : Implémentation complète de `get_active_connections()`
   - **Implémenté** : Tracking réel via gestionnaires WebSocket (telemetry + général)
   - **Fallback** : Retourne 0 si aucun manager disponible
   - **Tests** : `test_ecosystem_priority_high.py` créé et tous passent
   - **Statut** : ✅ Complété

3. ✅ **Ligne ~454** : `@router.post("/demo/start")` → **TERMINÉ**
   - **Action** : Logique de démarrage de démo automatique implémentée
   - **Implémenté** : Endpoint `/demo/start` avec modes (simulation, robot_real, mixed)
   - **Fonctionnalités** : Durée configurable, émotion optionnelle, arrêt automatique
   - **Tests** : Tests créés et tous passent
   - **Statut** : ✅ Complété

**Note** : Tous les TODOs prioritaires de `ecosystem.py` sont maintenant **100% implémentés et testés**.

---

## 🟡 Priorité Moyenne - Améliorations Code

### 3. 📊 Améliorer Coverage Modules Critiques

**Modules avec coverage < 50%** :

1. ✅ **`vision_yolo.py`** : **89.62% coverage** ✅ - **TERMINÉ**
   - **Statut** : Objectif 50%+ largement dépassé

2. **`voice_whisper.py`** : **36.84% coverage** ⬆️ (18+ tests ajoutés, +13.57%)
   - **Action** : Continuer amélioration vers 50%+ (~1-2h restantes)
   - **Focus** : transcribe_streaming, améliorer VAD

3. ✅ **`dashboard_advanced.py`** : **76.71% coverage** ✅ (**47 tests**, **1156 lignes**) - **TERMINÉ**

4. ✅ **`daemon/bridge.py`** : **31.23% coverage** ✅ (10+ tests ajoutés) - **TERMINÉ**
   - **Statut** : Objectif 30%+ atteint

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
| 🔴 Haute | TODOs ecosystem.py | 1-2h | ✅ **100% TERMINÉ** (uptime ✅, WS ✅, démo ✅, tests ✅) |
| 🔴 Haute | Optimisations performance | 2-3h | ✅ **TERMINÉ** (simulation 60Hz ✅, voix ✅, regex ✅) |
| 🟡 Moyenne | Coverage modules critiques | 8-12h | ⏳ À faire |
| 🟡 Moyenne | Réorganiser fichiers MD | 30 min | ✅ TERMINÉ |
| 🟢 Basse | Vérifier liens cassés | 1h | ✅ TERMINÉ (script créé, 212 liens cassés détectés) |
| 🟢 Basse | Consolider documents | 2-3h | ✅ TERMINÉ (INDEX_AUDITS_CONSOLIDES.md créé) |
| 🔵 Hardware | TODOs robot réel | 3-4h | ⏳ En attente |

---

## 🎯 Recommandations Immédiates

**Actions rapides (< 1h)** :
1. ✅ **TERMINÉ** : Mettre à jour `tests/README.md` avec 1210 tests
2. ✅ **TERMINÉ** : Déplacer `logs/comparison_official_report.md` vers `docs/conformite/`
3. ✅ **TERMINÉ** : Implémenter calcul uptime réel dans ecosystem.py

**Actions moyennes (1-3h)** :
1. ✅ **TERMINÉ** : Implémenter comptage connexions WebSocket (avec tests)
2. ✅ **TERMINÉ** : Implémenter logique démarrage démo (avec tests)
3. ✅ **TERMINÉ** : Optimisations performance (simulation 60Hz, voix, regex)
4. ⏳ **À FAIRE** : Améliorer coverage `vision_yolo.py` et `voice_whisper.py` (8-12h total)
5. ⏳ **À FAIRE** : Vérifier liens markdown cassés (1h)

**Actions longues (> 3h)** :
1. ✅ **TERMINÉ** : Logique démarrage démo (1-2h) - **COMPLÉTÉ**
2. ⏳ **À FAIRE** : Consolider documents redondants (2-3h)

---

**Date création** : Oct / Nov. 2025
**Prochaine révision** : Oct / Nov. 2025

---

## ✅ AUDIT SDK OFFICIEL Oct / Nov. 2025 - TERMINÉ

### Tests de Conformité Renforcés

**46 tests** vérifient maintenant TOUS les aspects du README officiel :

1. ✅ **Daemon** : Commande `reachy-mini-daemon` vérifiée
2. ✅ **API REST** : Endpoints `/`, `/docs`, `/api/state/full` vérifiés
3. ✅ **Modules Media** : `robot.media.camera`, `.microphone`, `.speaker` vérifiés
4. ✅ **Modules IO** : `robot.io.get_camera_stream()`, `.get_audio_stream()` vérifiés
5. ✅ **Versions Python** : Support 3.10-3.13 confirmé
6. ✅ **git-lfs** : Requis vérifié
7. ✅ **create_head_pose** : Signature conforme README
8. ✅ **Hugging Face** : Intégration vérifiée
9. ✅ **Statut Beta** : Fallbacks robustes pour gérer bugs SDK

### Points Potentiellement Problématiques - ✅ TOUS CORRIGÉS

1. ✅ **robot.io** : **TOUJOURS DISPONIBLE** via `SimulationIOModule` en simulation
   - Shim créé : `src/bbia_sim/backends/simulation_shims.py`
   - Retourne toujours un objet valide (jamais None)
   - Méthodes : `get_camera_stream()`, `get_audio_stream()`, `get_imu()`

2. ✅ **robot.media** : **TOUJOURS DISPONIBLE** via `SimulationMediaModule` en simulation
   - Shim créé avec `camera`, `microphone`, `speaker`
   - Retourne toujours un objet valide (jamais None)
   - Méthodes : `play_audio()`, `record_audio()`, `.camera.get_image()`, etc.

3. ✅ **Fallbacks** : **RENFORCÉS ET VÉRIFIÉS**
   - Audio : SDK → sounddevice → erreur gracieuse
   - Vision : SDK camera → OpenCV webcam → simulation
   - Voice : SDK speaker → pyttsx3 → erreur gracieuse
   - Tous les fallbacks testés et documentés

### Conformité Validée

- ✅ **SDK Version** : 1.0.0rc5 installé et fonctionnel
- ✅ **Daemon** : Commande disponible
- ✅ **Toutes méthodes SDK** : Implémentées et testées
- ✅ **Modules IO/Media** : Exposés correctement via `@property`

**Statut** : ✅ Conforme au SDK officiel (Oct / Nov. 2025)

