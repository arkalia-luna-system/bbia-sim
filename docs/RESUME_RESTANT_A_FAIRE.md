# 📋 Résumé - Ce Qu'il Reste à Faire

**Date** : Oct / Nov. 2025  
**Dernière mise à jour** : Après mise à jour de tous les MD

---

## ✅ CE QUI EST TERMINÉ

### Coverage Tests
- ✅ **`dashboard_advanced.py`** : **76.71% coverage** ✅
  - **47 tests collectés** (1156 lignes de code)
  - Objectif 70%+ **DÉPASSÉ** ✅
  - Routes FastAPI testées : GET /api/status, /api/metrics, /api/joints, /healthz, POST /api/emotion, /api/joint
  - Gestion commandes robot (`handle_advanced_robot_command`) testée

### TODOs Code
- ✅ **`bbia_tools.py` ligne 378-389** : Intégration VisionTrackingBehavior dans `_execute_head_tracking()` - **TERMINÉ**
  - Utilise `VisionTrackingBehavior.execute()` si vision et robot_api disponibles
- ✅ **`bbia_tools.py` ligne 469-493** : Arrêt réel mouvement avec `emergency_stop()` dans `_execute_stop_dance()` - **TERMINÉ**
  - Appelle `robot_api.emergency_stop()` pour arrêt immédiat et sécurisé

### Documentation
- ✅ **112 liens MD corrigés** dans fichiers actifs (-45%, 251 → 139 liens restants)
- ✅ **2 MD obsolètes archivés** : `CORRECTIONS_DEMOS_REACHY.md`, `CORRECTIONS_MODULES_NON_PRIORITAIRES_2025.md`
- ✅ Documentation tests et améliorations à jour

---

## ⏳ CE QUI RESTE À FAIRE

### 🔴 Priorité Haute - Coverage Tests (3-6h)

#### 1. `vision_yolo.py` - 27.74% → objectif 50%+ (2-3h)
- **Fichier test** : `tests/test_vision_yolo_comprehensive.py` (existe déjà)
- **Lignes non couvertes** : 99 lignes
- **Actions** :
  - Étendre tests existants pour couvrir détection objets
  - Ajouter tests classification
  - Tester gestion images et erreurs

#### 2. `voice_whisper.py` - 33.33% → objectif 50%+ (2-3h)
- **Fichiers tests** : `tests/test_vad_streaming.py`, `tests/test_ia_modules.py` (existent)
- **Lignes non couvertes** : 76 lignes
- **Actions** :
  - Étendre tests Whisper ASR
  - Ajouter tests transcription
  - Tester gestion erreurs et streaming

#### 3. `daemon/bridge.py` - 0% → objectif 30%+ (1-2h)
- **Fichier test** : `tests/test_daemon_bridge.py` (existe partiellement)
- **Lignes non couvertes** : 283 lignes
- **Actions** :
  - Étendre tests bridge Zenoh/FastAPI
  - Tester connexion Zenoh (avec mocks)
  - Tester envoi/reception de commandes
  - Tester gestion erreurs

**Estimation totale** : 3-6 heures

---

### 🟡 Priorité Moyenne - Documentation (1-2h)

#### Liens MD restants (~139 liens)
- Majoritairement dans archives (non prioritaire)
- **Estimation** : ~30 min si on veut tout corriger

#### Documentation supplémentaire (optionnel)
- Mettre à jour `docs/guides_techniques/FAQ_TROUBLESHOOTING.md`
- Créer guide pour `dashboard_advanced.py` (optionnel, déjà bien testé)
- **Estimation** : 1-2 heures

---

### 🔵 Priorité Basse - Hardware (En Attente Robot)

- TODOs robot réel (3-4h) - Nécessite robot physique
- Module IO SDK (optionnel)

---

**Total estimé restant** : **3-8 heures** (sans hardware)

**Voir** : `docs/TACHES_A_FAIRE_CONSOLIDEES.md` pour détails complets

