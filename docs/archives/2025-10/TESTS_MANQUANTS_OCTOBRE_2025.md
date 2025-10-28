# 📋 TESTS MANQUANTS - OCTOBRE 2025

**Date :** 28 Octobre 2025  
**Analyse :** Tests existants vs modules à couvrir

---

## ✅ TESTS CRÉÉS AUJOURD'HUI (55 tests)

1. ✅ `tests/test_sdk_signatures_conformity.py` (10 tests)
2. ✅ `tests/test_global_config.py` (21 tests)
3. ✅ `tests/test_telemetry.py` (14 tests)
4. ✅ `tests/test_daemon_bridge.py` (10 tests)

**Total nouveaux tests :** +55 ✅

---

## 🚨 PRIORITÉ CRITIQUE (0% Coverage)

### ❌ 1. `dashboard_advanced.py` (0% - 288 lignes)

**Fichier à créer :** `tests/test_dashboard_advanced.py`

**Modules à tester :**
- BBIAAdvancedWebSocketManager
- Endpoints FastAPI avancés
- Gestion WebSocket
- Statistiques en temps réel

**Estimation :** 15-20 tests nécessaires  
**Temps :** 2-3 heures

---

## 🟡 PRIORITÉ HAUTE (<50% Coverage)

### ⏳ 2. `bbia_emotion_recognition.py` (33% - 138 lignes)

**Fichier à créer :** `tests/test_bbia_emotion_recognition_extended.py`

**Modules à tester :**
- `detect_faces()` - détection visages
- `analyze_facial_emotion()` - analyse émotions faciales
- `analyze_vocal_emotion()` - analyse vocale
- `fuse_emotions()` - fusion multimodale
- `get_emotion_statistics()` - statistiques

**Estimation :** 12-15 tests  
**Temps :** 2 heures

---

### ⏳ 3. `bbia_huggingface.py` (38% - 149 lignes)

**Fichier existe :** `tests/test_bbia_huggingface_chat.py` ✅  
**Fichier à créer :** `tests/test_bbia_huggingface_extended.py`

**Modules à tester (pas dans chat) :**
- Vision (CLIP, BLIP)
- Audio (Whisper)
- Cache modèles
- Méthodes d'analyse d'images
- Méthodes d'analyse audio

**Estimation :** 10-12 tests  
**Temps :** 2 heures

---

### ⏳ 4. `bbia_integration.py` (26% - 106 lignes)

**Fichier à créer :** `tests/test_bbia_integration.py`

**Modules à tester :**
- BBIAIntegration class
- Intégration modules BBIA
- Orchestration comportements
- Gestion contexte
- Liaisons entre modules

**Estimation :** 10-12 tests  
**Temps :** 2 heures

---

### ⏳ 5. `voice_whisper.py` (36% - 67 lignes)

**Fichier à créer :** `tests/test_voice_whisper_extended.py`

**Modules à tester :**
- Transcription audio
- Différentes langues
- Gestion erreurs audio
- Stream audio

**Estimation :** 8-10 tests  
**Temps :** 1-2 heures

---

### ⏳ 6. `vision_yolo.py` (28% - 99 lignes)

**Fichier à créer :** `tests/test_vision_yolo_extended.py`

**Modules à tester :**
- Détection objets YOLO
- Classification objets
- Gestion images
- Performance détection

**Estimation :** 10-12 tests  
**Temps :** 2 heures

---

### ⏳ 7. `bbia_awake.py` (8.70% - 21 lignes)

**Fichier existe :** `tests/test_bbia_awake.py` ✅  
**Fichier à améliorer :** Étendre test_bbia_awake.py

**Modules à tester :**
- Séquence complète réveil
- CLI arguments
- Gestion erreurs
- Initialisation

**Estimation :** 8-10 tests  
**Temps :** 1 heure

---

### ⏳ 8. `reachy_mini_backend.py` (30% - 287 lignes)

**Fichier existe :** `tests/test_reachy_mini_backend.py` ✅  
**Fichier à créer :** `tests/test_reachy_mini_backend_extended.py`

**Modules à tester (pas encore couverts) :**
- `get_current_head_pose()`
- `look_at_image()`
- `goto_target()` avancé
- `enable_motors()` / `disable_motors()`
- `enable_gravity_compensation()`
- `set_automatic_body_yaw()`
- `start_recording()` / `stop_recording()`
- `play_move()` / `async_play_move()`

**Estimation :** 15-20 tests  
**Temps :** 3-4 heures

---

## 📊 RÉSUMÉ

### Tests Manquants par Priorité

| Priorité | Module | Coverage | Lignes | Tests | Temps |
|----------|--------|----------|--------|-------|-------|
| 🚨 Critique | dashboard_advanced.py | 0% | 288 | 15-20 | 2-3h |
| 🟡 Haute | bbia_emotion_recognition.py | 33% | 138 | 12-15 | 2h |
| 🟡 Haute | bbia_huggingface.py | 38% | 149 | 10-12 | 2h |
| 🟡 Haute | bbia_integration.py | 26% | 106 | 10-12 | 2h |
| 🟡 Moyenne | voice_whisper.py | 36% | 67 | 8-10 | 1-2h |
| 🟡 Moyenne | vision_yolo.py | 28% | 99 | 10-12 | 2h |
| 🟢 Basse | bbia_awake.py | 8.70% | 21 | 8-10 | 1h |
| 🟢 Basse | reachy_mini_backend.py | 30% | 287 | 15-20 | 3-4h |

**Total :** ~100 tests à créer  
**Estimation totale :** 13-18 heures

---

## 🎯 RECOMMANDATIONS

### Phase 1 : Critique (Aujourd'hui - 2-3h)
1. ✅ Créer `test_dashboard_advanced.py` (288 lignes)

### Phase 2 : Haute Priorité (Semaine prochaine - 8h)
2. ✅ Créer `test_bbia_emotion_recognition_extended.py`
3. ✅ Créer `test_bbia_huggingface_extended.py`
4. ✅ Créer `test_bbia_integration.py`

### Phase 3 : Moyenne Priorité (Semaine d'après - 5h)
5. ✅ Créer `test_voice_whisper_extended.py`
6. ✅ Créer `test_vision_yolo_extended.py`

### Phase 4 : Amélioration (Selon besoin - 5h)
7. ✅ Étendre `test_bbia_awake.py`
8. ✅ Créer `test_reachy_mini_backend_extended.py`

---

## 🚀 COMMANDES POUR LANCER

```bash
# Tests critiques seulement (rapide)
pytest tests/test_dashboard_advanced.py -v

# Tests haute priorité (moyen)
pytest tests/test_bbia_emotion_recognition_extended.py tests/test_bbia_huggingface_extended.py tests/test_bbia_integration.py -v

# Tous les nouveaux tests
pytest tests/test_*_extended.py tests/test_dashboard_advanced.py tests/test_*_conformity.py -v

# Tests complets (long - à faire en fin)
pytest tests/ --cov=src/bbia_sim --cov-report=html
```

---

**Prochaine action :** Créer `test_dashboard_advanced.py` ✅

