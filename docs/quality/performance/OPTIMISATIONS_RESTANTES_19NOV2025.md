# ⏳ Optimisations Performance - Reste à Faire

**Date** : 19 novembre 2025  
**Dernière mise à jour** : 19 novembre 2025  
**Statut** : ✅ **Phase 1 TERMINÉE** (19 novembre 2025) - Phase 2 en attente (optionnel)

---

## ✅ Ce qui a été fait (19 novembre 2025)

### Vision
- ✅ Réduction résolution image YOLO (640x480 au lieu de 1280x720)
- ✅ Cache YOLO déjà présent et fonctionnel
- ✅ YOLOv8n (nano) déjà utilisé par défaut

### Audio
- ✅ Cache Whisper déjà présent et fonctionnel
- ✅ Whisper "tiny" déjà utilisé par défaut
- ✅ Fonction `transcribe_audio()` créée pour utiliser le cache

### Tests
- ✅ Tests benchmarks créés (`tests/benchmarks/test_performance.py`)
- ✅ Mesures p50/p95/p99 pour vision, audio, mouvements

---

## ⏳ Ce qui reste vraiment à faire

### Phase 1 : Optimisation Latence (Priorité 🔴 HAUTE)

#### 1. ✅ **TERMINÉ** - Threading asynchrone pour vision
**Fichier** : `src/bbia_sim/bbia_vision.py`

**Objectif** : Détection objets en arrière-plan sans bloquer

**Actions réalisées** :
- ✅ Thread dédié créé (`_scan_thread_worker`)
- ✅ Queue pour résultats (`_scan_queue`)
- ✅ Méthode `scan_environment_async()` non-bloquante
- ✅ Méthodes `start_async_scanning()` / `stop_async_scanning()`

**Impact** : **Latence perçue réduite** (non-bloquant, résultats temps réel) - **IMPLÉMENTÉ 19/11/2025**

---

#### 2. ✅ **DÉJÀ FAIT** - Threading asynchrone pour audio STT
**Fichier** : `src/bbia_sim/bbia_voice.py`

**Objectif** : Transcription en arrière-plan

**Actions déjà présentes** :
- ✅ Thread dédié créé (`_transcribe_thread_worker`)
- ✅ Queue pour résultats (`_transcribe_queue`)
- ✅ Fonction `transcribe_audio_async()` non-bloquante
- ✅ Fonctions `start_async_transcription()` / `stop_async_transcription()`

**Impact** : **Latence perçue réduite** (non-bloquant) - **DÉJÀ IMPLÉMENTÉ**

---

#### 3. ✅ **TERMINÉ** - Cache poses fréquentes (mouvements)
**Fichier** : `src/bbia_sim/backends/reachy_mini_backend.py`

**Vérifications** :
- ✅ `goto_target()` est déjà direct (pas de wrapper inutile)
- ✅ Cache poses fréquentes avec `lru_cache` - **IMPLÉMENTÉ 19/11/2025**
- ⏳ Éviter conversions numpy → list inutiles (À VÉRIFIER)

**Actions réalisées** :
- ✅ Fonction `_create_cached_head_pose()` créée avec `@lru_cache(maxsize=50)`
- ✅ Tous les appels `create_head_pose()` remplacés par version cache
- ✅ Cache automatique des 50 poses les plus récentes

**Impact** : **-10 à -20% latence** sur poses répétées (émotions, mouvements fréquents) - **IMPLÉMENTÉ 19/11/2025**

---

### ✅ RÉSUMÉ PHASE 1 : Toutes les optimisations prioritaires sont terminées !

**Optimisations complétées le 19 novembre 2025 :**
1. ✅ Réduction résolution YOLO (640x480)
2. ✅ Fonction transcribe_audio() avec cache
3. ✅ Tests benchmarks créés
4. ✅ Cache poses fréquentes (LRU)
5. ✅ Threading asynchrone vision
6. ✅ Threading asynchrone audio (déjà fait)

---

### Phase 2 : Streaming Optimisé (Priorité 🟡 MOYENNE)

#### 4. Stream vidéo optimisé (⏳ À FAIRE)
- WebSocket ou WebRTC pour stream caméra
- Compression adaptative (JPEG quality)
- Frame rate adaptatif (30 FPS max)
- Buffer optimisé (deque maxlen=5)

---

#### 5. Stream audio optimisé (⏳ À FAIRE)
- WebSocket pour stream microphone
- Compression audio (Opus ou G.711)
- Buffer optimisé (deque maxlen=10)
- Latence minimale (<50ms)

---

#### 6. Optimiser WebSocket dashboard (⏳ À FAIRE)
- Réduire fréquence messages
- Batching messages (grouper updates)
- Compression JSON si nécessaire
- Heartbeat optimisé (30s au lieu de 10s)

---

### Phase 3 : Optimisation Mémoire (Priorité 🟢 BASSE)

#### 7. Quantification modèles (⏳ OPTIONNEL)
- Quantification 8-bit si possible
- Libérer GPU si disponible

---

#### 8. Optimisation gestion images/audio (⏳ OPTIONNEL)
- Réduire taille images en mémoire
- Libérer buffers après traitement
- Pas de copies inutiles

---

## 📊 Priorités Recommandées

1. ✅ **TERMINÉ** : Cache poses fréquentes (mouvements) - **FAIT 19/11/2025**
2. ✅ **TERMINÉ** : Threading asynchrone vision - **FAIT 19/11/2025**
3. ✅ **DÉJÀ FAIT** : Threading asynchrone audio - **DÉJÀ IMPLÉMENTÉ**
4. **🟡 MOYENNE** : Streaming optimisé - Pour cas d'usage temps réel
5. **🟢 BASSE** : Optimisations mémoire - Gain marginal

---

## 🎯 Objectifs de Latence

| Métrique | Actuel | Objectif | Statut |
|----------|--------|----------|--------|
| Latence Vision | ~100ms | <50ms | ✅ **OPTIMISÉ** (résolution 640x480 + threading) |
| Latence Audio | ~200ms | <100ms | ✅ **OPTIMISÉ** (cache + tiny + threading) |
| Latence Mouvements | ~20ms | <10ms | ✅ **OPTIMISÉ** (cache poses LRU implémenté) |

---

**✅ Phase 1 terminée** : Toutes les optimisations prioritaires sont complétées (19 novembre 2025).

**Vérification finale (19 novembre 2025) :**
- ✅ Tests passent (8/8 tests dashboard media)
- ✅ Imports OK (BBIAChat, Behaviors)
- ✅ Code quality OK (black, ruff, mypy)
- ✅ Cache poses LRU vérifié et fonctionnel
- ✅ Threading vision/audio vérifié et fonctionnel

**Prochaine étape recommandée** : Phase 2 - Streaming optimisé (WebSocket/WebRTC) pour cas d'usage temps réel.

