# 🔍 AUDIT COMPLET RAM - FICHIERS À OPTIMISER (Novembre 2025)

**Date** : Oct 25 / Nov 25
**Objectif** : Liste complète de tous les fichiers/tests consommant beaucoup de RAM, priorisés par urgence

---

## 📊 RÉSUMÉ

| Priorité | Nombre | Gain RAM Estimé |
|----------|--------|-----------------|
| **🔴 TRÈS URGENT** | 8 fichiers | **40-60%** |
| **🟡 PRIORITAIRE** | 10 fichiers | **30-50%** |
| **🟢 MOYEN** | 7 fichiers | **20-40%** |
| **TOTAL** | **25 fichiers** | **~35-50% global** |

---

## 🔴 PRIORITÉ TRÈS URGENTE (Gain RAM : 40-60%)

### 1. **`src/bbia_sim/bbia_huggingface.py`** ⚠️ **TRÈS ÉNERGIVORE**

**Problèmes identifiés :**
- Classe `BBIAHuggingFace` charge modèles Hugging Face (LLM = 2-8 GB RAM)
- `self.models` et `self.processors` stockent TOUS les modèles en mémoire
- `load_model()` vérifie cache mais peut encore être optimisé
- Chat LLM chargé même si pas utilisé (lazy loading partiel)
- Pipelines transformers chargés pour chaque type (vision, audio, NLP)
- Pas de limite de modèles en mémoire simultanément
- Pas de déchargement automatique après inactivité

**Optimisations proposées :**
1. **Lazy loading strict** : Ne charger LLM chat que si `chat()` appelé
2. **Limite modèles** : Max 3-4 modèles en mémoire, décharger LRU
3. **Déchargement auto** : Timer d'inactivité (5 min) → `unload_model()`
4. **Cache partagé global** : Singleton ou cache module-level (pas par instance)
5. **Quantization** : Utiliser `torch.int8` pour modèles non critiques

**Gain estimé** : **-50-70% RAM** (2-8 GB → 1-3 GB)

**Complexité** : ⭐⭐⭐ (modéré-élevé)

---

### 2. **`src/bbia_sim/bbia_vision.py`** ⚠️ **ÉNERGIVORE**

**Problèmes identifiés :**
- `BBIAVision()` crée instances YOLO + MediaPipe même si pas utilisé
- `scan_environment()` crée objets simulés à chaque appel (lignes 823-872)
- Détections stockées dans `self.objects_detected` et `self.faces_detected` sans limite
- MediaPipe Face chargé à l'init même si pas de caméra

**Optimisations proposées :**
1. **Lazy loading YOLO/MediaPipe** : Charger uniquement si `scan_environment()` appelé avec vraie caméra
2. **Limite historique détections** : Max 50 objets/visages, utiliser `deque(maxlen=50)`
3. **Cache objets simulés** : Réutiliser liste simulée au lieu de recréer à chaque appel
4. **Singleton BBIAVision** : Éviter créations multiples (partager même instance)

**Gain estimé** : **-40-50% RAM** (YOLO ~200MB, MediaPipe ~100MB)

**Complexité** : ⭐⭐ (modéré)

---

### 3. **`src/bbia_sim/vision_yolo.py`** ⚠️ **ÉNERGIVORE**

**Problèmes identifiés :**
- Cache global YOLO mais pas de limite taille
- MediaPipe cache singleton OK mais peut être optimisé
- `detect_objects()` fait `.cpu().numpy()` à chaque détection (GPU→CPU transfert)

**Optimisations proposées :**
1. **Cache LRU** : Limiter à 2 modèles YOLO max (n, s) → décharger m, l, x
2. **Batch processing** : Grouper détections si multiples images
3. **Optimisation GPU** : Garder résultats GPU si pas besoin CPU immédiat

**Gain estimé** : **-30-40% RAM** (YOLO cache optimisé)

**Complexité** : ⭐⭐ (modéré)

---

### 4. **`src/bbia_sim/voice_whisper.py`** ⚠️ **ÉNERGIVORE**

**Problèmes identifiés :**
- Cache Whisper global mais pas de limite
- `transcribe_streaming()` accumule `audio_buffer` sans limite claire
- `transcribe_audio()` crée fichiers temporaires `.wav` à chaque transcription
- VAD modèle chargé même si `enable_vad=False`

**Optimisations proposées :**
1. **Limite cache Whisper** : Max 2 modèles (tiny, base) → décharger medium/large
2. **Limite audio_buffer** : `deque(maxlen=10)` pour éviter accumulation
3. **Pool fichiers temp** : Réutiliser fichiers `.wav` au lieu de créer/supprimer
4. **Lazy VAD** : Charger VAD uniquement si `enable_vad=True` ET utilisé

**Gain estimé** : **-35-45% RAM** (Whisper ~500MB-2GB selon taille)

**Complexité** : ⭐⭐ (modéré)

---

### 5. **`src/bbia_sim/dashboard_advanced.py`** ⚠️ **ÉNERGIVORE**

**Problèmes identifiés :**
- `__init__()` crée `BBIAVision()` et `BBIAEmotions()` à chaque instance
- `metrics_history` accumule sans limite (`max_history=1000` mais pas appliqué correctement)
- `current_metrics` recréé à chaque mise à jour
- WebSocket connections stockées sans nettoyage automatique

**Optimisations proposées :**
1. **Singleton managers** : Une seule instance `BBIAVision` partagée
2. **Limite historique** : `deque(maxlen=1000)` au lieu de liste
3. **Mise à jour in-place** : Modifier `current_metrics` au lieu de recréer
4. **Nettoyage connexions** : Timer pour fermer connexions inactives (>5 min)

**Gain estimé** : **-40-50% RAM** (éviter duplications instances)

**Complexité** : ⭐⭐ (modéré)

---

### 6. **`src/bbia_sim/daemon/ws/telemetry.py`** ⚠️ **ÉNERGIVORE**

**Problèmes identifiés :**
- `_broadcast_loop()` génère JSON à chaque tick (10Hz = 10 fois/seconde)
- `_generate_telemetry_data()` crée nouveaux dicts à chaque appel
- Pas de pooling/reuse de structures JSON
- Connexions WebSocket accumulées sans limite

**Optimisations proposées :**
1. **Template JSON réutilisé** : Modifier dict existant au lieu de recréer
2. **Compression messages** : Utiliser `gzip` ou `zlib` si messages > 1KB
3. **Batching** : Envoyer 5-10 mesures en une fois si latence acceptable
4. **Limite connexions** : Max 10 connexions simultanées, rejeter nouvelles

**Gain estimé** : **-30-40% RAM** (moins d'allocations JSON)

**Complexité** : ⭐ (facile)

---

### 7. **`src/bbia_sim/backends/reachy_mini_backend.py`** ⚠️ **ÉNERGIVORE**

**Problèmes identifiés :**
- Watchdog thread actif même en simulation (consomme RAM pour rien)
- `joint_mapping`, `joint_limits` recréés à chaque instance (dicts statiques)
- Historique positions/joints non limité (si tracking activé)

**Optimisations proposées :**
1. **Désactiver watchdog en sim** : Skip si `use_sim=True`
2. **Constantes module-level** : `JOINT_MAPPING`, `JOINT_LIMITS` partagés
3. **Limite historique** : `deque(maxlen=100)` si tracking activé

**Gain estimé** : **-20-30% RAM** (threads + dicts constants)

**Complexité** : ⭐ (facile)

---

### 8. **`src/bbia_sim/sim/simulator.py`** ⚠️ **ÉNERGIVORE**

**Problèmes identifiés :**
- `_run_headless_simulation()` boucle infinie si `duration=None`
- MuJoCo modèle chargé en mémoire (peut être gros selon scène)
- Pas de déchargement modèle après arrêt simulation

**Optimisations proposées :**
1. **Limite steps obligatoire** : Forcer `duration` ou max 10000 steps
2. **Déchargement modèle** : `del self.model, self.data` après arrêt
3. **Lazy chargement scènes** : Charger scène uniquement si nécessaire

**Gain estimé** : **-25-35% RAM** (éviter boucles infinies + déchargement)

**Complexité** : ⭐ (facile)

---

## 🟡 PRIORITÉ PRIORITAIRE (Gain RAM : 30-50%)

### 9. **`src/bbia_sim/bbia_emotion_recognition.py`** 🟡

**Problèmes :**
- Cache pipelines transformers mais pas de limite
- MediaPipe Face chargé même si pas utilisé
- Historique émotions sans limite (`temporal_window_size=5` mais accumule)

**Optimisations :**
1. Limiter cache à 2 pipelines max
2. Lazy MediaPipe : Charger uniquement si détection visage demandée
3. `deque(maxlen=5)` pour historique

**Gain** : **-30-40% RAM**

---

### 10. **`src/bbia_sim/bbia_behavior.py`** 🟡

**Problèmes :**
- `BBIABehaviorManager` crée `BBIAVision()` et utilise `BBIAEmotions()`
- Queue comportements sans limite
- Logs écrits dans fichier (`log/bbia.log`) qui peut grossir

**Optimisations :**
1. Partager instance `BBIAVision` depuis singleton
2. Limiter queue : `Queue(maxsize=50)`
3. Rotation logs : Limiter taille fichier log (10MB max)

**Gain** : **-25-35% RAM**

---

### 11. **`src/bbia_sim/bbia_memory.py`** 🟡

**Problèmes :**
- Charge/sauvegarde JSON complet à chaque appel
- Pas de limite taille conversation_history
- Écriture fichier synchrones (bloquantes)

**Optimisations :**
1. Limiter historique : Max 1000 messages, supprimer anciens
2. Écriture asynchrone : `asyncio` ou thread pour I/O
3. Compression JSON : Utiliser `gzip` si > 1MB

**Gain** : **-20-30% RAM**

---

### 12. **`src/bbia_sim/daemon/app/routers/state.py`** 🟡

**Problèmes :**
- `ws_full_state()` génère JSON complet à chaque tick (10Hz)
- `get_full_state()` crée nouveaux Pydantic models à chaque appel

**Optimisations :**
1. Réutiliser modèle Pydantic : Modifier in-place si possible
2. Différentiel : Envoyer seulement changements (delta)
3. Compression WebSocket : Activer `permessage-deflate`

**Gain** : **-25-35% RAM**

---

### 13. **`src/bbia_sim/daemon/simulation_service.py`** 🟡

**Problèmes :**
- `_run_headless_simulation()` boucle à 60Hz même si pas nécessaire
- MuJoCoSimulator instance gardée même après arrêt

**Optimisations :**
1. Fréquence adaptative : 10-20Hz suffisant pour sim headless
2. Déchargement simulateur : `del self.simulator` après `stop_simulation()`

**Gain** : **-15-25% RAM**

---

### 14. **`src/bbia_sim/daemon/ws/__init__.py`** 🟡

**Problèmes :**
- `_generate_telemetry_data()` crée dicts à chaque appel
- Pas de pooling connexions WebSocket

**Optimisations :**
1. Template dict réutilisé
2. Limite connexions : Max 10 simultanées

**Gain** : **-20-30% RAM**

---

### 15. **`src/bbia_sim/telemetry.py`** 🟡

**Problèmes :**
- `TelemetryCollector` accumule données sans limite
- Écriture fichiers synchrones

**Optimisations :**
1. Limite données : Max 10000 steps, supprimer anciens
2. Écriture asynchrone : Thread ou `asyncio` pour I/O

**Gain** : **-20-30% RAM**

---

### 16. **`src/bbia_sim/bbia_audio.py`** 🟡

**Problèmes :**
- Cache `_cwd_cache` et `_temp_roots_cache` déjà optimisé ✅
- Mais `enregistrer_audio()` crée buffers audio sans limite

**Optimisations :**
1. Limiter taille buffer : Max 10s audio en mémoire
2. Streaming si > 5s : Écrire fichier progressivement

**Gain** : **-15-25% RAM**

---

### 17. **`src/bbia_sim/daemon/app/routers/ecosystem.py`** 🟡

**Problèmes :**
- `get_active_connections()` fait imports dynamiques à chaque appel
- Pas de cache résultat

**Optimisations :**
1. Cache résultat : TTL 1s (évite imports répétés)
2. Imports module-level : Déplacer en haut du fichier

**Gain** : **-10-20% RAM**

---

### 18. **`src/bbia_sim/bbia_voice_advanced.py`** 🟡

**Problèmes :**
- Crée instances TTS même si pas utilisé
- Cache pyttsx3 déjà optimisé ✅

**Optimisations :**
1. Lazy loading TTS : Charger uniquement si `synthesize()` appelé

**Gain** : **-10-20% RAM**

---

## 🟢 PRIORITÉ MOYENNE (Gain RAM : 20-40%)

### 19. **`src/bbia_sim/bbia_idle_animations.py`** 🟢

**Problèmes :**
- Threads animations actives même si robot inactif
- Pas de nettoyage automatique threads

**Optimisations :**
1. Désactiver animations si robot idle > 30s
2. Nettoyage threads : `threading.enumerate()` et join timeout

**Gain** : **-15-25% RAM**

---

### 20. **`src/bbia_sim/daemon/bridge.py`** 🟢

**Problèmes :**
- Buffer messages sans limite
- WebSocket connections multiples

**Optimisations :**
1. Limite buffer : `deque(maxlen=100)`
2. Limite connexions : Max 5 simultanées

**Gain** : **-15-25% RAM**

---

### 21. **`src/bbia_sim/daemon/app/routers/move.py`** 🟢

**Problèmes :**
- Crée nouvelles poses matrices à chaque appel
- Pas de réutilisation matrices

**Optimisations :**
1. Pool matrices : Réutiliser `np.eye(4)` avec modifications in-place
2. Cache poses fréquentes : Lookup table pour poses communes

**Gain** : **-10-20% RAM**

---

### 22. **`src/bbia_sim/dashboard.py`** 🟢

**Problèmes :**
- Instances multiples si plusieurs dashboards
- Historique métriques sans limite

**Optimisations :**
1. Singleton dashboard
2. Limite historique : `deque(maxlen=500)`

**Gain** : **-10-20% RAM**

---

### 23. **`tests/test_huggingface_latency.py`** 🟢

**Problèmes :**
- Charge vrais modèles LLM même en tests
- Mesure mémoire peak (consomme beaucoup)

**Optimisations :**
1. Utiliser mocks par défaut (`BBIA_FORCE_MOCK_MODELS=1`)
2. Marquer `@pytest.mark.heavy` (déjà fait ✅)

**Gain** : **-80-90% RAM** (tests uniquement)

---

### 24. **`tests/test_vision_yolo_comprehensive.py`** 🟢

**Problèmes :**
- Charge vrais modèles YOLO en tests
- Boucles de tests longues

**Optimisations :**
1. Mocks par défaut
2. Réduire itérations tests (déjà optimisé ✅)

**Gain** : **-70-80% RAM** (tests uniquement)

---

### 25. **`tests/e2e/test_e2e_full_interaction_loop.py`** 🟢

**Problèmes :**
- Crée instances BBIAVision, BBIAEmotions, BBIABehaviorManager
- Tests E2E consomment beaucoup

**Optimisations :**
1. Fixtures partagées : `@pytest.fixture(scope="module")`
2. Réutiliser instances entre tests

**Gain** : **-40-50% RAM** (tests uniquement)

---

## 📋 PLAN D'ACTION RECOMMANDÉ

### Phase 1 : Quick Wins (1-2 jours)
1. ✅ `reachy_mini_backend.py` : Constantes module-level
2. ✅ `simulator.py` : Limite steps + déchargement
3. ✅ `telemetry.py` : Template JSON réutilisé
4. ✅ `ecosystem.py` : Cache imports

### Phase 2 : Optimisations Modérées (3-5 jours)
1. ✅ `bbia_vision.py` : Lazy loading YOLO/MediaPipe
2. ✅ `bbia_behavior.py` : Singleton BBIAVision
3. ✅ `bbia_memory.py` : Limite historique + async I/O
4. ✅ `dashboard_advanced.py` : Singleton + deque historique

### Phase 3 : Optimisations Avancées (1-2 semaines)
1. ✅ `bbia_huggingface.py` : Lazy LLM + LRU cache
2. ✅ `voice_whisper.py` : Pool fichiers temp + limite buffer
3. ✅ `vision_yolo.py` : Cache LRU + batch processing
4. ✅ `bbia_emotion_recognition.py` : Lazy MediaPipe + limite cache

---

## 💡 STRATÉGIES GÉNÉRALES

### 1. **Singleton Pattern**
- `BBIAVision`, `BBIAHuggingFace` : Une seule instance partagée
- Évite chargements multiples modèles

### 2. **Lazy Loading**
- Ne charger modèles que si utilisés
- YOLO → uniquement si caméra réelle
- LLM → uniquement si `chat()` appelé

### 3. **Limites Mémoire**
- `deque(maxlen=N)` au lieu de listes infinies
- LRU cache pour modèles (max 2-3)
- Timeout inactivité → déchargement

### 4. **Réutilisation Structures**
- Templates JSON réutilisés (modification in-place)
- Pool matrices numpy
- Pool fichiers temporaires

### 5. **Compression**
- WebSocket : `permessage-deflate`
- JSON > 1KB : `gzip`
- Fichiers logs : Rotation + compression

---

## 🎯 GAIN TOTAL ESTIMÉ

| Catégorie | Fichiers | Gain RAM |
|-----------|----------|----------|
| **Très Urgent** | 8 | **-40-60%** |
| **Prioritaire** | 10 | **-30-50%** |
| **Moyen** | 7 | **-20-40%** |
| **TOTAL** | **25** | **~35-50% RAM globale** |

**Impact attendu :** Réduction de **1-3 GB RAM** selon configuration

---

**Dernière mise à jour :** Oct 25 / Nov 25

