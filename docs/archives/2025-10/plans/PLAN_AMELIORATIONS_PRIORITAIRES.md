# 🔧 PLAN D'AMÉLIORATIONS PRIORITAIRES - BBIA-SIM

**Date :** octobre 2025
**Objectif :** Améliorer la qualité, la couverture de tests et corriger les points critiques

---

## 🚨 PRIORITÉ CRITIQUE (0% Coverage - À corriger immédiatement)

### 1. `dashboard_advanced.py` - 0% Coverage (288 lignes non couvertes)

**Problème :** Module non testé du tout

**Impact :** Dashboard avancé utilisé en production mais non testé

**Action :**
- Créer `tests/test_dashboard_advanced.py`
- Tester les endpoints FastAPI
- Tester la gestion WebSocket
- Tester la logique de dashboard

**Estimation :** 2-3 heures

---

### 2. `daemon/bridge.py` - 0% Coverage (283 lignes non couvertes)

**Problème :** Bridge Zenoh/FastAPI non testé

**Impact :** Point d'intégration critique non testé

**Action :**
- Créer `tests/test_daemon_bridge.py`
- Tester connexion Zenoh (avec mocks)
- Tester envoi/reception de commandes
- Tester gestion erreurs

**Estimation :** 2-3 heures

---

### 3. `telemetry.py` - 0% Coverage (63 lignes non couvertes)

**Problème :** Module télémétrie non testé

**Impact :** Télémétrie utilisée partout mais non testée

**Action :**
- Créer `tests/test_telemetry.py`
- Tester collecte de métriques
- Tester agrégation de données
- Tester export de métriques

**Estimation :** 1-2 heures

---

### 4. `global_config.py` - 0% Coverage (49 lignes non couvertes)

**Problème :** Configuration globale non testée

**Impact :** Configuration utilisée partout mais non validée

**Action :**
- Créer `tests/test_global_config.py`
- Tester chargement configuration
- Tester validation des paramètres
- Tester gestion d'erreurs

**Estimation :** 1 heure

---

## 🟡 PRIORITÉ HAUTE (Coverage < 50%)

### 5. `reachy_mini_backend.py` - 30.17% Coverage (287 lignes non couvertes)

**Problème :** Backend SDK officiel mal testé

**Impact :** Backend critique pour conformité SDK

**Action :**
- Créer tests pour méthodes SDK avancées
- Tester méthodes `get_current_head_pose()`
- Tester méthodes `look_at_image()`
- Tester méthodes `goto_target()`
- Tester gestion moteurs (enable/disable)
- Tester compensation gravité
- Tester enregistrement mouvement

**Fichier à créer :** `tests/test_reachy_mini_backend_extended.py`

**Estimation :** 3-4 heures

---

### 6. `bbia_emotion_recognition.py` - 33.01% Coverage (138 lignes non couvertes)

**Problème :** Module reconnaissance émotions mal testé

**Impact :** Module IA critique pour interactions

**Action :**
- Créer tests pour analyse faciale
- Tester analyse vocale
- Tester fusion émotions
- Tester détection visages
- Tester historique émotions

**Fichier à créer :** `tests/test_bbia_emotion_recognition_extended.py`

**Estimation :** 2-3 heures

---

### 7. `bbia_huggingface.py` - 37.92% Coverage (149 lignes non couvertes)

**Problème :** Module Hugging Face mal testé malgré tests chat existants

**Impact :** Intégration ML pas complètement testée

**Action :**
- Étendre tests existants `test_bbia_huggingface_chat.py`
- Ajouter tests pour méthodes vision (CLIP, BLIP)
- Tester méthodes audio (Whisper)
- Tester méthodes NLP
- Tester gestion cache modèles

**Fichier à améliorer :** `tests/test_bbia_huggingface_chat.py` (existe déjà)

**Estimation :** 2-3 heures

---

### 8. `bbia_integration.py` - 26.39% Coverage (106 lignes non couvertes)

**Problème :** Module intégration mal testé

**Impact :** Orchestration des modules BBIA non testée

**Action :**
- Créer `tests/test_bbia_integration.py`
- Tester intégration modules
- Tester orchestration comportements
- Tester gestion contexte

**Estimation :** 2-3 heures

---

### 9. `voice_whisper.py` - 35.58% Coverage (67 lignes non couvertes)

**Problème :** Module Whisper mal testé

**Impact :** Reconnaissance vocale non validée

**Action :**
- Créer `tests/test_voice_whisper_extended.py`
- Tester transcription audio
- Tester différentes langues
- Tester gestion erreurs audio

**Estimation :** 2-3 heures

---

### 10. `vision_yolo.py` - 27.74% Coverage (99 lignes non couvertes)

**Problème :** Module YOLO mal testé

**Impact :** Détection objets non validée

**Action :**
- Créer `tests/test_vision_yolo_extended.py`
- Tester détection objets
- Tester classification
- Tester gestion images

**Estimation :** 2-3 heures

---

## 🔧 TODOs À IMPLÉMENTER (Points techniques)

### 1. `daemon/app/routers/ecosystem.py`

**TODOs trouvés :**
```python
# Ligne 121: uptime="00:00:00",  # TODO: Calculer le temps réel
# Ligne 124: active_connections=0,  # TODO: Compter les connexions WebSocket
# Ligne 408: # TODO: Implémenter la logique de démarrage de démo
```

**Action :**
- Implémenter calcul uptime réel
- Implémenter comptage connexions WebSocket actives
- Implémenter logique démarrage démo

**Estimation :** 1-2 heures

---

### 2. `backends/reachy_backend.py`

**TODOs trouvés :**
```python
# Ligne 52: # TODO: Implémenter la vraie connexion Reachy
# Ligne 71: # TODO: Implémenter la vraie déconnexion Reachy
# Ligne 104: # TODO: Envoyer la commande au robot réel
# Ligne 127: # TODO: Synchroniser avec le robot réel
# Ligne 165: # TODO: Implémenter l'envoi de commandes réelles
```

**Action :**
- Implémenter connexion réelle au robot
- Implémenter envoi commandes réelles
- Implémenter synchronisation état

**Estimation :** 3-4 heures (nécessite accès robot physique)

---

## 📊 RÉSUMÉ PAR PRIORITÉ

| Priorité | Modules | Lignes Non Couvertes | Estimation |
|----------|---------|---------------------|------------|
| 🚨 Critique (0%) | dashboard_advanced.py | 288 | 2-3h |
| 🚨 Critique (0%) | daemon/bridge.py | 283 | 2-3h |
| 🚨 Critique (0%) | telemetry.py | 63 | 1-2h |
| 🚨 Critique (0%) | global_config.py | 49 | 1h |
| 🟡 Haute (<50%) | reachy_mini_backend.py | 287 | 3-4h |
| 🟡 Haute (<50%) | bbia_emotion_recognition.py | 138 | 2-3h |
| 🟡 Haute (<50%) | bbia_huggingface.py | 149 | 2-3h |
| 🟡 Haute (<50%) | bbia_integration.py | 106 | 2-3h |
| 🟡 Haute (<50%) | voice_whisper.py | 67 | 2-3h |
| 🟡 Haute (<50%) | vision_yolo.py | 99 | 2-3h |
| 🔧 Technique | TODOs ecosystem.py | - | 1-2h |
| 🔧 Technique | TODOs reachy_backend.py | - | 3-4h |

**Total estimé :** 23-35 heures

---

## 🎯 PLAN D'EXÉCUTION RECOMMANDÉ

### Semaine 1 : Modules critiques (0% coverage)

**Jour 1 :**
- [ ] `global_config.py` - Tests et implémentation (1h)
- [ ] `telemetry.py` - Tests et implémentation (2h)
- [ ] TODOs `ecosystem.py` - Implémentation (2h)

**Jour 2 :**
- [ ] `daemon/bridge.py` - Tests mocks (3h)

**Jour 3 :**
- [ ] `dashboard_advanced.py` - Tests endpoints (3h)

### Semaine 2 : Modules haute priorité (<50% coverage)

**Jour 1 :**
- [ ] `reachy_mini_backend.py` - Tests avancés (4h)

**Jour 2 :**
- [ ] `bbia_emotion_recognition.py` - Tests étendus (3h)

**Jour 3 :**
- [ ] `bbia_huggingface.py` - Tests enrichis (3h)

**Jour 4 :**
- [ ] `bbia_integration.py` - Tests intégration (2h)

**Jour 5 :**
- [ ] `voice_whisper.py` - Tests audio (2h)
- [ ] `vision_yolo.py` - Tests vision (2h)

---

## 📋 COMMANDES VALIDATION

### Tester progressivement

```bash
# Test module spécifique avec coverage
pytest tests/test_global_config.py --cov=src.bbia_sim.global_config --cov-report=term-missing

pytest tests/test_telemetry.py --cov=src.bbia_sim.telemetry --cov-report=term-missing

pytest tests/test_daemon_bridge.py --cov=src.bbia_sim.daemon.bridge --cov-report=term-missing

pytest tests/test_dashboard_advanced.py --cov=src.bbia_sim.dashboard_advanced --cov-report=term-missing
```

### Vérifier coverage global

```bash
pytest tests/ --cov=src/bbia_sim --cov-report=term-missing --cov-fail-under=60
```

---

## 🎯 OBJECTIF FINAL

**Couverture cible :** ≥ 70%
**Modules critiques :** 100% couverts
**TODOs implémentés :** Tous résolus
**Tests passent :** 100%

---

**Prochaines actions immédiates :**
1. ✅ Créer tests pour `global_config.py`
2. ✅ Créer tests pour `telemetry.py`
3. ✅ Implémenter TODOs `ecosystem.py`
4. ✅ Créer tests pour `daemon/bridge.py`
5. ✅ Créer tests pour `dashboard_advanced.py`

**Souhaitez-vous que je commence par les modules critiques (0% coverage) ?** 🚀

