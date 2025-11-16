# ✅ Checklist Validation Hardware - Oct / No2025025025025025

**Objectif :** Valider BBIA-SIM sur robot Reachy Mini physique  
**Date cible :** Oct / No2025025025025025  
**Durée estimée :** 2-3 semaines à plein temps

---

## 📦 Phase 1 : Réception & Installation (Semaine 1)

### Réception Robot

- [ ] **Robot Reachy Mini reçu**
  - Vérifier colis complet (robot + accessoires)
  - Inspecter état physique (pas de dommages)
  - Photographier déballage pour documentation

- [ ] **Vérification hardware**
  - Batterie chargée ou alimentation USB-C connectée
  - LED d'alimentation allumée
  - Wi-Fi accessible (vérifier SSID réseau)
  - Robot répond au ping réseau

### Installation SDK

- [ ] **SDK Reachy Mini installé**

  ```bash
  pip install reachy-mini-motor-controller
  # ou selon version SDK officielle
  ```

- [ ] **Vérification connexion SDK**

  ```bash
  python -c "from reachy_mini_motor_controller import ReachyMini; import logging; robot = ReachyMini(); logging.info(robot.is_connected)"
  ```

- [ ] **Test basique SDK**

  ```bash
  python examples/demo_reachy_mini_corrigee.py --quick
  ```

### Configuration Réseau

- [ ] **Wi-Fi configuré**
  - Robot connecté au même réseau que PC
  - IP robot connue (vérifier avec `reachy_mini` ou scan réseau)
  - Ports réseau ouverts (vérifier firewall)

- [ ] **Test connexion réseau**

  ```bash
  ping <robot_ip>
  # ou
  curl http://<robot_ip>:8080/api/state  # si API HTTP disponible
  ```

---

## 🧪 Phase 2 : Tests Hardware (Semaine 1-2)

### Activation Tests Hardware

- [ ] **Variables d'environnement configurées**

  ```bash
  export SKIP_HARDWARE_TESTS=0
  export REACHY_REAL=1
  export BBIA_ROBOT_IP=<robot_ip>  # si nécessaire
  ```

- [ ] **Tests hardware activés**

  ```bash
  SKIP_HARDWARE_TESTS=0 pytest tests/test_reachy_mini_backend.py::TestReachyMiniBackendReal -v
  ```

### Tests Conformité SDK

- [ ] **Test connexion réelle**

  ```bash
  SKIP_HARDWARE_TESTS=0 pytest tests/test_reachy_mini_backend.py::TestReachyMiniBackendReal::test_real_connection -v
  ```

- [ ] **Test contrôle joints**

  ```bash
  SKIP_HARDWARE_TESTS=0 pytest tests/test_reachy_mini_backend.py::TestReachyMiniBackendReal::test_real_joint_control -v
  ```

- [ ] **Dry-run complet**

  ```bash
  python scripts/hardware_dry_run_reachy_mini.py --output-dir artifacts --duration 30
  ```

### Tests Modules BBIA

- [ ] **Vision (caméra SDK)**

  ```bash
  SKIP_HARDWARE_TESTS=0 pytest tests/test_camera_sdk_latency_real.py -v
  python examples/demo_vision_ok.py --backend reachy_mini
  ```

- [ ] **Audio (microphones)**

  ```bash
  python examples/demo_voice_ok.py --backend reachy_mini
  python examples/demo_chat_bbia.py --backend reachy_mini
  ```

- [ ] **Mouvements (articulations)**

  ```bash
  python examples/demo_emotion_ok.py --backend reachy_mini --emotion happy --duration 5
  python examples/demo_behavior_ok.py --backend reachy_mini --behavior greeting
  ```

---

## 📊 Phase 3 : Mesures Performance (Semaine 2)

### Latence Mesurée

- [ ] **Latence joints (p50/p95)**

  ```bash
  # Mesurer latence set_joint_pos()
  python scripts/bbia_performance_benchmarks.py --jsonl artifacts/latency_reachy_mini.csv
  ```

  - Cible p50 : < 20ms
  - Cible p95 : < 50ms

- [ ] **Latence caméra SDK**

  ```bash
  SKIP_HARDWARE_TESTS=0 pytest tests/test_camera_sdk_latency_real.py::test_camera_sdk_latency_and_fps_placeholder -v
  ```

  - Cible FPS : 30 FPS stable
  - Cible latence : < 33ms par frame

- [ ] **Latence audio (microphones)**

  ```bash
  python examples/demo_voice_ok.py --backend reachy_mini --measure-latency
  ```

  - Cible latence STT : < 500ms

### Watchdog & Sécurité

- [ ] **Test watchdog timeout**

  ```bash
  SKIP_HARDWARE_TESTS=0 pytest tests/test_watchdog_monitoring.py::test_watchdog_timeout_triggers_emergency_stop_real -v
  ```

  - Vérifier que timeout > 2s déclenche `emergency_stop()`

- [ ] **Test emergency stop**

  ```bash
  python -c "from bbia_sim.robot_factory import RobotFactory; import logging; r = RobotFactory.create_backend('reachy_mini', use_sim=False); r.connect(); r.wake_up(); r.emergency_stop(); logging.info('Emergency stop OK')"
  ```

---

## 🎬 Phase 4 : Démos Vidéo (Semaine 2-3)

### Préparation Démos

- [ ] **Setup enregistrement**
  - Caméra externe positionnée (ou capture écran)
  - Éclairage correct
  - Audio clair (micro externe si nécessaire)
  - Arrière-plan neutre

### 5 Démos Obligatoires

- [ ] **Démo 1 : Vision + Détection Visage + Émotions** (30-60s)
  - Robot détecte visage
  - Analyse émotions (happy/sad/excited)
  - Réaction robotique correspondante
  - **Fichier :** `assets/videos/demo_vision_emotions_reachy_mini.mp4`

- [ ] **Démo 2 : Conversation Vocale Française** (30-60s)
  - Reconnaissance vocale (Whisper)
  - Réponse intelligente (LLM ou fallback)
  - Synthèse vocale (pyttsx3 ou TTS)
  - **Fichier :** `assets/videos/demo_conversation_vocale_reachy_mini.mp4`

- [ ] **Démo 3 : Head Tracking + Regard Fluide** (30-60s)
  - Suivi objet/personne
  - Mouvements tête fluides
  - Antennes expressives
  - **Fichier :** `assets/videos/demo_head_tracking_reachy_mini.mp4`

- [ ] **Démo 4 : Émotions Expressives** (30-60s)
  - Transition émotions (happy → sad → excited)
  - Animations fluides articulations
  - Synchronisation mouvements
  - **Fichier :** `assets/videos/demo_emotions_expressives_reachy_mini.mp4`

- [ ] **Démo 5 : Scénario Complet "Réveil → Conversation → Action"** (60-90s)
  - Réveil (`wake_up()`)
  - Conversation interactive
  - Action déclenchée (ex: "regarde-moi" → head tracking)
  - Retour veille (`goto_sleep()`)
  - **Fichier :** `assets/videos/demo_scenario_complet_reachy_mini.mp4`

### Post-Production

- [ ] **Montage vidéos**
  - Découpage début/fin (silence)
  - Ajout texte explicatif si nécessaire
  - Compression optimale (YouTube-ready)

- [ ] **Upload YouTube** (optionnel mais recommandé)
  - Créer playlist "BBIA-SIM Reachy Mini Démos"
  - Titres descriptifs
  - Descriptions avec liens GitHub

---

## 📝 Phase 5 : Documentation & Validation (Semaine 3)

### Documentation Hardware

- [ ] **Mettre à jour README.md**
  - Section "Robot Réel" avec captures vidéo
  - Instructions hardware validées
  - Troubleshooting hardware réel

- [ ] **Créer guide hardware**
  - `docs/guides/GUIDE_HARDWARE_REACHY_MINI.md`
  - Checklist setup
  - Problèmes courants + solutions
  - Performances mesurées (latence, FPS)

### Validation Finale

- [ ] **Tous tests hardware passent**

  ```bash
  SKIP_HARDWARE_TESTS=0 pytest tests/ -k "real" -v
  ```

- [ ] **Coverage tests hardware**
  - Vérifier que tous les skips sont désactivés
  - Tests passent sur robot réel

- [ ] **Artifacts générés**
  - `artifacts/latency_reachy_mini.csv` (mesures latence)
  - `artifacts/test_results_reachy_mini.json` (résultats tests)
  - `assets/videos/*_reachy_mini.mp4` (5 démos)

### Checklist Pré-Portfolio

- [ ] **5 vidéos prêtes**
  - Qualité acceptable
  - Démos fonctionnelles
  - Pas d'erreurs visibles

- [ ] **Métriques collectées**
  - Latence p50/p95 documentée
  - FPS caméra mesuré
  - Performances notées

- [ ] **Documentation complète**
  - Guide hardware créé
  - README mis à jour
  - Troubleshooting documenté

---

## 🎯 Critères de Succès

### Technique

✅ **Robot fonctionne avec BBIA-SIM**

- Connexion stable
- Tests hardware passent
- Latence acceptable (< 50ms p95)

✅ **Démos fonctionnelles**

- 5 vidéos montrent fonctionnalités
- Pas d'erreurs visibles
- Qualité suffisante pour portfolio

### Portfolio

✅ **Preuve hardware**

- Vidéos montrent robot réel (pas simulation)
- Performances mesurées documentées
- Guides hardware complets

---

## ⚠️ Problèmes Potentiels & Solutions

### Robot ne connecte pas

**Symptômes :**

- Timeout connexion
- SDK ne trouve pas robot

**Solutions :**

1. Vérifier réseau Wi-Fi (même réseau)
2. Scanner réseau pour IP robot
3. Vérifier firewall ports
4. Essayer USB direct si disponible

### Tests timeout

**Symptômes :**

- Tests hardware timeout
- Robot ne répond pas

**Solutions :**

1. Vérifier batterie/alimentation
2. Redémarrer robot
3. Augmenter timeout tests (`--timeout=60`)
4. Vérifier connexion réseau

### Latence élevée

**Symptômes :**

- Mouvements saccadés
- Latence > 100ms

**Solutions :**

1. Vérifier qualité réseau Wi-Fi
2. Réduire charge CPU (fermer autres apps)
3. Utiliser USB si disponible
4. Vérifier firmware robot à jour

### Caméra ne fonctionne pas

**Symptômes :**

- `robot.media.camera` retourne None
- Pas de frames

**Solutions :**

1. Vérifier permissions caméra
2. Vérifier SDK version (compatibilité)
3. Tester caméra directement (SDK seul)
4. Fallback OpenCV webcam si nécessaire

---

## 📅 Timeline Détaillé

### Semaine 1 (Décembre, Jours 1-7)

**Jours 1-2 :** Réception, installation, connexion de base  
**Jours 3-4 :** Tests SDK officiel, dry-run  
**Jours 5-7 :** Tests BBIA modules (vision, audio, mouvements)

### Semaine 2 (Décembre, Jours 8-14)

**Jours 8-9 :** Mesures performance (latence, FPS)  
**Jours 10-12 :** Enregistrement 5 démos vidéo  
**Jours 13-14 :** Post-production vidéos

### Semaine 3 (Décembre, Jours 15-21)

**Jours 15-16 :** Documentation hardware complète  
**Jours 17-18 :** Validation finale tous tests  
**Jours 19-21 :** Préparation portfolio (README, upload vidéos)

---

## 🚀 Prochaines Étapes Après Validation

1. **Mise à jour README.md** avec vidéos intégrées
2. **Création page Démos** dans docs/
3. **Upload YouTube** (optionnel)
4. **LinkedIn post** avec vidéos
5. **Candidatures** avec portfolio complet

---

**Date création :** Oct / No2025025025025025  
**Date validation cible :** Fin Oct / No2025025025025025  
**Status :** ⏳ En attente réception robot
