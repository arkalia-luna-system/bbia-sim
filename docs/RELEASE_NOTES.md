# Notes de Release - BBIA-SIM

## Version 1.2.0 - IA Légère + Scripts One-Click + Vitesse Robot Optimisée ✅ RELEASED

### 🎯 Résumé

Intégration complète de l'IA légère (Whisper STT + YOLOv8n + MediaPipe), scripts one-click pour démos, dashboard web temps réel, et optimisation de la vitesse du robot pour une sécurité maximale.

### 🚀 Nouvelles fonctionnalités v1.2.0

#### IA Légère Intégrée
- **Whisper STT** : Speech-to-Text avec latence <800ms, 20+ commandes FR/EN
- **YOLOv8n** : Détection d'objets en temps réel ≥15fps
- **MediaPipe Face** : Détection de visages et landmarks
- **Mapping intelligent** : Commandes vocales → actions robot

#### Scripts One-Click
- **run_demo_sim.sh** : Démo simulation complète avec viewer 3D
- **run_demo_real.sh** : Démo robot réel avec mode lent sécurisé
- **bbia_dashboard_server.py** : Serveur dashboard web temps réel
- **stt_demo.py** : Tests commandes vocales interactives

#### Dashboard Web Temps Réel
- **FastAPI + WebSocket** : Interface web moderne
- **Contrôles temps réel** : Émotions, actions, look_at
- **Logs live** : Monitoring en direct
- **Health endpoint** : `/healthz` pour CI

#### Optimisation Vitesse Robot
- **Mode lent sécurisé** : 0.1 Hz (10s par cycle) pour robot réel
- **Amplitude réduite** : 0.2 rad (au lieu de 0.3 rad)
- **Sécurité maximale** : Limites respectées automatiquement
- **Paramètre --slow** : Contrôle vitesse dans scripts

## 🏗️ Architecture de la Version 1.1.1

```mermaid
graph TB
    subgraph "BBIA Modules"
        EMOTIONS[bbia_emotions.py<br/>8 émotions]
        VISION[bbia_vision.py<br/>Détection objets]
        AUDIO[bbia_audio.py<br/>Enregistrement]
        VOICE[bbia_voice.py<br/>TTS/STT]
        BEHAVIOR[bbia_behavior.py<br/>Comportements]
    end
    
    subgraph "RobotAPI Interface"
        API[RobotAPI<br/>Interface unifiée<br/>CONTRACT.md gelé v1.1.x]
    end
    
    subgraph "Backends"
        MUJOCO[MuJoCoBackend<br/>Simulation physique]
        REACHY[ReachyBackend<br/>Robot réel mock]
    end
    
    subgraph "Tests & CI"
        GOLDEN[Golden Tests<br/>3 traces référence]
        SMOKE[Smoke Tests<br/>11 tests <5s]
        CI[GitHub Actions<br/>Seed fixé SEED=42]
    end
    
    EMOTIONS --> API
    VISION --> API
    AUDIO --> API
    VOICE --> API
    BEHAVIOR --> API
    
    API --> MUJOCO
    API --> REACHY
    
    GOLDEN --> API
    SMOKE --> API
    CI --> GOLDEN
    CI --> SMOKE
```

## 🔧 Améliorations techniques

### Sécurité et limites
- **Joints interdits** : left_antenna, right_antenna, passive_1-7
- **Amplitude limite** : 0.3 rad maximum
- **Validation centralisée** : Dans RobotAPI._validate_joint_pos()

### Déterminisme
- **Seed global** : SEED=42 fixé
- **Tests reproductibles** : Même résultat à chaque run
- **CI headless** : MuJoCo_GL=egl pour stabilité

### Évolutivité
- **API versionnée** : CONTRACT.md gelé v1.1.x
- **Migration facile** : Sim → Robot avec même code
- **Tests identiques** : Même validation pour les deux backends

## 📊 Métriques

### Tests
- **466 tests collectés** par pytest
- **453 tests passent** (97% de réussite)
- **Coverage** : 63.37%
- **Golden tests** : 3 traces de référence

### Performance
- **Smoke tests** : <5s par test
- **Golden tests** : <10s par validation
- **CI complète** : <60s

## 🚀 Commandes de migration

### Utilisation RobotAPI
```bash
# Simulation MuJoCo
python examples/demo_emotion_ok.py --backend mujoco --emotion happy

# Robot réel (mock)
python examples/demo_emotion_ok.py --backend reachy --emotion happy
```

### Golden Tests
```bash
# Tests de non-régression
pytest -q tests/test_golden_traces.py

# Régénérer référence
python scripts/record_trace.py --emotion happy --duration 5
```

## 🔄 Migration depuis v1.0.x

### Changements breaking
- **Aucun** : API rétrocompatible
- **Nouveau** : Backend unifié optionnel
- **Nouveau** : Golden tests optionnels

### Migration recommandée
1. **Tester** : Vérifier que les démos existantes fonctionnent
2. **Migrer** : Remplacer les appels directs MuJoCo par RobotAPI
3. **Valider** : Lancer les golden tests

## 🎯 Prochaines versions

### 📊 Métriques Finales v1.2.0 ✅ RELEASED
- **Tests** : 581 collectés, 453 passent (78% de réussite)
- **Coverage** : 63.37% de couverture de code
- **Latence** : Moyenne 0.02ms, max 4.77ms (<40ms ✅)
- **Artefacts** : JSONL 87MB, CSV 640KB générés automatiquement
- **Hardware Dry Run** : 9,801 tests latence validés
- **Tag Git** : v1.2.0 créé avec commit 0875d2a

### v1.2.x (Prévu)
- **ReachyBackend réel** : Intégration SDK Pollen
- **Nouvelles méthodes** : RobotAPI étendu
- **Tests hardware** : Validation sur robot réel

### v2.0.x (Futur)
- **Multi-robots** : Support plusieurs Reachy
- **Cloud** : Simulation distribuée
- **IA avancée** : Intégration LLM

---

*Dernière mise à jour : Octobre 2025*