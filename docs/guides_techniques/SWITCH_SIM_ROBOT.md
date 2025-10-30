# 🔄 Switch Sim → Robot

> Compatibilité Python et CI
>
> - Version requise: Python 3.11+
> - CI: `.github/workflows/ci.yml`
> - Setup rapide:
>   ```bash
>   pyenv install 3.11.9 && pyenv local 3.11.9
>   python -m pip install --upgrade pip
>   pip install -e .
>   ```

## Backend Unifié RobotAPI

Le projet BBIA-Reachy-SIM utilise maintenant une **interface unifiée RobotAPI** qui permet de basculer facilement entre la simulation MuJoCo et le robot Reachy réel.

## 🎯 Architecture

```mermaid
graph TB
    subgraph "BBIA Modules"
        EMOTIONS[bbia_emotions.py]
        VISION[bbia_vision.py]
        AUDIO[bbia_audio.py]
        VOICE[bbia_voice.py]
        BEHAVIOR[bbia_behavior.py]
    end

    subgraph "RobotAPI Interface"
        API[RobotAPI<br/>Interface unifiée]
    end

    subgraph "Backends"
        MUJOCO[MuJoCoBackend<br/>Simulation]
        REACHY[ReachyBackend<br/>Robot réel]
    end

    EMOTIONS --> API
    VISION --> API
    AUDIO --> API
    VOICE --> API
    BEHAVIOR --> API

    API --> MUJOCO
    API --> REACHY

    MUJOCO --> SIM[MuJoCo Simulator<br/>3D Physics]
    REACHY --> ROBOT[Reachy Mini<br/>Hardware]
```

## 🔄 Workflow de Switch

```mermaid
sequenceDiagram
    participant DEV as Développeur
    participant FACTORY as RobotFactory
    participant API as RobotAPI
    participant SIM as MuJoCoBackend
    participant ROBOT as ReachyBackend

    Note over DEV,ROBOT: Phase 1: Développement
    DEV->>FACTORY: create_backend("mujoco")
    FACTORY->>SIM: Initialiser MuJoCo
    SIM->>API: Interface unifiée
    API->>DEV: Robot prêt (simulation)

    Note over DEV,ROBOT: Phase 2: Tests
    DEV->>API: set_emotion("happy", 0.8)
    API->>SIM: Appliquer émotion
    SIM->>API: Confirmation

    Note over DEV,ROBOT: Phase 3: Production
    DEV->>FACTORY: create_backend("reachy")
    FACTORY->>ROBOT: Initialiser Reachy
    ROBOT->>API: Interface unifiée
    API->>DEV: Robot prêt (réel)

    DEV->>API: set_emotion("happy", 0.8)
    API->>ROBOT: Appliquer émotion
    ROBOT->>API: Confirmation
```

## 🚀 Utilisation

### **Simulation MuJoCo (Développement)**
```bash
# Démo avec backend MuJoCo - MODE GRAPHIQUE (voir la 3D)
mjpython examples/demo_emotion_ok.py --backend mujoco --emotion happy --duration 5

# Démo avec backend MuJoCo - MODE HEADLESS (tests rapides)
python examples/demo_emotion_ok.py --headless --backend mujoco --emotion happy --duration 5

# Tests avec MuJoCo
python -m pytest tests/test_robot_api_smoke.py -v
```

### **Robot Reachy Réel (Production)**
```bash
# Démo avec backend Reachy réel
python examples/demo_emotion_ok.py --backend reachy --emotion happy --duration 5

# Configuration robot
python examples/demo_emotion_ok.py --backend reachy --robot-ip 192.168.1.100 --robot-port 8080
```

## 🔧 Code Exemple

### **Utilisation Simple**
```python
from bbia_sim.robot_factory import RobotFactory

# Créer le backend
robot = RobotFactory.create_backend("mujoco")  # ou "reachy"

# Connecter
robot.connect()

# Utiliser l'API unifiée
robot.set_emotion("happy", intensity=0.8)
robot.set_joint_pos("yaw_body", 0.3)
robot.look_at(target_x=0.5, target_y=0.0)
robot.run_behavior("wake_up", duration=5.0)

# Déconnecter
robot.disconnect()
```

### **Switch Automatique**
```python
import os

# Variable d'environnement pour choisir le backend
backend_type = os.environ.get("BBIA_BACKEND", "mujoco")
robot = RobotFactory.create_backend(backend_type)
```

## 📊 Comparaison des Backends

```mermaid
graph LR
    subgraph "MuJoCoBackend (Simulation)"
        SIM_FEATURES[✅ Physique 3D<br/>✅ Viewer graphique<br/>✅ Mode headless<br/>✅ Tests automatisés<br/>✅ Débogage facile<br/>❌ Pas de robot physique]
    end

    subgraph "ReachyBackend (Robot réel)"
        ROBOT_FEATURES[✅ Robot physique<br/>✅ Vraie interaction<br/>✅ Capteurs réels<br/>✅ Production<br/>❌ Plus lent<br/>❌ Plus risqué]
    end

    SIM_FEATURES -.->|Migration| ROBOT_FEATURES
```

## 🔄 Migration Sim → Robot

```mermaid
flowchart TD
    START[Début du développement] --> SIM[Utiliser MuJoCoBackend]
    SIM --> TEST[Tester les fonctionnalités]
    TEST --> WORK{Ça fonctionne ?}
    WORK -->|Non| DEBUG[Déboguer en simulation]
    DEBUG --> TEST
    WORK -->|Oui| SWITCH[Basculer vers ReachyBackend]
    SWITCH --> PROD[Tests sur robot réel]
    PROD --> DEPLOY[Déploiement production]
```

## 🎬 Record & Replay

### **Enregistrement**
```bash
# Enregistrer une animation
python examples/demo_emotion_ok.py --record artifacts/my_animation.jsonl --emotion happy --duration 10
```

### **Rejeu**
```bash
# Rejouer une animation
python scripts/replay_viewer.py artifacts/my_animation.jsonl --speed 1.5
```

## 📈 Télémétrie

### **Collecte Automatique**
```python
from bbia_sim.telemetry import TelemetryCollector

telemetry = TelemetryCollector()
telemetry.start_collection()

# Pendant l'animation
telemetry.record_step({"yaw_body": 0.3, "stewart_1": 0.1})

# Export
stats = telemetry.stop_collection()
telemetry.export_csv("demo_stats.csv", stats)
```

### **Métriques**
- Steps par seconde
- Temps moyen de step
- Drift maximum
- Positions des joints

## 🔄 Migration Sim → Robot

### **Étape 1 : Développement**
```bash
# Utiliser MuJoCo pour le développement
export BBIA_BACKEND=mujoco
python examples/demo_emotion_ok.py --emotion happy
```

### **Étape 2 : Tests**
```bash
# Tester avec les deux backends
python -m pytest tests/test_robot_api_smoke.py -v
```

### **Étape 3 : Production**
```bash
# Basculer vers le robot réel
export BBIA_BACKEND=reachy
export BBIA_ROBOT_IP=192.168.1.100
python examples/demo_emotion_ok.py --emotion happy
```

## ✅ Avantages

1. **Code identique** : Même code pour Sim et Robot
2. **Tests unifiés** : Tests automatiques sur les deux backends
3. **Développement rapide** : MuJoCo pour l'itération
4. **Production fiable** : Reachy pour le déploiement
5. **Record/Replay** : Débogage et portfolio
6. **Télémétrie** : Monitoring et optimisation

## 🎯 Prochaines Étapes

1. **Implémentation ReachyBackend** : Connexion réelle au robot
2. **API avancée** : Plus de méthodes de contrôle
3. **Configuration** : Fichiers de config par environnement
4. **Monitoring** : Dashboard temps réel
5. **CI/CD** : Tests automatiques sur robot réel

## 📊 Métriques v1.1.1

- **Tests** : 441 tests passent (79% réussite)
- **Coverage** : 68.86%
- **Performance** : <5s par test smoke
- **Golden Tests** : 3 traces référence + validation
- **Latence** : 0.0ms moyenne (cible <40ms) ✅
- **CI Solide** : Seed fixé, artefacts, headless
