# 🔄 Switch Sim → Robot

## Backend Unifié RobotAPI

Le projet BBIA-Reachy-SIM utilise maintenant une **interface unifiée RobotAPI** qui permet de basculer facilement entre la simulation MuJoCo et le robot Reachy réel.

## 🎯 Architecture

```
BBIA Modules → RobotAPI → Backend
                    ├── MuJoCoBackend (Simulation)
                    └── ReachyBackend (Robot réel)
```

## 🚀 Utilisation

### **Simulation MuJoCo (Développement)**
```bash
# Démo avec backend MuJoCo
python examples/demo_emotion_ok.py --backend mujoco --emotion happy --duration 5

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

## 📊 Fonctionnalités

### **API Unifiée**
- `set_joint_pos(joint_name, position)` : Contrôle des articulations
- `set_emotion(emotion, intensity)` : Gestion des émotions
- `look_at(target_x, target_y, target_z)` : Regarder vers une cible
- `run_behavior(behavior_name, duration)` : Exécuter un comportement
- `get_telemetry()` : Données de télémétrie

### **Backends Disponibles**
- **MuJoCoBackend** : Simulation complète avec viewer
- **ReachyBackend** : Robot réel (implémentation mock)

### **Sécurité**
- Limites automatiques (amplitude ≤ 0.3 rad)
- Joints interdits bloqués
- Clamp automatique des positions
- Gestion d'erreurs robuste

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
