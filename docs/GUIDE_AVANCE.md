# 🔬 Guide Avancé - BBIA-SIM

## 📋 **Pour Développeurs Avancés**

**Version :** 1.3.0  
**Pour :** Développeurs expérimentés qui veulent exploiter toutes les capacités de BBIA

---

## 🏗️ **Architecture Détaillée**

### **Backend Unifié**

```python
# Backend unifié : Simulation ↔ Robot réel
from bbia_sim.robot_api import RobotFactory

# Simulation (Développement & Tests)
robot_sim = RobotFactory.create_backend('mujoco')
robot_sim.connect()

# Robot réel (Production)
robot_real = RobotFactory.create_backend('reachy_mini')
robot_real.connect()

# MÊME CODE fonctionne sur les deux ! ✅
robot_sim.set_emotion('happy', 0.8)
robot_real.set_emotion('happy', 0.8)

# Basculer facilement
backend = 'mujoco' if os.getenv('DEV') else 'reachy_mini'
robot = RobotFactory.create_backend(backend)
```

**Avantages :**
- ✅ Développement sans hardware
- ✅ Tests automatisés complets
- ✅ Debugging facilité
- ✅ Migration transparente vers robot réel

### **Modules BBIA Avancés**

#### **1. Module Émotions**
```python
from bbia_sim.bbia_emotions import BBIAEmotions

emotions = BBIAEmotions()

# Toutes les émotions disponibles
emotions.set_emotion('excited', intensity=0.9)
emotions.set_emotion('curious', intensity=0.6)
emotions.set_emotion('calm', intensity=0.4)
```

#### **2. Module Comportements**
```python
from bbia_sim.bbia_behavior import BBIABehaviorManager

manager = BBIABehaviorManager()

# Créer comportement personnalisé
class MyBehavior(BBIABehavior):
    def execute(self, context):
        # Votre logique ici
        pass

manager.register_behavior(MyBehavior())
```

#### **3. Module Vision**
```python
from bbia_sim.bbia_vision import BBIAVision

vision = BBIAVision()
objects = vision.scan_environment()
faces = vision.detect_faces()
```

### **Tests Avancés**

```bash
# Lancer tous les tests
pytest tests/ -v --tb=short

# Tests avec coverage
pytest tests/ --cov=src --cov-report=html

# Voir rapport coverage
open htmlcov/index.html
```

### **Optimisations Performance**

```python
# Mode headless (rapide)
MUJOCO_GL=disable python your_script.py

# Activer cache simulation
bbia_sim.sim.enable_caching()
```

---

## 🎯 **Migration Simulation → Robot Réel**

Voir : [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md)

---

## 🔧 **API Avancée**

### **Conformité SDK Officiel**

```python
# Toutes les méthodes SDK officiel disponibles
robot.get_current_head_pose()
robot.get_current_joint_positions()
robot.look_at_world(x, y, z)
robot.goto_target(head=pose)
robot.start_recording()
robot.stop_recording()
```

Voir : [RAPPORT_CONFORMITE_SDK_2024.md](./RAPPORT_CONFORMITE_SDK_2024.md)

---

**Pour plus d'infos :** Toute la documentation dans `docs/` 📚

