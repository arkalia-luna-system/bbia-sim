# 🚀 BBIA-SIM v1.2.0 - GUIDE QUICKSTART DÉVELOPPEURS

**Version** : 1.2.0 "Reachy-Ready + IA Légère"  
**Objectif** : Démo professionnelle avec robot réel  
**Cible** : Développeurs et chercheurs  

## 🎯 **VUE D'ENSEMBLE**

BBIA-SIM est une simulation robotique complète du Reachy Mini Wireless avec intégration IA. Le projet est **prêt pour le robot réel** avec une architecture unifiée Sim ↔ Robot.

### **✅ Ce qui fonctionne déjà**
- **Simulation 3D** : Robot visible et animé dans MuJoCo
- **Backend unifié** : Switch facile entre simulation et robot réel
- **4 Vertical Slices** : Émotion, Voix, Vision, Comportement
- **Tests robustes** : 431 tests passent, Golden Tests anti-régression
- **CI/CD** : Pipeline GitHub Actions avec artefacts

### **🎯 Objectif v1.2.0**
- **Démo professionnelle** avec robot réel
- **IA légère** : Whisper (STT) + YOLOv8n (vision) + MediaPipe (faces)
- **Interface simple** : Dashboard web minimal
- **Sécurité maximale** : Kill-switch + validation centralisée

---

## 🚀 **INSTALLATION RAPIDE**

### **1. Prérequis**
```bash
# Python 3.10+
python --version

# Git
git --version

# MuJoCo (macOS)
brew install mujoco
```

### **2. Installation**
```bash
# Cloner le projet
git clone https://github.com/arkalia-luna-system/bbia-sim.git
cd bbia-sim

# Environnement virtuel
python -m venv venv
source venv/bin/activate  # macOS/Linux
# ou
venv\Scripts\activate  # Windows

# Dépendances
pip install -r requirements.txt
```

### **3. Vérification**
```bash
# Tests rapides
pytest tests/test_robot_api_smoke.py -v

# Démo 3D (MuJoCo)
mjpython examples/demo_emotion_fixed.py

# Tests complets
pytest tests/ -v --tb=short
```

---

## 🎮 **UTILISATION RAPIDE**

### **🎭 Démo Émotions (3D)**
```bash
# Démo stable avec viewer MuJoCo
mjpython examples/demo_emotion_fixed.py

# Démo avec RobotAPI (headless)
python examples/demo_emotion_ok.py --backend mujoco --headless
```

### **🗣️ Démo Voix**
```bash
# Reconnaissance vocale + synthèse
python examples/demo_voice_ok.py --backend mujoco
```

### **👁️ Démo Vision**
```bash
# Détection objets + suivi
python examples/demo_vision_ok.py --backend mujoco
```

### **🤖 Démo Comportements**
```bash
# Comportements complexes
python examples/demo_behavior_ok.py --backend mujoco
```

---

## 🔧 **API PRINCIPALE**

### **🤖 RobotAPI (Interface Unifiée)**
```python
from bbia_sim.robot_factory import RobotFactory

# Créer backend (simulation ou robot réel)
robot = RobotFactory.create_backend("mujoco")  # ou "reachy"

# Connexion
robot.connect()

# Contrôle joints
robot.set_joint_pos("yaw_body", 0.2)  # rad
position = robot.get_joint_pos("yaw_body")

# Émotions
robot.set_emotion("happy", intensity=0.8)

# Comportements
robot.run_behavior("greeting")

# Déconnexion
robot.disconnect()
```

### **🎭 Système d'Émotions**
```python
from bbia_sim.bbia_emotions import BBIAEmotions

emotions = BBIAEmotions()

# Émotions disponibles
print(emotions.emotions.keys())
# ['neutral', 'happy', 'sad', 'angry', 'curious', 'excited', 'surprised', 'fearful']

# Changer émotion
emotions.set_emotion("happy", intensity=0.7)
```

### **👁️ Système de Vision**
```python
from bbia_sim.bbia_vision import BBIAVision

vision = BBIAVision()

# Scanner environnement
result = vision.scan_environment()
print(f"Objets détectés: {len(result['objects'])}")

# Suivre objet
vision.track_object("personne")
```

---

## 🛡️ **SÉCURITÉ & LIMITES**

### **⚠️ Joints Interdits**
```python
# Ces joints ne doivent JAMAIS être contrôlés :
FORBIDDEN_JOINTS = {
    "left_antenna", "right_antenna",
    "passive_1", "passive_2", "passive_3", 
    "passive_4", "passive_5", "passive_6", "passive_7"
}
```

### **🔒 Limites de Sécurité**
```python
# Amplitude maximale : 0.3 rad
SAFE_AMPLITUDE_LIMIT = 0.3

# Joints sûrs recommandés
SAFE_JOINTS = {"yaw_body"}
```

### **🛑 Kill-Switch**
```bash
# Script d'arrêt d'urgence
python scripts/emergency_stop.py

# Bouton STOP matériel (rappel constant)
# Toujours prévoir un bouton STOP physique sur le robot réel
```

---

## 🧪 **TESTS & VALIDATION**

### **🔍 Tests Rapides**
```bash
# Tests smoke (<5s)
pytest tests/test_robot_api_smoke.py -v

# Tests limites sécurité
pytest tests/test_robot_api_limits.py -v

# Tests vertical slices
pytest tests/test_vertical_slices.py -v
```

### **🧪 Golden Tests**
```bash
# Tests de régression
pytest tests/test_golden_traces.py -v

# Régénérer traces de référence
python scripts/record_trace.py --emotion happy --duration 3 --output artifacts/golden/happy_mujoco.jsonl
```

### **📊 Validation Hardware**
```bash
# Test hardware Reachy réel
python scripts/hardware_dry_run.py --backend reachy

# Mesure latence
python scripts/measure_latency.py --backend reachy
```

---

## 🎯 **ROADMAP v1.2.0**

### **🎯 Semaine 1 - Reachy-Ready (✅ ACCOMPLI)**
- [x] Installation Reachy SDK
- [x] Mapping joints physique
- [x] Extension hardware_dry_run.py
- [x] Vidéo démo MuJoCo

### **⏳ Semaine 2 - IA Légère (EN COURS)**
- [ ] Intégration Whisper STT
- [ ] Intégration YOLOv8n + MediaPipe
- [ ] Dashboard web minimal
- [ ] Tests d'intégration IA

### **⏳ Semaine 3 - Polish Démo (EN ATTENTE)**
- [ ] Scripts one-click
- [ ] Documentation utilisateur
- [ ] Portfolio one-pager
- [ ] Release v1.2.0

---

## 📚 **RESSOURCES**

### **📖 Documentation**
- `README.md` - Guide principal
- `docs/CONTRACT.md` - Contrat RobotAPI
- `docs/SWITCH_SIM_ROBOT.md` - Guide switch backend
- `docs/ROADMAP_STRATEGIQUE_v1.2.0.md` - Roadmap détaillée

### **🔧 Scripts Utiles**
- `scripts/hardware_dry_run.py` - Test hardware
- `scripts/record_trace.py` - Enregistrement traces
- `scripts/validate_trace.py` - Validation traces
- `scripts/emergency_stop.py` - Arrêt d'urgence

### **📁 Fichiers Clés**
- `src/bbia_sim/robot_api.py` - Interface unifiée
- `src/bbia_sim/global_config.py` - Configuration centralisée
- `src/bbia_sim/bbia_emotions.py` - Système émotions
- `examples/demo_emotion_fixed.py` - Démo 3D stable

---

## 🆘 **DÉPANNAGE**

### **❌ Problèmes Courants**

#### **MuJoCo ne s'ouvre pas**
```bash
# Utiliser mjpython au lieu de python
mjpython examples/demo_emotion_fixed.py

# Vérifier installation MuJoCo
python -c "import mujoco; print('MuJoCo OK')"
```

#### **Tests échouent**
```bash
# Vérifier environnement
source venv/bin/activate
pip install -r requirements.txt

# Tests individuels
pytest tests/test_robot_api_smoke.py::test_mujoco_backend_creation -v
```

#### **RobotAPI ne fonctionne pas**
```bash
# Vérifier backend
python -c "from bbia_sim.robot_factory import RobotFactory; print(RobotFactory.list_backends())"

# Test simple
python -c "from bbia_sim.robot_factory import RobotFactory; robot = RobotFactory.create_backend('mujoco'); print(robot.connect())"
```

### **📞 Support**
- **Issues GitHub** : [Créer une issue](https://github.com/arkalia-luna-system/bbia-sim/issues)
- **Documentation** : `docs/` directory
- **Tests** : `tests/` directory pour exemples

---

## 🏆 **CAS D'USAGE IDÉAL v1.2.0**

### **🎯 Scénario Parfait**
```python
# Séquence de démo professionnelle :
1. "Bonjour BBIA" → Robot se tourne vers l'interlocuteur
2. "Comment ça va ?" → Robot répond avec émotion happy
3. "Regarde cette personne" → Robot suit + curious
4. "Au revoir" → Robot salue + "Au revoir !"
```

### **📊 Métriques de Succès**
- **Latence** : <40ms (robot réel)
- **Précision** : >90% (reconnaissance commandes)
- **Stabilité** : 0 crash en 10 min de démo
- **Sécurité** : 0 mouvement dangereux

---

**Ce guide vous permet de démarrer rapidement avec BBIA-SIM. Focus sur la démo professionnelle avec le robot réel !** 🚀
