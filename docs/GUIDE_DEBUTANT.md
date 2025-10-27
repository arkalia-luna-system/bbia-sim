# 🎯 Guide Débutant - BBIA-SIM

## 📋 **Mon Premier Robot BBIA en 5 Minutes**

### **1️⃣ Installation**

```bash
# Installer BBIA-SIM
pip install bbia-sim

# C'est tout ! ✅
```

### **2️⃣ Premier Programme**

```python
from bbia_sim.robot_api import RobotFactory

# Créer le robot
robot = RobotFactory.create_backend('reachy_mini')
robot.connect()

# Faire quelque chose !
robot.set_emotion('happy', 0.8)
robot.run_behavior('wake_up', duration=3.0)

# Déconnecter
robot.disconnect()
```

### **3️⃣ Tester en Simulation 3D**

```bash
# Voir le robot bouger dans MuJoCo
mjpython examples/demo_mujoco_continue.py
```

**🎉 VOUS AVEZ CONTRÔLÉ VOTRE PREMIER ROBOT !**

---

## 🤔 **Questions Fréquentes**

### **Q : J'ai pas de robot Reachy Mini, ça marche ?**
**R : OUI !** Mode simulation inclus. Vous développez maintenant, testez sur robot plus tard.

### **Q : C'est compatible Mac/Linux/Windows ?**
**R :** Mac/Linux : OUI ✅  
Windows : À tester ⚠️

### **Q : Puis-je créer mes propres émotions ?**
**R : OUI !** Le système est extensible. Voir `examples/demo_emotion_ok.py`

---

## 📚 **Prochaines Étapes**

1. **Lire** : [README](../README.md)
2. **Comprendre** : [Architecture](../docs/ARCHITECTURE.md)
3. **Explorer** : Dossier `examples/`
4. **Créer** : Vos propres comportements !

---

**Besoin d'aide ?** Ouvrez une issue sur GitHub ou rejoignez le Discord Reachy ! 🤖

