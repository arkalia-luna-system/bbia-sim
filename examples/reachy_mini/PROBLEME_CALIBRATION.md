# ⚠️ PROBLÈME DE CALIBRATION - Tête de travers

## 🔴 **SITUATION**

- ✅ Tous les moteurs sont détectés (reflash réussi)
- ✅ Le moteur 2 bouge (test réussi)
- ✅ Les câbles sont corrects (vérifiés 2 fois)
- ❌ La tête est toujours de travers
- ❌ Le moteur 2 clignote en rouge

## 💡 **DIAGNOSTIC**

Ce n'est **PAS** un problème de câblage. C'est probablement un problème de **CALIBRATION/OFFSET**.

Les moteurs ont des **offsets différents** qui font que la position "neutre" (tous à 0) ne correspond pas à une tête droite.

### Preuve :
- Le script montre que même en position "neutre", les stewart joints ne sont pas à 0 :
  - stewart_1: 0.00°
  - stewart_2: -22.85° ← **PROBLÈME ICI**
  - stewart_3: -11.34°
  - stewart_4: 32.78°
  - stewart_5: -19.16°
  - stewart_6: 43.51°

## ✅ **SOLUTIONS**

### **1️⃣ Script de correction forcée**

```bash
# Sur le robot
python3 /tmp/force_head_straight.py
```

Ce script va :
- Désactiver/réactiver les moteurs
- Faire des mouvements pour débloquer
- Essayer de repositionner la tête

### **2️⃣ Recalibration des offsets**

Si le problème persiste, il faut **recalibrer les offsets** des moteurs.

**Option A : Via le script officiel (si disponible)**
```bash
reachy-mini-calibrate-offsets
```

**Option B : Manuellement via SSH**
```bash
ssh pollen@192.168.129.64
sudo systemctl stop reachy-mini-daemon
# Utiliser les outils de calibration du SDK
```

### **3️⃣ Ajustement manuel des offsets**

Les offsets sont dans la configuration hardware. Il faut les ajuster pour que la tête soit droite quand tous les stewart joints sont à leur position "neutre".

**Fichier de configuration :**
- `/home/pollen/.local/lib/python3.*/site-packages/reachy_mini/assets/config/hardware_config.yaml`

**Ou via le SDK :**
```python
from reachy_mini.utils.hardware_config.parser import parse_yaml_config
config = parse_yaml_config("hardware_config.yaml")
# Ajuster les offsets pour stewart_2
```

### **4️⃣ Contacter Pollen Robotics**

Si rien ne fonctionne, c'est un problème de **calibration d'usine** ou de **moteur défectueux**.

**Formulaire de support :**
https://forms.gle/JdhMzadeCnbynw7Q6

**Informations à fournir :**
- Tous les moteurs sont détectés ✅
- Le moteur 2 bouge ✅
- Les câbles sont corrects ✅
- La tête est de travers (stewart_2 à -22.85° en position "neutre")
- Le moteur 2 clignote en rouge

## 🔧 **TEST RAPIDE**

Pour vérifier si c'est vraiment un problème d'offset :

```bash
ssh pollen@192.168.129.64
python3 << 'EOF'
from reachy_mini import ReachyMini
from reachy_mini.utils import create_head_pose

robot = ReachyMini(media_backend="no_media", use_sim=False, localhost_only=True)
robot.__enter__()

# Position actuelle
head_pos, _ = robot.get_current_joint_positions()
print("Position actuelle stewart_2:", head_pos[1]*180/3.14159, "°")

# Essayer de forcer stewart_2 à 0 en ajustant les autres
# (nécessite calcul IK inverse - complexe)

robot.__exit__(None, None, None)
EOF
```

## 📋 **CHECKLIST**

- [ ] Script force_head_straight.py exécuté
- [ ] Tête toujours de travers après correction
- [ ] Offsets vérifiés dans la config
- [ ] Support Pollen contacté avec toutes les infos

## 💬 **NOTE IMPORTANTE**

Si tu as **jamais échangé les moteurs** et que le problème persiste après :
- ✅ Reflash réussi
- ✅ Câbles vérifiés 2 fois
- ✅ Moteurs qui bougent

Alors c'est **probablement un problème de calibration d'usine** ou un **moteur défectueux** qui nécessite un remplacement.

**Ne te blâme pas** - ce n'est pas de ta faute ! C'est un problème matériel/calibration.

