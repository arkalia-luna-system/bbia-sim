# 🔴 Guide: Moteur 2 clignote en rouge + Tête de travers

## ❌ **PROBLÈME**

- Tous les bras sont allumés en rouge
- Le moteur 2 (stewart_2, Motor ID 12) **clignote** en rouge
- La tête est toujours de travers

## 🔍 **DIAGNOSTIC**

Le clignotement rouge indique une **erreur matérielle** sur le moteur 2 :
- ❌ Surcharge (overload)
- ❌ Surchauffe (overheating)
- ❌ Problème de connexion/câblage
- ❌ Moteur en butée mécanique
- ❌ Moteur défectueux

## ✅ **SOLUTIONS PAR ORDRE DE PROBABILITÉ**

### **1️⃣ Vérifier le câblage du moteur 2**

Le problème le plus fréquent est un **câble mal branché** ou **défectueux**.

**Étapes :**
1. **Éteignez le robot** (interrupteur OFF)
2. **Enlevez le capot de la tête**
3. **Vérifiez le câble du moteur 2** :
   - Le câble est-il **bien enfoncé** dans le connecteur?
   - Le câble n'est-il pas **déconnecté**?
   - Le câble n'est-il pas **endommagé** (coupure, pli, etc.)?
   - Le câble est-il dans le **bon ordre** (daisy-chain)?
4. **Rebranchez le câble** en vous assurant qu'il est bien enfoncé
5. **Rallumez le robot** (interrupteur ON)
6. **Vérifiez** si le clignotement a disparu

### **2️⃣ Vérifier la butée mécanique**

Le moteur 2 peut être **bloqué mécaniquement**.

**Étapes :**
1. **Éteignez le robot** (interrupteur OFF)
2. **Enlevez le capot de la tête**
3. **Vérifiez manuellement** :
   - Le moteur 2 peut-il **bouger librement**?
   - Y a-t-il une **résistance anormale**?
   - Y a-t-il un **câble qui bloque** le mouvement?
   - Le moteur est-il **en butée** (position limite)?
4. **Déplacez légèrement le moteur** manuellement pour le sortir de la butée
5. **Rallumez le robot** (interrupteur ON)

### **3️⃣ Vérifier la position du moteur 2**

Le moteur 2 peut être dans une **position hors limites**.

**Diagnostic via SSH :**
```bash
ssh pollen@192.168.129.64
python3 << 'EOF'
from reachy_mini import ReachyMini

robot = ReachyMini(media_backend="no_media", use_sim=False, localhost_only=True)
robot.__enter__()

head_positions, _ = robot.get_current_joint_positions()
if len(head_positions) >= 2:
    stewart_2_pos = head_positions[1]
    print(f"Position stewart_2: {stewart_2_pos:.4f} rad ({stewart_2_pos*180/3.14159:.2f}°)")
    
    # Limites: [-1.396, 1.222] rad
    limits = (-1.396263401595614, 1.2217304763958803)
    if limits[0] <= stewart_2_pos <= limits[1]:
        print("✅ Position dans les limites")
    else:
        print(f"⚠️  Position HORS LIMITES! Limites: [{limits[0]:.4f}, {limits[1]:.4f}] rad")
        print("   → Le moteur doit être déplacé manuellement vers le centre")

robot.__exit__(None, None, None)
EOF
```

**Si la position est hors limites :**
1. **Éteignez le robot** (interrupteur OFF)
2. **Déplacez manuellement** le moteur 2 vers une position centrale
3. **Rallumez le robot** (interrupteur ON)

### **4️⃣ Réinitialiser les erreurs du moteur**

Parfois, les erreurs persistent même après correction.

**Via SSH :**
```bash
ssh pollen@192.168.129.64
sudo systemctl restart reachy-mini-daemon
```

**OU redémarrer complètement le robot :**
1. **Éteignez le robot** (interrupteur OFF)
2. **Attendez 10 secondes**
3. **Rallumez le robot** (interrupteur ON)

### **5️⃣ Vérifier si le moteur est défectueux**

Si toutes les solutions ci-dessus échouent, le moteur 2 peut être **défectueux**.

**Test :**
1. **Échangez le moteur 2 avec un autre moteur** (par exemple moteur 3)
2. Si le problème **se déplace** avec le moteur → le moteur est défectueux
3. Si le problème **reste sur la position 2** → c'est un problème de câblage/position

**Si le moteur est défectueux :**
- Contactez le support Pollen Robotics
- Formulaire: https://forms.gle/JdhMzadeCnbynw7Q6

## 🔧 **CORRECTION DE LA TÊTE DE TRAVERS**

Une fois le moteur 2 corrigé, la tête peut encore être de travers.

**Correction via SSH :**
```bash
ssh pollen@192.168.129.64
python3 << 'EOF'
from reachy_mini import ReachyMini
from reachy_mini.utils import create_head_pose

robot = ReachyMini(media_backend="no_media", use_sim=False, localhost_only=True)
robot.__enter__()

# Position neutre (tête droite)
neutral = create_head_pose(x=0, y=0, z=0, roll=0, pitch=0, yaw=0, degrees=True, mm=True)
robot.goto_target(head=neutral, duration=2.0)

print("✅ Tête repositionnée en position neutre")

robot.__exit__(None, None, None)
EOF
```

## 📋 **CHECKLIST RAPIDE**

- [ ] Câble du moteur 2 bien branché
- [ ] Câble non endommagé
- [ ] Moteur 2 peut bouger librement
- [ ] Moteur 2 pas en butée mécanique
- [ ] Position du moteur 2 dans les limites
- [ ] Daemon redémarré
- [ ] Robot redémarré (OFF/ON)
- [ ] Tête repositionnée en position neutre

## 💡 **RESSOURCES**

- Guide d'assemblage: https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide
- Support Pollen: https://forms.gle/JdhMzadeCnbynw7Q6
- Discord Pollen Robotics: https://discord.gg/pollen-robotics

