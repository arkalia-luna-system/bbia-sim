# 🔧 Guide Complet de Dépannage Reachy Mini

**Dernière mise à jour** : Décembre 2025  
**Version SDK** : 1.2.4+

---

## 📋 Table des Matières

1. [Diagnostic Rapide](#-diagnostic-rapide)
2. [Problèmes de Moteurs](#-problèmes-de-moteurs)
3. [Problèmes de Calibration](#-problèmes-de-calibration)
4. [Reflash des Moteurs](#-reflash-des-moteurs)
5. [Support Pollen Robotics](#-support-pollen-robotics)

---

## ⚡ Diagnostic Rapide

### Tableau de Correspondance Motor ID ↔ Moteur Physique

| Motor ID | Nom physique | Emplacement |
|----------|--------------|-------------|
| **10** | yaw_body | Base (rotation du corps) |
| **11** | stewart_1 | Tête (moteur 1) |
| **12** | stewart_2 | Tête (moteur 2) |
| **13** | stewart_3 | Tête (moteur 3) |
| **14** | stewart_4 | Tête (moteur 4) |
| **15** | stewart_5 | Tête (moteur 5) |
| **16** | stewart_6 | Tête (moteur 6) |
| **17** | left_antenna | Antenne gauche |
| **18** | right_antenna | Antenne droite |

### Diagnostic Automatique

```bash
# Depuis votre Mac (via SSH automatique - RECOMMANDÉ)
python examples/reachy_mini/diagnostic_motor_errors_ssh.py

# OU depuis votre Mac (connexion directe)
python examples/reachy_mini/diagnostic_motor_errors.py
```

---

## 🔴 Problèmes de Moteurs

### Symptôme : Moteur Clignote en Rouge

**C'EST ANORMAL !** Un moteur qui clignote en rouge indique une **erreur matérielle** :
- ❌ Surcharge (overload)
- ❌ Surchauffe (overheating)
- ❌ Problème de connexion/câblage
- ❌ Moteur en butée mécanique
- ❌ Moteur défectueux

### Solutions par Ordre de Probabilité

#### 1️⃣ Vérifier le Câblage

**Le problème le plus fréquent est un câble mal branché ou défectueux.**

**Étapes :**
1. **Éteignez le robot** (interrupteur OFF)
2. **Enlevez le capot de la tête**
3. **Vérifiez le câble du moteur problématique** :
   - Le câble est-il **bien enfoncé** dans le connecteur?
   - Le câble n'est-il pas **déconnecté**?
   - Le câble n'est-il pas **endommagé** (coupure, pli, etc.)?
   - Le câble est-il dans le **bon ordre** (daisy-chain)?
4. **Rebranchez le câble** en vous assurant qu'il est bien enfoncé
5. **Rallumez le robot** (interrupteur ON)
6. **Vérifiez** si le clignotement a disparu

#### 2️⃣ Vérifier la Butée Mécanique

**Le moteur peut être bloqué mécaniquement.**

**Étapes :**
1. **Éteignez le robot** (interrupteur OFF)
2. **Enlevez le capot de la tête**
3. **Vérifiez manuellement** :
   - Le moteur peut-il **bouger librement**?
   - Y a-t-il une **résistance anormale**?
   - Y a-t-il un **câble qui bloque** le mouvement?
   - Le moteur est-il **en butée** (position limite)?
4. **Déplacez légèrement le moteur** manuellement pour le sortir de la butée
5. **Rallumez le robot** (interrupteur ON)

#### 3️⃣ Vérifier la Position du Moteur

**Le moteur peut être dans une position hors limites.**

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
2. **Déplacez manuellement** le moteur vers une position centrale
3. **Rallumez le robot** (interrupteur ON)

#### 4️⃣ Réinitialiser les Erreurs du Moteur

**Parfois, les erreurs persistent même après correction.**

**Via SSH :**
```bash
ssh pollen@192.168.129.64
sudo systemctl restart reachy-mini-daemon
```

**OU redémarrer complètement le robot :**
1. **Éteignez le robot** (interrupteur OFF)
2. **Attendez 10 secondes**
3. **Rallumez le robot** (interrupteur ON)

#### 5️⃣ Moteur avec Mauvais Baudrate (Problème Décembre 2025)

**Symptôme** : Moteur clignote rouge, "Missing motor stewart_X" dans l'app  
**Cause** : Moteur avec paramètres d'usine (ID=1, baudrate 57,600) au lieu de configuration correcte

**Solution Rapide :**

```bash
# Étape 1 : Se connecter au robot
ssh pollen@192.168.129.64

# Étape 2 : Arrêter le daemon
sudo systemctl stop reachy-mini-daemon

# Étape 3 : Activer l'environnement virtuel
source /venvs/mini-daemon/bin/activate

# Étape 4 : Scanner le bus Dynamixel
python3 << 'EOF'
from reachy_mini_motor_controller import MotorsBus

print("🔍 Scan à 1,000,000 baud...")
bus = MotorsBus("/dev/ttyAMA3", baudrate=1_000_000)
motors_1M = bus.scan()
print(f"Moteurs trouvés: {motors_1M}")

print("\n🔍 Scan à 57,600 baud...")
bus = MotorsBus("/dev/ttyAMA3", baudrate=57_600)
motors_57k = bus.scan()
print(f"Moteurs trouvés: {motors_57k}")

if motors_57k:
    print(f"\n⚠️  PROBLÈME: Moteur(s) {motors_57k} encore à 57,600 baud!")
    print("   Ces moteurs doivent être reconfigurés à 1,000,000 baud")
else:
    print("\n✅ Aucun moteur à 57,600 baud (normal)")
EOF
```

**Si des moteurs sont détectés à 57.6k baud, utiliser le script de reflash (voir section Reflash).**

#### 6️⃣ Vérifier si le Moteur est Défectueux

**Si toutes les solutions ci-dessus échouent, le moteur peut être défectueux.**

**Test :**
1. **Échangez le moteur problématique avec un autre moteur** (par exemple moteur 3)
2. Si le problème **se déplace** avec le moteur → le moteur est défectueux
3. Si le problème **reste sur la position** → c'est un problème de câblage/position

**Si le moteur est défectueux :**
- Contactez le support Pollen Robotics (voir section Support)

---

## ⚠️ Problèmes de Calibration

### Symptôme : Tête de Travers

**Situation :**
- ✅ Tous les moteurs sont détectés (reflash réussi)
- ✅ Les moteurs bougent (test réussi)
- ✅ Les câbles sont corrects (vérifiés)
- ❌ La tête est toujours de travers
- ❌ Un moteur clignote en rouge

**Diagnostic :**

Ce n'est **PAS** un problème de câblage. C'est probablement un problème de **CALIBRATION/OFFSET**.

Les moteurs ont des **offsets différents** qui font que la position "neutre" (tous à 0) ne correspond pas à une tête droite.

**Preuve :**
- Le script montre que même en position "neutre", les stewart joints ne sont pas à 0 :
  - stewart_1: 0.00°
  - stewart_2: -22.85° ← **PROBLÈME ICI**
  - stewart_3: -11.34°
  - stewart_4: 32.78°
  - stewart_5: -19.16°
  - stewart_6: 43.51°

### Solutions

#### 1️⃣ Script de Correction Forcée

```bash
# Sur le robot
python3 /tmp/force_head_straight.py
```

Ce script va :
- Désactiver/réactiver les moteurs
- Faire des mouvements pour débloquer
- Essayer de repositionner la tête

#### 2️⃣ Correction Manuelle de la Position

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

#### 3️⃣ Recalibration des Offsets

**Si le problème persiste, il faut recalibrer les offsets des moteurs.**

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

**Option C : Ajustement manuel des offsets**

Les offsets sont dans la configuration hardware. Il faut les ajuster pour que la tête soit droite quand tous les stewart joints sont à leur position "neutre".

**Fichier de configuration :**
- `/home/pollen/.local/lib/python3.*/site-packages/reachy_mini/assets/config/hardware_config.yaml`

---

## 🔄 Reflash des Moteurs

### Quand Utiliser le Reflash

**Utiliser le reflash si :**
- Moteur avec paramètres d'usine (ID=1, baudrate 57,600)
- Moteurs non détectés après assemblage
- Problèmes de configuration persistants

### ⚠️ IMPORTANT - Ce n'est PAS un problème de montage !

Ce bug affecte plusieurs utilisateurs du lot décembre 2025. Votre assemblage est correct, c'est un problème logiciel qui se corrige avec le script de reflash.

**Confirmé par:**
- robertodipizzamano: "Motor 1 issue fixed for me by running the 592 branch reachy-mini-reflash-motors script"
- Post officiel Augustin (Pollen Team): "Head tilted, motor n°1 not moving, but get stiff when powered on - SOLVED"

### Procédure Officielle Complète

#### Étape 1: Préparation

1. **Alimenter le robot SANS démarrer le daemon:**
   - Brancher le robot
   - Mettre l'interrupteur sur **ON**
   - **NE PAS** ouvrir le dashboard
   - **NE PAS** démarrer le daemon

2. **Arrêter le daemon si déjà lancé:**
   ```bash
   # Sur le robot (SSH) ou localement:
   sudo systemctl stop reachy-mini-daemon
   ```

#### Étape 2: Mettre à jour le package

```bash
pip install --upgrade reachy-mini
```

#### Étape 3: Lancer le script de reflash

```bash
reachy-mini-reflash-motors
```

**Ce que fait le script:**
- Demande si vous avez la version **Lite** (USB) ou **Wireless** (WiFi)
- Détecte automatiquement les ports série:
  - Linux: `/dev/ttyUSB0`, `/dev/ttyAMA3`, etc.
  - Windows: `COM3`, `COM4`, etc.
  - macOS: `/dev/tty.usbserial-*`, etc.
- Scanne tous les moteurs
- Reprogramme automatiquement ceux qui ont une mauvaise configuration:
  - Change l'ID de 1 → ID correct (11-18)
  - Change le baudrate de 57,600 → 1,000,000

#### Étape 4: Redémarrer normalement

Une fois le reflash terminé:

```bash
# Redémarrer le daemon
sudo systemctl start reachy-mini-daemon

# Tester les mouvements de la tête
python examples/reachy_mini/fix_head_tilted.py
```

### Alternatives si la commande directe ne fonctionne pas

#### Option 1: Script wrapper

```bash
python examples/reachy_mini/reflash_motors_simple.py
```

#### Option 2: Module Python

```bash
python -m reachy_mini.tools.reflash_motors
```

#### Option 3: Spécifier le port série manuellement

```bash
reachy-mini-reflash-motors --serialport /dev/ttyAMA3
# ou
python -m reachy_mini.tools.reflash_motors --serialport /dev/ttyAMA3
```

### Dépannage Reflash

#### Erreur: "No module named 'reachy_mini.tools'"

**Solution:** Installer depuis la branche 592 (version de développement)

```bash
pip install git+https://github.com/pollen-robotics/reachy_mini.git@592
```

#### Erreur: "No Reachy Mini serial port found"

**Solutions:**
1. Vérifier la connexion USB/WiFi
2. Vérifier les permissions du port série:
   ```bash
   # Linux
   sudo chmod 666 /dev/ttyUSB0
   # ou ajouter votre utilisateur au groupe dialout
   sudo usermod -a -G dialout $USER
   ```
3. Spécifier le port manuellement avec `--serialport`

#### Erreur: "Multiple Reachy Mini serial ports found"

**Solution:** Spécifier le port manuellement

```bash
reachy-mini-reflash-motors --serialport /dev/ttyUSB0
```

---

## 🆘 Support Pollen Robotics

### Quand Contacter le Support

**Contacter le support si :**
- Toutes les solutions ci-dessus ont été essayées
- Le problème persiste après reflash
- Le moteur est probablement défectueux
- Problème de calibration d'usine

### Informations à Fournir

**Formulaire Pollen Robotics :**
https://forms.gle/JdhMzadeCnbynw7Q6

**Informations à copier-coller dans le formulaire :**

```
Problème: [Description du problème]

Actions effectuées:
- [ ] Reflash réussi (tous moteurs détectés)
- [ ] Câblage vérifié
- [ ] Câbles changés
- [ ] Moteur bouge mais clignote rouge
- [ ] Tête de travers (si applicable)

Diagnostic:
- Position moteur problématique: [valeur]
- Position dans les limites: [oui/non]
- Clignotement rouge persistant: [oui/non]

Demande:
- [Recalibration des offsets / Vérification/remplacement moteur si défectueux]
```

### Ressources

- **Discord Pollen Robotics:** https://discord.gg/pollen-robotics
- **GitHub Issues:** https://github.com/pollen-robotics/reachy_mini/issues
- **Documentation officielle:** https://github.com/pollen-robotics/reachy_mini/blob/develop/docs/troubleshooting.md
- **Guide d'assemblage:** https://huggingface.co/spaces/pollen-robotics/Reachy_Mini_Assembly_Guide

---

## 📋 Checklist Rapide

### Diagnostic Initial
- [ ] Robot éteint
- [ ] Capot enlevé
- [ ] Robot allumé
- [ ] Moteur qui clignote identifié (numéro noté)
- [ ] Diagnostic automatique lancé

### Vérifications Câblage
- [ ] Câbles vérifiés (bien branchés, pas pliés, pas endommagés)
- [ ] Ordre des câbles vérifié (daisy-chain)
- [ ] Câbles rebranchés correctement

### Vérifications Mécaniques
- [ ] Moteur testé manuellement (pas bloqué)
- [ ] Moteur pas en butée mécanique
- [ ] Position du moteur dans les limites

### Actions Correctives
- [ ] Daemon redémarré
- [ ] Robot redémarré (OFF/ON)
- [ ] Reflash effectué (si nécessaire)
- [ ] Tête repositionnée en position neutre

### Si Problème Persiste
- [ ] Diagnostic relancé
- [ ] Support contacté avec toutes les infos

---

**💡 Astuce:** Gardez le robot **éteint** pendant que vous vérifiez le câblage pour éviter tout risque.

**Note:** Ce guide est consolidé à partir de plusieurs guides de dépannage. Pour des problèmes spécifiques, consultez la documentation officielle ou contactez le support Pollen Robotics.

