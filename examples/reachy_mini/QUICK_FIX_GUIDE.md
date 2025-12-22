# 🔧 Guide Rapide - Correction Moteur Clignotant Rouge

**Problème** : Moteur clignote rouge, "Missing motor stewart_3" dans l'app  
**Cause** : Moteur avec paramètres d'usine (ID=1, baudrate 57,600) au lieu de configuration correcte  
**Solution** : Reconfiguration du moteur vers ID correct, baudrate 1,000,000

---

## ⚡ Solution Rapide (Recommandée)

### Étape 1 : Se connecter au robot

```bash
ssh pollen@192.168.129.64
# Mot de passe : reachy-mini (ou celui que tu as configuré)
```

### Étape 2 : Arrêter le daemon

```bash
sudo systemctl stop reachy-mini-daemon
```

### Étape 3 : Activer l'environnement virtuel

```bash
source /venvs/mini-daemon/bin/activate
```

### Étape 4 : Scanner le bus Dynamixel

```python
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

### Étape 5 : Corriger le moteur (si détecté à 57.6k baud)

```python
python3 << 'EOF'
from importlib.resources import files
import reachy_mini
from reachy_mini.tools.setup_motor import setup_motor
from reachy_mini.utils.hardware_config.parser import parse_yaml_config

# Charger la configuration
config_file_path = str(files(reachy_mini).joinpath("assets/config/hardware_config.yaml"))
config = parse_yaml_config(config_file_path)

# Trouver le moteur problématique (stewart_3 = Motor ID 13)
motor_config = config.motors["stewart_3"]

print("🔧 Reconfiguration Motor ID 13 (stewart_3)...")
print("   - Ancien ID: 1 (paramètres d'usine)")
print("   - Nouveau ID: 13")
print("   - Ancien baudrate: 57,600")
print("   - Nouveau baudrate: 1,000,000")

# Reconfigurer
setup_motor(
    motor_config,
    "/dev/ttyAMA3",
    from_id=1,  # ID actuel (paramètres d'usine)
    from_baudrate=57600,  # Baudrate actuel
    target_baudrate=1000000,  # Baudrate cible
)

print("✅ Motor ID 13 reconfiguré avec succès")
EOF
```

### Étape 6 : Redémarrer le daemon

```bash
sudo systemctl start reachy-mini-daemon
```

### Étape 7 : Vérifier

Relancer le scan (étape 4) pour vérifier que le moteur est maintenant détecté à 1M baud.

---

## 🔄 Alternative : Script Automatique

Si tu as copié le script `diagnose_and_fix_motor_ssh.py` sur le robot :

```bash
ssh pollen@192.168.129.64
sudo systemctl stop reachy-mini-daemon
source /venvs/mini-daemon/bin/activate
python3 diagnose_and_fix_motor_ssh.py
```

Le script va :
1. Scanner automatiquement le bus à 1M et 57.6k baud
2. Détecter les moteurs avec mauvais baudrate
3. Proposer la correction automatique

---

## 📋 Mapping Motor ID → Joint

| Motor ID | Joint Name |
|----------|------------|
| 10 | yaw_body |
| 11 | stewart_1 |
| 12 | stewart_2 |
| 13 | stewart_3 ← **Le moteur problématique** |
| 14 | stewart_4 |
| 15 | stewart_5 |
| 16 | stewart_6 |
| 17 | left_antenna |
| 18 | right_antenna |

---

## 🆘 Si ça ne marche pas

1. **Vérifier le câblage** : Le moteur est-il bien connecté ?
2. **Redémarrer le robot** : Interrupteur OFF/ON
3. **Utiliser le script officiel** : `reachy-mini-reflash-motors`
4. **Contacter support** : Formulaire https://forms.gle/JdhMzadeCnbynw7Q6

---

**Basé sur la solution de squirrel (Discord Pollen Robotics - 20/12/2025)**

