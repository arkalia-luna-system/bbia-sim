# Diagnostic et reconfiguration des moteurs Reachy Mini (dans bbia-reachy-sim)

**Équivalent Pollen** : `scan_motors.py` = **`scan_motors_baudrate.py`** dans ce dépôt.  
**Équivalent Pollen** : `reachy_mini.tools.setup_motor` = utilisé par **`reachy-mini-reflash-motors`** sur le robot, ou par **`scan_motors_baudrate.py --auto-fix`** sur le robot.

---

## 1. Diagnostiquer (voir quels moteurs posent problème)

### Option A – Depuis le Mac (sans accès port série)

Le scan « vrai » (bus Dynamixel) a besoin du port série du robot → il faut lancer le script **sur le robot** via SSH. **Sur le robot, le scan avec `--serialport` nécessite le module `reachy_mini_motor_controller`.** S’il n’est pas installé, le script affiche un message et propose d’utiliser **`reachy-mini-reflash-motors`** à la place (recommandé).

**Sur le robot, pour reconfigurer sans se prendre la tête :** garder le daemon démarré et lancer **`reachy-mini-reflash-motors`** (script officiel). Pas besoin de notre scan.

**Si tu veux quand même lancer le scan bbia** (depuis le Mac, puis SSH) :

```bash
# Copier le script sur le robot
scp examples/reachy_mini/scan_motors_baudrate.py pollen@reachy-mini.local:/tmp/

# Se connecter au robot
ssh pollen@reachy-mini.local
```

Puis **sur le robot** :
- **Scan direct (daemon arrêté)** : `sudo systemctl stop reachy-mini-daemon` puis `python3 /tmp/scan_motors_baudrate.py --serialport /dev/ttyAMA3`. **Si le script dit « Module motor_controller non disponible »** → utilise **`reachy-mini-reflash-motors`** (redémarrer le daemon avant).
- **Diagnostic via SDK (daemon démarré)** : `sudo systemctl start reachy-mini-daemon` puis `python3 /tmp/scan_motors_baudrate.py` (sans `--serialport`).

Tu peux aussi utiliser le **wrapper** `scan_motors.py` (même chose que `scan_motors_baudrate.py`).

### Option B – Script SSH tout-en-un (depuis le Mac)

Le script **`scan_motors_baudrate_ssh.py`** copie et exécute le scan sur le robot (à adapter si ton robot n’a pas le SDK / le chemin du script) :

```bash
python3 examples/reachy_mini/scan_motors_baudrate_ssh.py
```

(À lancer depuis la racine du dépôt ; le script doit pouvoir être copié sur le robot.)

---

## 2. Reconfigurer (corriger ID / baudrate)

**À faire sur le robot** (SSH), pas depuis le Mac (pas d’accès port série).

### Méthode 1 – Script officiel Pollen (recommandé)

```bash
ssh pollen@reachy-mini.local
reachy-mini-reflash-motors
```

Choisir **Wireless**, laisser le script reconfigurer tous les moteurs. Il utilise en interne la logique type `setup_motor`.

### Méthode 2 – Scan + correction automatique (bbia)

```bash
ssh pollen@reachy-mini.local
sudo systemctl stop reachy-mini-daemon

# Copier le script depuis ton Mac si pas déjà fait
# Depuis le Mac : scp examples/reachy_mini/scan_motors_baudrate.py pollen@reachy-mini.local:/tmp/

python3 /tmp/scan_motors_baudrate.py --serialport /dev/ttyAMA3 --auto-fix
```

Le script va détecter les moteurs en config usine (ID=1 @ 57 600) et les reconfigurer (ID correct @ 1 000 000) **si** le module `reachy_mini.tools.setup_motor` est disponible sur le robot (SDK reachy-mini installé).

### Méthode 3 – Reconfiguration manuelle (squirrel / Discord)

Si le SDK sur le robot expose le module en ligne de commande :

```bash
ssh pollen@reachy-mini.local
sudo systemctl stop reachy-mini-daemon
source /venvs/mini-daemon/bin/activate   # ou le venv du robot
python3 -m reachy_mini.tools.setup_motor
```

(Sous réserve que le package reachy_mini sur le robot ait un `setup_motor` exécutable en `__main__`.)

### Méthode 4 – Script bbia (un moteur ciblé)

Pour un **moteur précis** (ex. stewart_2, ID 12) encore en ID=1 @ 57 600 :

```bash
# Sur le robot, après avoir copié le script
python3 /tmp/fix_motor_config_december_bug.py --serialport /dev/ttyAMA3 --motor-id 12
```

(À adapter selon les options du script ; voir `fix_motor_config_december_bug.py --help`.)

---

## 3. Où sont les « bons » fichiers dans bbia

| Ce que dit Pollen | Fichier / commande dans bbia-reachy-sim |
|-------------------|----------------------------------------|
| `scan_motors.py` pour diagnostiquer | **`examples/reachy_mini/scan_motors_baudrate.py`** ou **`examples/reachy_mini/scan_motors.py`** (wrapper) |
| `reachy_mini.tools.setup_motor` pour reconfigurer | **`reachy-mini-reflash-motors`** sur le robot, ou **`scan_motors_baudrate.py --auto-fix`** sur le robot (utilise setup_motor si dispo) |
| Reconfigurer un moteur précis | **`examples/reachy_mini/fix_motor_config_december_bug.py`** (sur le robot, avec --serialport et --motor-id) |

---

## 4. Ordre conseillé pour tout corriger

1. **Câbles** : tout rebrancher, bien enfoncer, surtout headboard → moteur 1. Tester les câbles de secours.  
2. **Diagnostic** : sur le robot, `scan_motors_baudrate.py --serialport /dev/ttyAMA3` (ou `scan_motors.py`).  
3. **Reconfig** : sur le robot, `reachy-mini-reflash-motors` (ou `scan_motors_baudrate.py --auto-fix` si tu préfères).  
4. **Daemon** : `sudo systemctl start reachy-mini-daemon`.  
5. **Tête tordue / moteurs 1 et 2** : depuis le Mac, `python3 examples/reachy_mini/fix_motors_1_2_overload_ssh.py --robot-ip reachy-mini.local`.  
6. Si ça bloque encore : **Pollen** (calibration / offsets).

Voir aussi : **`docs/hardware/DESCRIPTION_PROBLEME_ET_COMPOSANTS_REACHY_MINI.md`** (section « Que faire pour corriger »).
