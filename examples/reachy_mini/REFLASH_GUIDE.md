# 🔧 Guide Complet - Reflash Moteurs (Bug Décembre 2025)

**Date** : Décembre 2025  
**Problème** : Tête penchée, moteur 1 bloqué mais pas de LED rouge  
**Cause** : Moteur avec paramètres d'usine (ID=1, baudrate 57,600) au lieu de la config correcte  
**Solution** : Script officiel `reachy-mini-reflash-motors`

---

## ⚠️ IMPORTANT - Ce n'est PAS un problème de montage !

Ce bug affecte plusieurs utilisateurs du lot décembre 2025. Votre assemblage est correct, c'est un problème logiciel qui se corrige avec le script de reflash.

**Confirmé par:**
- robertodipizzamano: "Motor 1 issue fixed for me by running the 592 branch reachy-mini-reflash-motors script"
- Post officiel Augustin (Pollen Team): "Head tilted, motor n°1 not moving, but get stiff when powered on - SOLVED"

---

## ✅ Procédure Officielle Complète

### Étape 1: Préparation

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

### Étape 2: Mettre à jour le package

```bash
pip install --upgrade reachy-mini
```

### Étape 3: Lancer le script de reflash

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
  - Change l'ID de 1 → 13 (pour stewart_1)
  - Change le baudrate de 57,600 → 1,000,000

### Étape 4: Redémarrer normalement

Une fois le reflash terminé:

```bash
# Redémarrer le daemon
sudo systemctl start reachy-mini-daemon

# Tester les mouvements de la tête
python examples/reachy_mini/fix_head_tilted.py
```

---

## 🔄 Alternatives si la commande directe ne fonctionne pas

### Option 1: Script wrapper

```bash
python examples/reachy_mini/reflash_motors_simple.py
```

### Option 2: Module Python

```bash
python -m reachy_mini.tools.reflash_motors
```

### Option 3: Spécifier le port série manuellement

```bash
reachy-mini-reflash-motors --serialport /dev/ttyAMA3
# ou
python -m reachy_mini.tools.reflash_motors --serialport /dev/ttyAMA3
```

---

## 🐛 Dépannage

### Erreur: "No module named 'reachy_mini.tools'"

**Solution:** Installer depuis la branche 592 (version de développement)

```bash
pip install git+https://github.com/pollen-robotics/reachy_mini.git@592
```

### Erreur: "No Reachy Mini serial port found"

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

### Erreur: "Multiple Reachy Mini serial ports found"

**Solution:** Spécifier le port manuellement

```bash
reachy-mini-reflash-motors --serialport /dev/ttyUSB0
```

---

## 📚 Ressources

- **Documentation officielle:** https://github.com/pollen-robotics/reachy_mini/blob/develop/docs/troubleshooting.md
- **Discord Pollen Robotics:** https://discord.gg/Y7FgMqHsub
- **Post épinglé Discord:** "Head tilted, motor n°1 not moving, but get stiff when powered on - SOLVED"
- **Code source script:** https://github.com/pollen-robotics/reachy_mini/blob/develop/src/reachy_mini/tools/reflash_motors.py

---

## ✅ Après le reflash

Une fois le reflash réussi:

1. **Corriger la position de la tête:**
   ```bash
   python examples/reachy_mini/fix_head_tilted.py
   ```

2. **Tester les mouvements:**
   ```bash
   python examples/reachy_mini/minimal_demo.py
   ```

3. **Diagnostic complet:**
   ```bash
   python examples/reachy_mini/diagnostic_stewart.py
   ```

---

**💡 Astuce:** Si le problème persiste après le reflash, poster sur le Discord #support avec une vidéo du comportement et mentionner que vous avez le lot de décembre 2025.

