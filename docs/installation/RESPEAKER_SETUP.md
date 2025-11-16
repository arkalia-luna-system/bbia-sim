# 🎤 Guide ReSpeaker - Configuration et Dépannage

<div align="center">

**Date** : Oct / Nov. 2025  
**Hardware** : ReSpeaker 4 microphones directionnels (Reachy Mini)

[![🎤 Audio](https://img.shields.io/badge/🎤-Audio%20Hardware-blue.svg)](#-vue-densemble)
[![🔧 Configuration](https://img.shields.io/badge/🔧-Configuration-green.svg)](#-détection-et-configuration)
[![🧪 Tests](https://img.shields.io/badge/🧪-Tests%20Audio-orange.svg)](#-scripts-de-test)
[![🔍 Dépannage](https://img.shields.io/badge/🔍-Dépannage-purple.svg)](#-dépannage)

</div>

> **Référence SDK** : `robot.media.microphone` - 4 microphones directionnels avec annulation de bruit automatique

---

## 📋 Vue d'ensemble

<div align="center">

### 🎯 ReSpeaker 4 Microphones Directionnels

**Capture audio avancée pour robot Reachy Mini**

[![🎤 Hardware](https://img.shields.io/badge/🎤-4%20Microphones-blue)](#-canaux-disponibles)
[![🔇 Noise Cancel](https://img.shields.io/badge/🔇-Annulation%20Bruit-green)](#-vue-densemble)
[![📍 DoA](https://img.shields.io/badge/📍-Localisation%20Source-orange)](#-vue-densemble)

</div>

Le Reachy Mini utilise un **ReSpeaker avec 4 microphones directionnels** pour la capture audio avancée :

- ✅ **4 microphones directionnels** : Localisation de source audio (DoA - Direction of Arrival)
- ✅ **Annulation de bruit automatique** : Filtrage du bruit ambiant
- ✅ **Sample rate** : 16 kHz (aligné SDK Reachy Mini)
- ✅ **Channels** : 4 canaux (1 par microphone) ou mixage mono/stéréo

---

## 🔧 Détection et Configuration

<div align="center">

### 🚀 Détection automatique et configuration simple

[![🔍 Détection](https://img.shields.io/badge/🔍-Détection%20Auto-blue)](#1-détection-automatique)
[![✅ Vérification](https://img.shields.io/badge/✅-Vérification%20Script-green)](#2-vérification-via-script)
[![🎛️ Canaux](https://img.shields.io/badge/🎛️-Configuration%20Canaux-orange)](#-configuration-canaux)

</div>

### 1. Détection Automatique

Le SDK Reachy Mini détecte automatiquement le ReSpeaker via `robot.media.microphone` :

```python
from bbia_sim.robot_factory import RobotFactory

# Créer robot (détection automatique ReSpeaker)
robot = RobotFactory.create_backend("reachy")
robot.connect()

# Accéder au microphone ReSpeaker
if robot.media and robot.media.microphone:
    logging.info("✅ ReSpeaker détecté")
    logging.info(f"Canaux disponibles: {robot.media.microphone.channels}")
    logging.info(f"Sample rate: {robot.media.microphone.sample_rate}")
else:
    logging.warning("⚠️ ReSpeaker non disponible (mode simulation)")
```

### 2. Vérification via Script

Créer un script de test simple :

```python
#!/usr/bin/env python3
"""Test détection ReSpeaker"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory

def test_respeaker():
    """Test détection ReSpeaker."""
    robot = RobotFactory.create_backend("reachy")
    
    if not robot:
        logging.error("❌ Robot non disponible")
        return False
    
    connected = robot.connect()
    if not connected:
        logging.warning("⚠️ Robot non connecté (mode simulation)")
        return False
    
    # Vérifier microphone
    if robot.media and robot.media.microphone:
        mic = robot.media.microphone
        logging.info("✅ ReSpeaker détecté")
        logging.info(f"   Canaux: {getattr(mic, 'channels', 'N/A')}")
        logging.info(f"   Sample rate: {getattr(mic, 'sample_rate', 'N/A')} Hz")
        return True
    else:
        logging.error("❌ ReSpeaker non détecté")
        return False

if __name__ == "__main__":
    test_respeaker()
```

---

## 🎛️ Configuration Canaux

<div align="center">

### 🎯 4 Canaux Audio Directionnels

[![📊 Tableau](https://img.shields.io/badge/📊-Tableau%20Canaux-blue)](#-canaux-disponibles)
[![🔧 Config](https://img.shields.io/badge/🔧-Configuration%20Avancée-green)](#-configuration-sample-rate)
[![🎙️ Multi-Canal](https://img.shields.io/badge/🎙️-Multi-Canal-orange)](#-enregistrement-multi-canal)

</div>

### Canaux Disponibles

Le ReSpeaker expose **4 canaux** (1 par microphone) :

| Canal | Microphone | Direction | Usage |
|:-----:|:----------:|:---------:|:-----:|
| 0 | Mic 1 | Avant | Capture principale |
| 1 | Mic 2 | Arrière | Capture arrière |
| 2 | Mic 3 | Gauche | Localisation DoA |
| 3 | Mic 4 | Droite | Localisation DoA |

### Configuration Sample Rate

**Recommandé** : 16 kHz (aligné SDK Reachy Mini)

```python
# Configuration par défaut BBIA
DEFAULT_SAMPLE_RATE = 16000  # 16 kHz
DEFAULT_BUFFER_SIZE = 512    # Latence minimale
DEFAULT_CHANNELS = 1         # Mono (mixage automatique)
```

### Enregistrement Multi-Canal

```python
# Enregistrement 4 canaux (tous les microphones)
audio_4ch = robot.media.record_audio(
    duration=3.0,
    sample_rate=16000,
    channels=4  # 4 canaux
)

# Enregistrement mono (mixage automatique)
audio_mono = robot.media.record_audio(
    duration=3.0,
    sample_rate=16000,
    channels=1  # Mono
)
```

---

## 🖥️ Matrices de Compatibilité OS

<div align="center">

### 💻 Support Multi-Plateformes

[![🍎 macOS](https://img.shields.io/badge/🍎-macOS-blue)](#macos)
[![🐧 Linux](https://img.shields.io/badge/🐧-Linux-green)](#linux-ubuntudebian)
[![🪟 Windows](https://img.shields.io/badge/🪟-Windows-orange)](#windows)

</div>

### macOS

| Version | Support | Notes |
|:-------:|:-------:|:-----:|
| macOS 13+ (Ventura) | ✅ | Support natif Core Audio |
| macOS 12 (Monterey) | ✅ | Support natif Core Audio |
| macOS 11 (Big Sur) | ⚠️ | Peut nécessiter permissions |

**Installation** :

```bash
# PortAudio (requis pour PyAudio)
brew install portaudio

# PyAudio
pip install pyaudio
```

**Permissions** : Autoriser accès microphone dans **Réglages Système > Confidentialité > Microphone**

### Linux (Ubuntu/Debian)

| Version | Support | Notes |
|:-------:|:-------:|:-----:|
| Ubuntu 22.04+ | ✅ | ALSA natif |
| Ubuntu 20.04 | ✅ | ALSA natif |
| Debian 11+ | ✅ | ALSA natif |

**Installation** :

```bash
# ALSA et PortAudio
sudo apt-get update
sudo apt-get install -y portaudio19-dev python3-pyaudio alsa-utils

# Vérifier devices
arecord -l  # Liste devices d'enregistrement
```

**Configuration ALSA** : Vérifier `/etc/asound.conf` si nécessaire

### Windows

| Version | Support | Notes |
|:-------:|:-------:|:-----:|
| Windows 11 | ✅ | DirectX natif |
| Windows 10 | ✅ | DirectX natif |

**Installation** :

1. Télécharger PortAudio : <http://files.portaudio.com/download.html>
2. Installer PyAudio :
   ```bash
   pip install pyaudio
   ```

**Configuration** : Vérifier périphériques audio dans **Paramètres > Système > Son**

---

## 🧪 Scripts de Test

<div align="center">

### 🔧 Scripts de test complets

[![🎤 Sound In](https://img.shields.io/badge/🎤-Sound%20In-blue)](#test-sound-inout)
[![🔊 Sound Out](https://img.shields.io/badge/🔊-Sound%20Out-green)](#test-sound-inout)
[![📋 Devices](https://img.shields.io/badge/📋-Devices%20List-orange)](#test-détection-devices)

</div>

### Test Sound In/Out

Créer `scripts/test_respeaker.py` :

```python
#!/usr/bin/env python3
"""Test ReSpeaker sound in/out"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from bbia_sim.robot_factory import RobotFactory
import time

def test_sound_in():
    """Test enregistrement audio (sound in)."""
    robot = RobotFactory.create_backend("reachy")
    
    if not robot or not robot.connect():
        logging.error("❌ Robot non disponible")
        return False
    
    if not robot.media or not robot.media.microphone:
        logging.error("❌ ReSpeaker non disponible")
        return False
    
    logging.info("🎤 Test enregistrement (3 secondes)...")
    try:
        audio = robot.media.record_audio(duration=3.0, sample_rate=16000)
        if audio:
            logging.info(f"✅ Enregistrement OK ({len(audio)} échantillons)")
            return True
        else:
        logging.error("❌ Enregistrement vide")
            return False
    except (OSError, RuntimeError, ValueError) as e:
        logging.error(f"❌ Erreur enregistrement: {e}")
        return False

def test_sound_out():
    """Test lecture audio (sound out)."""
    robot = RobotFactory.create_backend("reachy")
    
    if not robot or not robot.connect():
        logging.error("❌ Robot non disponible")
        return False
    
    if not robot.media or not robot.media.speaker:
        logging.error("❌ Speaker non disponible")
        return False
    
    logging.info("🔊 Test lecture audio...")
    try:
        # Générer tone de test (440 Hz, 1 seconde)
        import numpy as np
        sample_rate = 16000
        duration = 1.0
        t = np.linspace(0, duration, int(sample_rate * duration))
        tone = np.sin(2 * np.pi * 440 * t).astype(np.float32)
        
        robot.media.speaker.play(tone, sample_rate=sample_rate)
        logging.info("✅ Lecture OK")
        return True
    except (OSError, RuntimeError, ValueError) as e:
        logging.error(f"❌ Erreur lecture: {e}")
        return False

if __name__ == "__main__":
    logging.info("🧪 Tests ReSpeaker\n")
    
    logging.info("1. Test Sound In (microphone)")
    test_sound_in()
    
    logging.info("\n2. Test Sound Out (speaker)")
    test_sound_out()
    
    logging.info("\n✅ Tests terminés")
```

### Test Détection Devices

```python
#!/usr/bin/env python3
"""Liste devices audio disponibles"""

try:
    import pyaudio
    
    p = pyaudio.PyAudio()
    
    logging.info("📋 Devices audio disponibles:\n")
    
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info.get("maxInputChannels") > 0:
            logging.info(f"Device {i}: {info.get('name')}")
            logging.info(f"  Input channels: {info.get('maxInputChannels')}")
            logging.info(f"  Sample rate: {info.get('defaultSampleRate')} Hz")
            logging.info("")
    
    p.terminate()
    
except ImportError:
    logging.error("❌ PyAudio non disponible")
    logging.error("Installer: pip install pyaudio")
except (OSError, RuntimeError, ValueError) as e:
    logging.error(f"❌ Erreur: {e}")
```

---

## 🔍 Dépannage

<div align="center">

### 🛠️ Solutions aux problèmes courants

[![❌ Non détecté](https://img.shields.io/badge/❌-Non%20Détecté-red)](#problème--respeaker-non-détecté)
[![🔉 Mauvaise qualité](https://img.shields.io/badge/🔉-Mauvaise%20Qualité-orange)](#problème--audio-de-mauvaise-qualité)
[![⏱️ Latence](https://img.shields.io/badge/⏱️-Latence%20Élevée-yellow)](#problème--latence-élevée)

</div>

### Problème : ReSpeaker non détecté

**Solutions** :

1. **Vérifier connexion USB** :
   ```bash
   # Linux
   lsusb | grep -i respeaker
   
   # macOS
   system_profiler SPUSBDataType | grep -i respeaker
   ```

2. **Vérifier permissions** :
   - macOS : Réglages Système > Confidentialité > Microphone
   - Linux : Vérifier groupe `audio` : `groups $USER`
   - Windows : Paramètres > Confidentialité > Microphone

3. **Vérifier drivers** :
   ```bash
   # Linux - vérifier ALSA
   arecord -l
   
   # macOS - vérifier Core Audio
   system_profiler SPAudioDataType
   ```

### Problème : Audio de mauvaise qualité

**Solutions** :

1. **Vérifier sample rate** : Utiliser 16 kHz (recommandé SDK)
2. **Vérifier buffer size** : Utiliser 512 (latence minimale)
3. **Vérifier distance microphones** : Positionner à 30-50 cm du robot

### Problème : Latence élevée

**Solutions** :

1. **Réduire buffer size** : 256 ou 512 échantillons
2. **Vérifier sample rate** : 16 kHz (pas 44.1 kHz)
3. **Désactiver traitement audio système** : Égaliseurs, effets

---

## 📚 Références

- **SDK Reachy Mini** : <https://github.com/pollen-robotics/reachy_mini>
- **Documentation audio BBIA** : `docs/installation/AUDIO_SETUP.md`
- **Module BBIA Audio** : `src/bbia_sim/bbia_audio.py`
- **Module BBIA Voice** : `src/bbia_sim/bbia_voice.py`

---

## 🎯 Navigation

**Retour à** : [README Documentation](../../README.md)  
**Voir aussi** : [Guide Audio](../installation/AUDIO_SETUP.md) • [Troubleshooting](../getting-started/troubleshooting.md)

---

**Dernière mise à jour** : Oct / Nov. 2025

