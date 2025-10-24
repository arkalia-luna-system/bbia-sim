# Installation des dépendances audio pour BBIA-SIM

## Installation standard (sans audio)

```bash
pip install -e .
```

## Installation avec fonctionnalités audio complètes

### Prérequis système

**Ubuntu/Debian :**
```bash
sudo apt-get update
sudo apt-get install portaudio19-dev python3-dev
```

**macOS :**
```bash
brew install portaudio
```

**Windows :**
Téléchargez PortAudio depuis https://www.portaudio.com/

### Installation Python

```bash
# Option 1 : Via les dépendances optionnelles
pip install -e .[audio]

# Option 2 : Via le fichier requirements-audio.txt
pip install -r requirements-audio.txt

# Option 3 : Installation manuelle
pip install pyaudio
```

## Fonctionnalités affectées

- **Avec pyaudio** : Reconnaissance vocale complète via microphone
- **Sans pyaudio** : 
  - ✅ Synthèse vocale (TTS) fonctionne
  - ✅ Enregistrement/lecture audio via sounddevice fonctionne
  - ❌ Reconnaissance vocale via microphone non disponible
  - ⚠️ Messages d'avertissement dans les logs

## Tests

Les tests continuent de fonctionner même sans pyaudio. Les tests de reconnaissance vocale retournent `None` au lieu de lever une exception.
