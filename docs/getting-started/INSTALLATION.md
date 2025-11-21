# 🔧 Installation BBIA-SIM

> **Guide centralisé d'installation - Source de vérité unique**

**Dernière mise à jour** : 21 novembre 2025

---

## ⚡ Installation Rapide

### Option 1 : Script All-in-One (Recommandé) ⚡

```bash
# Cloner le projet
git clone https://github.com/arkalia-luna-system/bbia-sim.git
cd bbia-sim

# Lancer le script all-in-one (fait tout automatiquement)
./scripts/reachy-mini-sim-starter.sh

# Le script :
# ✅ Vérifie Python 3.11+, pip, mjpython (macOS)
# ✅ Crée/active l'environnement virtuel
# ✅ Installe BBIA-SIM + dépendances
# ✅ Vérifie l'installation (bbia_doctor)
# ✅ Lance le dashboard sur http://localhost:8000
```

**Options disponibles** :
- `--skip-install` : Vérification uniquement (sans installation)
- `--skip-dashboard` : Installation sans lancer le dashboard
- `--help` : Aide complète

### Option 2 : Installation Manuelle

```bash
# Cloner le projet
git clone https://github.com/arkalia-luna-system/bbia-sim.git
cd bbia-sim

# Installer les dépendances (mode développement recommandé)
pip install -e .[dev]

# Pour une installation minimale :
pip install -e .
```

---

## 📋 Prérequis

### Python

- **Version requise** : Python 3.11+
- **Installation avec pyenv** (recommandé) :
  ```bash
  pyenv install 3.11.9 && pyenv local 3.11.9
  python -m pip install --upgrade pip
  ```

### Environnement virtuel (recommandé)

```bash
# Créer l'environnement virtuel
python -m venv venv

# Activer (macOS/Linux)
source venv/bin/activate

# Activer (Windows)
venv\Scripts\activate
```

---

## 🎯 Vérification de l'installation

```bash
# Vérifier l'installation complète
bbia_doctor

# Tester l'import
python -c "from bbia_sim import RobotFactory; print('✅ Installation OK')"
```

---

## 📦 Extras disponibles

BBIA-SIM propose plusieurs extras pour différentes utilisations :

- `[dev]` : Dépendances de développement (tests, linting, etc.)
- `[test]` : Dépendances pour les tests
- `[audio]` : Dépendances audio (speech recognition, TTS, etc.)

**Exemple** :
```bash
pip install -e .[dev,audio]
```

---

## 🐳 Installation avec Docker

```bash
# Build de l'image
docker build -t bbia-sim .

# Démarrage du conteneur
docker run -p 8000:8000 bbia-sim

# Avec variables d'environnement
docker run -p 8000:8000 -e BBIA_API_TOKEN=secret bbia-sim
```

---

## 🔧 Configuration

### Variables d'environnement

```bash
# Développement
export BBIA_API_HOST=127.0.0.1
export BBIA_API_PORT=8000
export BBIA_LOG_LEVEL=info
export MUJOCO_GL=egl

# Production
export BBIA_API_HOST=0.0.0.0
export BBIA_API_PORT=8000
export BBIA_LOG_LEVEL=warning
export BBIA_API_TOKEN=your_secret_token
export BBIA_RATE_LIMIT=100
export MUJOCO_GL=egl
```

---

## 🚀 Démarrage rapide

Après l'installation, voir le [Guide de Démarrage](../guides/GUIDE_DEMARRAGE.md) pour les premiers pas.

---

## ❓ Dépannage

Pour les problèmes d'installation, consultez :
- [Guide Troubleshooting](troubleshooting.md)
- [FAQ Installation](../development/troubleshooting.md)

---

**Source unique** : Ce fichier est la référence pour toutes les instructions d'installation  
**Mise à jour** : Automatique lors des changements de configuration

