# BBIA Reachy Mini Wireless – Simulation Python

![Bannière Reachy Mini](assets/images/Capture d'écran 2025-07-15 à 04.09.24.png)

[![Licence MIT](https://img.shields.io/badge/Licence-MIT-blue.svg)](./LICENCE)
[![Python](https://img.shields.io/badge/python-3.9%2B-blue)](https://www.python.org/)
[![Tests](https://img.shields.io/badge/tests-automatisés-green)](./tests/README.md)
[![Couverture](https://codecov.io/gh/arkalia-luna-system/bbia-sim/branch/main/graph/badge.svg)](https://codecov.io/gh/arkalia-luna-system/bbia-sim)

---

## 🚀 Démarrage en 30 secondes

**3 commandes pour voir BBIA en action :**

```bash
# 1. Démarrer l'API (dans un terminal)
uvicorn src.bbia_sim.daemon.app.main:app --port 8000

# 2. Lancer la démo "BBIA réagit" (dans un autre terminal)
python examples/behave_follow_face.py --token bbia-secret-key-dev

# 3. Vérifier que tout fonctionne
python -m bbia_sim --sim --headless --duration 1
```

**🎯 Résultat attendu :** La tête du robot suit un mouvement oscillant pendant 10 secondes, puis retourne à sa position initiale.

---

## Table des matières
- [BBIA Reachy Mini Wireless – Simulation Python](#bbia-reachy-mini-wireless--simulation-python)
  - [🚀 Démarrage en 30 secondes](#-démarrage-en-30-secondes)
  - [Table des matières](#table-des-matières)
  - [Présentation](#présentation)
  - [Capture d'écran](#capture-décran)
  - [Structure du projet](#structure-du-projet)
  - [🤖 Contrôle Temps Réel du Robot](#-contrôle-temps-réel-du-robot)
  - [Installation rapide](#installation-rapide)
    - [Développement](#développement)
    - [Production](#production)
  - [Simulation 3D (MuJoCo)](#simulation-3d-mujoco)
    - [Mode graphique](#mode-graphique)
    - [Mode headless (test)](#mode-headless-test)
    - [Exemple simple](#exemple-simple)
  - [Mode 3D visible (MuJoCo)](#mode-3d-visible-mujoco)
    - [Linux](#linux)
    - [macOS](#macos)
    - [Dépannage Mode 3D](#dépannage-mode-3d)
    - [Détermination durée headless garantie (tests)](#détermination-durée-headless-garantie-tests)
    - [Utiliser les assets officiels](#utiliser-les-assets-officiels)
  - [API REST/WebSocket](#api-restwebsocket)
    - [Démarrage API](#démarrage-api)
    - [Authentification Bearer](#authentification-bearer)
    - [Endpoints clés](#endpoints-clés)
  - [Exemples d'utilisation](#exemples-dutilisation)
  - [Audio \& Voix sur macOS](#audio--voix-sur-macos)
  - [Lancer les tests](#lancer-les-tests)
  - [Sécurité \& perfs](#sécurité--perfs)
    - [Configuration environnement](#configuration-environnement)
    - [Commandes d'audit sécurité](#commandes-daudit-sécurité)
    - [CORS et limites (prod)](#cors-et-limites-prod)
    - [Démarrage production](#démarrage-production)
  - [Tests \& CI](#tests--ci)
    - [Tests unitaires](#tests-unitaires)
    - [Tests avec couverture](#tests-avec-couverture)
    - [CI complète](#ci-complète)
    - [Exemples automatisés](#exemples-automatisés)
  - [Troubleshooting](#troubleshooting)
    - [MuJoCo](#mujoco)
    - [uvicorn](#uvicorn)
    - [API](#api)
    - [Audio \& Voix](#audio--voix)
    - [Général](#général)
  - [Documentation](#documentation)
  - [Roadmap](#roadmap)
  - [Contribuer](#contribuer)
  - [Licence](#licence)
  - [🛠️ Conseils pratiques pour fiabiliser et améliorer BBIA](#️-conseils-pratiques-pour-fiabiliser-et-améliorer-bbia)

---

## Présentation
Projet BBIA pour Reachy Mini Wireless : IA émotionnelle, simulation 100% Python, stable, testée, documentée.

## Capture d'écran
![Capture d'écran Reachy Mini](assets/images/Capture d’écran 2025-07-15 à 04.09.24.png)

## Structure du projet
- `src/bbia_sim/` : modules principaux (réveil, audio, voix, émotions, vision)
- `src/bbia_sim/sim/` : simulation MuJoCo (modèles MJCF/XML, simulateur)
- `src/bbia_sim/daemon/` : API REST/WebSocket (FastAPI)
- `tests/` : tests automatisés pour chaque module
- `docs/` : documentation complète et guides
- `scripts/` : scripts de démarrage et tests

## 🤖 Contrôle Temps Réel du Robot

**BBIA-SIM** permet de contrôler le robot Reachy Mini en temps réel via API REST :

```python
# Contrôle des articulations
import httpx

headers = {"Authorization": "Bearer bbia-secret-key-dev"}

# Déplacer le cou
response = httpx.post("http://localhost:8000/api/motion/joints", 
                     json=[{"joint_name": "neck_yaw", "position": 0.5}],
                     headers=headers)
print(response.json())  # {"status": "moving", "success_count": 1}

# Contrôler la tête
response = httpx.post("http://localhost:8000/api/motion/head",
                     json={"yaw": 0.3, "pitch": 0.1},
                     headers=headers)

# Retour à la position d'origine
response = httpx.post("http://localhost:8000/api/motion/home", headers=headers)

# Récupérer l'état du robot
response = httpx.get("http://localhost:8000/api/state/joints", headers=headers)
joints = response.json()["joints"]
print(f"Position cou: {joints['neck_yaw']['position']:.3f} rad")
```

**🎯 Résultat :** Les commandes API contrôlent vraiment le robot 3D dans MuJoCo !

## Installation rapide

### Développement
```bash
git clone https://github.com/arkalia-luna-system/bbia-sim.git
cd bbia-sim
python3 -m venv venv
source venv/bin/activate
pip install -e ".[dev]"
```

### Production
```bash
pip install -e ".[prod]"
cp .env.example .env
# Modifier .env avec vos valeurs
```

## Simulation 3D (MuJoCo)

### Mode graphique
```bash
python -m bbia_sim --sim
```

### Mode headless (test)
```bash
python -m bbia_sim --sim --headless --duration 2
```

### Exemple simple
```bash
python examples/hello_sim.py --duration 2 --verbose
```

## Mode 3D visible (MuJoCo)

### Linux
```bash
# Installation des dépendances
pip install mujoco-python-viewer

# Lancement avec fenêtre 3D
python -m bbia_sim --sim --verbose
```

### macOS
```bash
# Installation des dépendances
pip install mujoco-python-viewer

# Lancement avec fenêtre 3D (nécessite mjpython)
mjpython -m bbia_sim --sim --verbose

# Alternative : mode headless
python -m bbia_sim --sim --headless --duration 5
```

### Dépannage Mode 3D
- **macOS** : Si erreur "mjpython required", utilisez `mjpython` au lieu de `python`
- **Linux** : Vérifiez que `DISPLAY` est défini et que les drivers graphiques sont installés
- **Erreur viewer** : Installez `mujoco-python-viewer` avec `pip install mujoco-python-viewer`
- **Mode headless** : Utilisez `--headless` si pas d'affichage graphique disponible
- **Erreur GLFW/EGL** : Installez les drivers graphiques et bibliothèques OpenGL

### Détermination durée headless garantie (tests)
La durée en mode headless est strictement respectée avec une tolérance de ±0.05s grâce à :
- Utilisation de `time.monotonic()` pour éviter la dérive temporelle
- Vérification après chaque step de simulation
- Tests automatisés validant la précision temporelle

### Utiliser les assets officiels

✅ **Assets officiels intégrés !** Ce projet utilise maintenant les vrais modèles 3D de Reachy Mini.

**Source :** Dépôt officiel Pollen Robotics - https://github.com/pollen-robotics/reachy_mini (v1.0.0rc5)

**Assets intégrés :**
- **Corps :** `body_top_3dprint.stl`, `body_down_3dprint.stl`, `body_foot_3dprint.stl`
- **Tête :** `head_front_3dprint.stl`, `head_back_3dprint.stl`, `head_mic_3dprint.stl`
- **Bras Stewart :** `mp01062_stewart_arm_3.stl`, `stewart_link_rod.stl`
- **Plateforme :** `stewart_main_plate_3dprint.stl`, `stewart_tricap_3dprint.stl`

**Localisation :** `src/bbia_sim/sim/assets/reachy_official/`

**Documentation complète :** Voir `src/bbia_sim/sim/assets/reachy_official/OFFICIAL_ASSETS.md`

## API REST/WebSocket

### Démarrage API
```bash
# Développement
uvicorn src.bbia_sim.daemon.app.main:app --port 8000 --reload

# Production
BBIA_ENV=prod BBIA_TOKEN=votre-token uvicorn src.bbia_sim.daemon.app.main:app --port 8000
```

### Authentification Bearer
```bash
# Tous les endpoints nécessitent le header
curl -H "Authorization: Bearer votre-token" http://localhost:8000/api/state/full
```

### Endpoints clés
```bash
# État robot
curl -H "Authorization: Bearer bbia-secret-key-dev" http://localhost:8000/api/state/joints

# Mouvement
curl -X POST -H "Authorization: Bearer bbia-secret-key-dev" -H "Content-Type: application/json" \
  "http://localhost:8000/api/motion/joints" \
  -d '[{"joint_name": "neck_yaw", "position": 0.6}]'

# WebSocket télémétrie
python examples/subscribe_telemetry.py --token bbia-secret-key-dev --count 5
```

## Exemples d'utilisation
```python
# Simulation MuJoCo
from src.bbia_sim.sim.simulator import MuJoCoSimulator
simulator = MuJoCoSimulator("src/bbia_sim/sim/models/reachy_mini.xml")
simulator.launch_simulation(headless=True, duration=5)

# API REST
import httpx
response = httpx.get("http://localhost:8000/api/state/full", 
                    headers={"X-API-Key": "supersecrettoken"})
print(response.json())

# WebSocket
import asyncio
import websockets
async def test_ws():
    async with websockets.connect("ws://localhost:8000/ws/telemetry") as ws:
        data = await ws.recv()
        print(json.loads(data))

# Synthèse vocale
from src.bbia_sim.bbia_voice import dire_texte, reconnaitre_parole
dire_texte("Bonjour, je suis BBIA.")
texte = reconnaitre_parole(duree=3)
print(texte)
```

## Audio & Voix sur macOS
- **Voix utilisée** : Amélie (français Canada, ID macOS : com.apple.voice.compact.fr-CA.Amelie)
- **Installer la voix** : Préférences Système > Accessibilité > Parole > Voix du système > Personnaliser…
- **Dépendances** : pyttsx3, speechrecognition, pyaudio, sounddevice, numpy
- **Installer portaudio** :
```bash
brew install portaudio
pip install pyaudio
```
- [Guide installation/dépannage audio/voix](#troubleshooting)

## Lancer les tests
```bash
# Tests unitaires
python -m unittest discover tests

# Tests avec couverture
pytest --cov=bbia_sim --cov-report=html

# Tests de l'API
python scripts/test_api.py

# Linting et formatage
ruff check src/ tests/
black src/ tests/
mypy src/
bandit -r src/
```

## Sécurité & perfs

### Configuration environnement
```bash
# Copier le fichier d'exemple
cp .env.example .env

# Modifier les valeurs selon votre environnement
# BBIA_ENV=dev  # ou prod
# BBIA_TOKEN=votre-token-secret
```

### Commandes d'audit sécurité
```bash
# Audit des dépendances (CRITICAL/HIGH)
pip-audit --desc

# Analyse statique de sécurité
bandit -r src/ -c .bandit

# Tests de sécurité complets
ruff check . && black --check . && mypy src/ && bandit -r src/ && pip-audit
```

### CORS et limites (prod)
- **Dev** : CORS permissif (`*`)
- **Prod** : CORS restrictif (domaines spécifiques)
- **Limites** : 1MB max par requête, 100 req/min, timeouts 30s
- **Headers sécurité** : X-Content-Type-Options, X-Frame-Options, etc.

### Démarrage production
```bash
# Mode production avec sécurité renforcée
BBIA_ENV=prod BBIA_TOKEN=votre-token-secure uvicorn src.bbia_sim.daemon.app.main:app --port 8000 --host 0.0.0.0
```

## Tests & CI

### Tests unitaires
```bash
pytest tests/ -v
```

### Tests avec couverture
```bash
pytest tests/ --cov=src/bbia_sim --cov-report=html
```

### CI complète
```bash
ruff check . && black --check . && mypy src/ && bandit -r src/ && pip-audit
```

### Exemples automatisés
```bash
python examples/hello_sim.py --duration 1
python examples/goto_pose.py --token bbia-secret-key-dev --joint neck_yaw --pos 0.1
python examples/subscribe_telemetry.py --token bbia-secret-key-dev --count 3
```

## Troubleshooting

### MuJoCo
- **Erreur "No module named 'mujoco'"** : `pip install mujoco`
- **Modèle introuvable** : Vérifier `src/bbia_sim/sim/models/reachy_mini.xml`

### uvicorn
- **Port 8000 occupé** : `pkill -f "uvicorn.*8000"` ou changer le port
- **Erreur CORS** : Vérifier `BBIA_ENV` et `BBIA_CORS_ORIGINS`

### API
- **401 Unauthorized** : Vérifier le token Bearer
- **WebSocket timeout** : Vérifier que la télémétrie est activée

### Audio & Voix
- **Erreur "No module named 'pyaudio'"** :
  - Installe portaudio (`brew install portaudio`)
  - Puis `pip install pyaudio`
- **Aucune voix Amélie trouvée** :
  - Va dans Préférences Système > Accessibilité > Parole > Voix du système > Personnaliser…
  - Coche "Amélie (français — Canada)" et télécharge-la
  - Relance le script

### Général
- **API ne démarre pas** :
  - Vérifie que le port 8000 est libre
  - Lance avec `python scripts/start_api.py`
  - Vérifie les logs dans `logs/bbia.log`
- **WebSocket ne se connecte pas** :
  - Assure-toi que l'API est démarrée
  - Teste avec `python scripts/test_api.py`

## Documentation
- [Documentation complète](docs/README.md)
- [Guide simulation MuJoCo](docs/simulations/MUJOCO_SIMULATION_GUIDE.md)
- [Tests automatisés](tests/README.md)
- [Guides d'utilisation avancée](docs/guides/)

## Roadmap
- [x] Simulation MuJoCo intégrée
- [x] API REST/WebSocket fonctionnelle
- [x] CLI unifié (`--sim`, `--awake`)
- [x] Tests automatisés et CI/CD
- [ ] Simulation visuelle avancée
- [ ] Intégration ROS
- [ ] Contrôle temps réel via Web
- [ ] IA émotionnelle enrichie
- [ ] Documentation vidéo

## Contribuer
Les contributions sont les bienvenues ! Consultez [CONTRIBUTING.md](CONTRIBUTING.md) et ouvrez une issue ou une pull request.

Contact : [arkalia.luna.system@gmail.com](mailto:arkalia.luna.system@gmail.com)

## Licence
MIT

---

## 🛠️ Conseils pratiques pour fiabiliser et améliorer BBIA

- Lance les tests régulièrement (`python3 -m unittest discover tests`)
- Sauvegarde ton code avec git et des backups
- Documente chaque nouveauté ou bug
- Ajoute un arrêt d’urgence logiciel/matériel
- Prépare la calibration et la configuration
- Loggue toutes les erreurs et actions critiques
- Range bien tes fichiers et nettoie régulièrement
- Prépare un plan de test pour le vrai robot
- Note et partage tes découvertes
- Demande de l’aide à la communauté si besoin

---
