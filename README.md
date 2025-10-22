# BBIA Reachy Mini Wireless – Simulation Python

![Bannière Reachy Mini](assets/images/Capture d’écran 2025-07-15 à 04.09.24.png)

[![Licence MIT](https://img.shields.io/badge/Licence-MIT-blue.svg)](./LICENCE)
[![Python](https://img.shields.io/badge/python-3.9%2B-blue)](https://www.python.org/)
[![Tests](https://img.shields.io/badge/tests-automatisés-green)](./tests/README.md)
[![Couverture](https://codecov.io/gh/arkalia-luna-system/bbia-sim/branch/main/graph/badge.svg)](https://codecov.io/gh/arkalia-luna-system/bbia-sim)

---

## Table des matières
- [BBIA Reachy Mini Wireless – Simulation Python](#bbia-reachy-mini-wireless--simulation-python)
  - [Table des matières](#table-des-matières)
  - [Présentation](#présentation)
  - [Capture d'écran](#capture-décran)
  - [Structure du projet](#structure-du-projet)
  - [Installation rapide](#installation-rapide)
  - [Quickstart](#quickstart)
  - [Exemples d'utilisation](#exemples-dutilisation)
  - [Audio \& Voix sur macOS](#audio--voix-sur-macos)
  - [Lancer les tests](#lancer-les-tests)
  - [Dépannage](#dépannage)
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

## Installation rapide
```bash
git clone https://github.com/arkalia-luna-system/bbia-sim.git
cd bbia-sim
python3 -m venv venv
source venv/bin/activate
pip install -e ".[dev]"
```

## Quickstart
```bash
# Simulation MuJoCo (mode graphique)
python -m bbia_sim --sim

# Simulation MuJoCo (mode headless)
python -m bbia_sim --sim --headless --duration 10

# Séquence de réveil BBIA
python -m bbia_sim --awake

# API REST/WebSocket
python scripts/start_api.py

# Tests de l'API
python scripts/test_api.py

# Synthèse vocale (voix Amélie fr_CA)
python src/bbia_sim/bbia_voice.py

# Enregistrement et lecture audio
python src/bbia_sim/bbia_audio.py
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
- [Guide installation/dépannage audio/voix](#dépannage)

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

## Dépannage
- **Erreur "No module named 'pyaudio'"** :
  - Installe portaudio (`brew install portaudio`)
  - Puis `pip install pyaudio`
- **Aucune voix Amélie trouvée** :
  - Va dans Préférences Système > Accessibilité > Parole > Voix du système > Personnaliser…
  - Coche "Amélie (français — Canada)" et télécharge-la
  - Relance le script
- **Erreur MuJoCo "Modèle introuvable"** :
  - Vérifie que `src/bbia_sim/sim/models/reachy_mini.xml` existe
  - Lance avec `python -m bbia_sim --sim --verbose`
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
