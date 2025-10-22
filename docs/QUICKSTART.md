# Quickstart BBIA-SIM (≤10 min)

## Prérequis
- Python 3.9+
- Git

## Installation (2 min)

```bash
# 1. Cloner le projet
git clone https://github.com/arkalia-luna-system/bbia-sim.git
cd bbia-sim

# 2. Environnement virtuel
python3 -m venv venv
source venv/bin/activate

# 3. Installation dépendances
pip install -e ".[dev]"
```

**Résultat attendu** : Installation réussie, pas d'erreurs

## Test simulation MuJoCo (1 min)

```bash
# Test simulation headless
python -m bbia_sim --sim --headless --duration 1
```

**Résultat attendu** :
```
🚀 Lancement simulation MuJoCo headless (1s)
📊 Démarrage simulation...
✅ Simulation terminée
```

## Mode 3D visible (optionnel)

### Linux
```bash
# Installation viewer
pip install mujoco-python-viewer

# Lancement avec fenêtre 3D
python -m bbia_sim --sim --verbose
```

### macOS
```bash
# Installation viewer
pip install mujoco-python-viewer

# Lancement avec fenêtre 3D (nécessite mjpython)
mjpython -m bbia_sim --sim --verbose
```

**Résultat attendu** : Fenêtre MuJoCo avec robot Reachy Mini visible et animé

## Démarrage API (2 min)

```bash
# Terminal 1 : Démarrage API
BBIA_ENV=prod BBIA_TOKEN=dev uvicorn src.bbia_sim.daemon.app.main:app --port 8000 --timeout-keep-alive 5
```

**Résultat attendu** :
```
INFO:     Started server process
INFO:     Waiting for application startup.
✅ Simulation MuJoCo démarrée avec succès
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

## Test API REST (2 min)

```bash
# Terminal 2 : Test API
curl -H "Authorization: Bearer dev" http://localhost:8000/api/state/full
```

**Résultat attendu** :
```json
{
  "position": {"x": 0.0, "y": 0.0, "z": 0.0},
  "status": "ready",
  "battery": 85.5,
  "temperature": 25.5,
  "timestamp": "2025-01-01T12:00:00"
}
```

## Test mouvement robot (2 min)

```bash
# Contrôle articulation
python examples/goto_pose.py --token dev --joint neck_yaw --pos 0.6
```

**Résultat attendu** :
```
🎯 Contrôle articulation: neck_yaw → 0.6 rad
📊 État initial...
   Position initiale: 0.0
🚀 Envoi commande mouvement...
   Statut: moving
⏳ Attente exécution...
✅ Position finale: 0.6
📈 Succès: 1/1
```

## Test télémétrie WebSocket (1 min)

```bash
# Télémétrie temps réel
python examples/subscribe_telemetry.py --token dev --count 5
```

**Résultat attendu** :
```
📡 Connexion WebSocket: ws://localhost:8000/ws/telemetry
📊 Réception de 5 messages de télémétrie...
✅ Connexion établie

📨 Message 1/5:
   Robot: x=0.050, y=0.030
   Articulations: 7 joints
   Batterie: 87.2%
```

## Vérification complète (1 min)

```bash
# Tests automatisés
python examples/hello_sim.py --duration 1
python examples/goto_pose.py --token dev --joint neck_yaw --pos 0.1
python examples/subscribe_telemetry.py --token dev --count 3
```

**Résultat attendu** : Tous les scripts s'exécutent sans erreur

## Arrêt

```bash
# Arrêter l'API (Ctrl+C dans le terminal)
# Ou tuer le processus
pkill -f "uvicorn.*8000"
```

## Prochaines étapes

1. **Explorer l'API** : `http://localhost:8000/docs`
2. **Modifier le robot** : Éditer `src/bbia_sim/sim/models/reachy_mini.xml`
3. **Ajouter des scènes** : Créer dans `src/bbia_sim/sim/scenes/`
4. **Intégrer ROS** : Utiliser l'API REST comme bridge

## Troubleshooting rapide

- **Port 8000 occupé** : `pkill -f "uvicorn.*8000"`
- **Token invalide** : Utiliser `dev` ou vérifier `BBIA_TOKEN`
- **MuJoCo erreur** : `pip install mujoco`
- **WebSocket timeout** : Vérifier que l'API est démarrée

**Total : ≤10 minutes pour un système robotique complet !**
