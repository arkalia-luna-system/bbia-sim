# Exemples BBIA-SIM

Ce dossier contient des exemples pratiques pour utiliser BBIA-SIM.

## Scripts disponibles

### `hello_sim.py` - Simulation MuJoCo simple
Lance une simulation MuJoCo headless pour tester les performances.

```bash
python examples/hello_sim.py --duration 2 --verbose
```

**Résultat attendu** : Simulation 2s, affichage steps/s

### `goto_pose.py` - Contrôle mouvement robot
Contrôle une articulation du robot via l'API REST.

```bash
python examples/goto_pose.py --token dev --joint neck_yaw --pos 0.6
```

**Résultat attendu** : Position articulation changée, confirmation API

### `subscribe_telemetry.py` - Télémétrie WebSocket
S'abonne à la télémétrie temps réel via WebSocket.

```bash
python examples/subscribe_telemetry.py --token dev --count 5
```

**Résultat attendu** : 5 messages de télémétrie affichés

### `demo_emotion_ok.py` - Démo Émotion → Pose (RobotAPI)
Démo BBIA utilisant le backend unifié RobotAPI.

```bash
# Mode headless (test)
python examples/demo_emotion_ok.py --emotion happy --duration 5 --headless --backend mujoco

# Mode graphique (voir 3D)
mjpython examples/demo_emotion_ok.py --emotion happy --duration 10 --backend mujoco
```

**Résultat attendu** : Animation émotion → joint, backend unifié

### `demo_emotion_fixed.py` - Démo 3D Stable
Démo 3D stable utilisant MuJoCo directement.

```bash
# Voir la 3D (stable)
mjpython examples/demo_emotion_fixed.py --emotion happy --duration 10 --intensity 0.8
```

**Résultat attendu** : Fenêtre 3D stable, animation fluide

### `demo_voice_ok.py` - Démo Voix → Action
Démo BBIA Voix utilisant RobotAPI.

```bash
python examples/demo_voice_ok.py --command "regarde-moi" --duration 5 --headless --backend mujoco
```

**Résultat attendu** : Commande vocale → action robot

### `demo_vision_ok.py` - Démo Vision → Tracking
Démo BBIA Vision utilisant RobotAPI.

```bash
python examples/demo_vision_ok.py --target "virtual_target" --duration 5 --headless --backend mujoco
```

**Résultat attendu** : Tracking visuel → mouvement robot

### `demo_behavior_ok.py` - Démo Comportement → Scénario
Démo BBIA Comportement utilisant RobotAPI.

```bash
python examples/demo_behavior_ok.py --behavior "wake_up" --duration 5 --headless --backend mujoco
```

**Résultat attendu** : Comportement complexe → séquence d'actions

## 🎯 **Backend Unifié**

Toutes les démos supportent le backend unifié :

```bash
# Simulation MuJoCo
--backend mujoco

# Robot réel (mock)
--backend reachy
```

## 📊 **Métriques**

- **Tests** : 441 tests passent (79% réussite)
- **Coverage** : 68.86%
- **Performance** : <5s par test smoke
- **Golden Tests** : 3 traces référence + validation

## Prérequis

1. **API démarrée** :
   ```bash
   BBIA_ENV=prod BBIA_TOKEN=dev uvicorn src.bbia_sim.daemon.app.main:app --port 8000
   ```

2. **Dépendances installées** :
   ```bash
   pip install httpx websockets
   ```

## Codes de sortie

- `0` : Succès
- `1` : Erreur (détails dans la sortie)

## Troubleshooting

- **Erreur connexion API** : Vérifier que l'API est démarrée sur le bon port
- **Token invalide** : Utiliser le même token que l'API (`dev` par défaut)
- **Timeout WebSocket** : Vérifier que la télémétrie est activée
