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
