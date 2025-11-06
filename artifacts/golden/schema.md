# 📊 Golden Traces Schema

## Format JSONL

Chaque ligne = JSON valide avec les champs suivants :

```json
{
  "t": 0.123,
  "joint": "yaw_body", 
  "qpos": 0.156,
  "meta": {
    "seed": 42,
    "backend": "mujoco"
  }
}
```

## Champs

- **`t`** (float) : Temps écoulé depuis le début (secondes)
- **`joint`** (string) : Nom du joint animé
- **`qpos`** (float) : Position du joint en radians
- **`meta.seed`** (int) : Seed utilisé pour la génération
- **`meta.backend`** (string) : Backend utilisé ("mujoco" ou "reachy")

## Exemples de traces

### Happy (émotion joyeuse)

- Durée : 6s
- Pattern : Sinusoïde 0.5 Hz, amplitude 0.3 rad
- Émotion : "happy"

### Look-at (regard latéral)

- Durée : 6s  
- Pattern : Sinusoïde 0.5 Hz, amplitude 0.3 rad
- Émotion : "neutral"

### Wake-up (réveil)

- Durée : 6s
- Pattern : Sinusoïde 0.5 Hz, amplitude 0.3 rad
- Émotion : "excited"

## Tolérances de validation

- **Position (qpos)** : ±0.05 rad
- **Cadence** : ±15% de la référence
- **Durée minimale** : 10 échantillons
