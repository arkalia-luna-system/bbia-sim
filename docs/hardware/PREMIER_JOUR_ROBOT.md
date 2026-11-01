# Premier jour robot physique — procédure sûre BBIA

> **Objectif** : brancher un Reachy Mini réel **sans mouvement accidentel** au démarrage.
> Aligné SDK officiel Pollen `reachy_mini` **1.13.x**.

## Règle d’or

Sans `BBIA_ALLOW_HARDWARE=1`, BBIA **ne crée pas** de client `ReachyMini()` vers un daemon
matériel : mode simulation / MuJoCo uniquement.

Le SDK officiel se connecte dès `ReachyMini(...)` à un daemon existant ;
`use_sim` ne s’applique pleinement qu’avec `spawn_daemon=True`. C’est pourquoi BBIA
ajoute ce garde en plus de `use_sim`.

## Checklist avant tout mouvement

1. Espace libre autour de la tête / antennes ; pas de câbles en tension.
2. Daemon Pollen joignable (`reachy-mini-daemon` / robot Wireless allumé).
3. Arrêt d’urgence matériel + connaissance de `emergency_stop()`.
4. Tests BBIA en sim OK (`pytest -m "not hardware and not e2e and not slow"`).
5. Lire aussi [`SECURITE_ROBOT.md`](SECURITE_ROBOT.md).

## Activation explicite

```bash
export BBIA_ALLOW_HARDWARE=1
# Optionnel : télémétrie SDK réelle (exige aussi le flag ci-dessus)
export BBIA_TELEMETRY_SDK=1
```

Sans ces variables : démarrage daemon BBIA, bridge Zenoh, factory `auto` et
`ReachyMiniBackend` restent en **simulation**.

## Premier contact (mouvements minimaux)

```bash
# Dry-run documenté (refuse sans BBIA_ALLOW_HARDWARE)
export BBIA_ALLOW_HARDWARE=1
python scripts/hardware_dry_run_reachy_mini.py --hardware --duration 10 --output artifacts

# Backend Python explicite
python - <<'PY'
from bbia_sim.robot_factory import RobotFactory
robot = RobotFactory.create_backend(
    "reachy_mini",
    use_sim=False,
    media_backend="no_media",  # pas de caméra/audio au 1er contact
    connection_mode="localhost_only",
)
assert robot is not None
robot.connect()
# Puis seulement : wake_up / goto_target à faible amplitude
robot.disconnect()
PY
```

## Interdits le premier jour

- Lancer des `examples/reachy_mini/*` « fix moteur » sans relecture.
- `backend_type=auto` en prod sans comprendre le flag (auto = MuJoCo tant que
  `BBIA_ALLOW_HARDWARE` est absent).
- `ReachyMini()` sans arguments dans du code custom.
- Amplitudes antennes hors ±0,3 rad (limite de sécurité BBIA).
- Lancer l’audio USB / GStreamer host si le module XMOS est instable
  (`BBIA_DISABLE_AUDIO=1` ou `media_backend="no_media"`) — issues Pollen #1507 / #1525.

## Anticipation bugs daemon (aperçu)

BBIA refuse par défaut un second move concurrent (Pollen #1512), tolère
`state/full` sans target (Pollen #1527), et expose un keepalive WS
configurable (`BBIA_WS_KEEPALIVE_TIMEOUT`, Pollen #1494). Détail :
[`POLLEN_ISSUES_ANTICIPATION.md`](POLLEN_ISSUES_ANTICIPATION.md).

## Références

- SDK GitHub : https://github.com/pollen-robotics/reachy_mini (tag **v1.13.0** ; **≥ 1.12.0** pour PIN BT)
- Clone de référence local (lecture seule) : `/Users/athalia/Documents/dev/reachy_mini` @ `v1.13.0`
- Garde code : `src/bbia_sim/hardware_guard.py`
- Sécurité DO/DON’T : [`SECURITE_ROBOT.md`](SECURITE_ROBOT.md)
- Issues/GHSA : [`POLLEN_ISSUES_ANTICIPATION.md`](POLLEN_ISSUES_ANTICIPATION.md)

## BBIA vs SDK Pollen (honnête)

| Domaine | Qui mène | Note |
|---|---|---|
| Firmware / daemon / moteurs | **Pollen** | BBIA s’appuie dessus |
| Couche cognitive BBIA | **BBIA** | Au-dessus du SDK |
| Garde hardware | BBIA (`BBIA_ALLOW_HARDWARE`) | Sans flag → sim forcée |

**Dernière mise à jour** : anticipation issues/GHSA Pollen + SDK 1.13
