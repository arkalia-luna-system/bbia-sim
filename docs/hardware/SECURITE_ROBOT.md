# Sécurité Robot - Guide Complet

> Voir aussi : [`PREMIER_JOUR_ROBOT.md`](PREMIER_JOUR_ROBOT.md) (procédure jour J),
> [`docs/reference/INDEX_THEMATIQUE.md`](../reference/INDEX_THEMATIQUE.md)

---

## 🔒 Garde BBIA (obligatoire)

| Variable | Effet |
|----------|--------|
| `BBIA_ALLOW_HARDWARE=1` | Autorise connexion daemon / robot physique |
| absent / autre | `use_sim` forcé, pas de `ReachyMini()` matériel |
| `BBIA_WS_KEEPALIVE_TIMEOUT` | Timeout keepalive WS daemon (s, défaut 30) — #1494 |
| `BBIA_SDK_CONNECT_TIMEOUT` / `BBIA_SDK_RECONNECT_*` | Client SDK → robot (timeout + retries) |
| `BBIA_DISABLE_AUDIO=1` | Force `no_media` / coupe audio host (#1507/#1525) |
| `BBIA_HEAD_TRACKING_SIM=0` | Défaut sûr : skip tracking en sim (#1453+) |
| `BBIA_SLEEP_HOLD_TORQUE=0` | Soft only ; droop sleep reste exposé firmware (#1478) |
| `BBIA_YAW_SAFE_AMPLITUDE` | Clamp soft yaw_body (rad) — #1479 |

Sans `BBIA_ALLOW_HARDWARE` : daemon BBIA, bridge Zenoh, factory `auto` et backends restent en sim.
Détail : [`POLLEN_ISSUES_ANTICIPATION.md`](POLLEN_ISSUES_ANTICIPATION.md).

---

## ✅ DO (À Faire)

- ✅ Utiliser `yaw_body` pour animations visibles
- ✅ Respecter `safe_amplitude_limit` (±0.3 rad) et mapping joints
- ✅ Tester en simulation avant robot réel
- ✅ Prévoir `emergency_stop()` accessible (logiciel + matériel)
- ✅ Premier contact avec `media_backend="no_media"`
- ✅ Suivre [`PREMIER_JOUR_ROBOT.md`](PREMIER_JOUR_ROBOT.md)

---

## ❌ DON'T (À Éviter)

- ❌ Ne pas dépasser les limites des antennes (-0.3 à 0.3 rad) — protection hardware
- ❌ Ne jamais animer joints `passive_*` (bloqués)
- ❌ Ne pas dépasser les limites matérielles
- ❌ Ne pas désactiver watchdog en production
- ❌ Ne pas appeler `ReachyMini()` sans arguments ni sans `BBIA_ALLOW_HARDWARE`
- ❌ Ne pas compter sur `backend=auto` pour « trouver » le robot sans le flag
- ❌ Ne pas réintroduire un upload « sounds » sans `safe_media_upload` (GHSA-m2pc-3q4q-w6jr)
- ❌ Ne pas exécuter de commandes Bluetooth brutes `CMD_` / PIN (GHSA-vg3x / GHSA-993g) — hors BBIA ; laisser le daemon robot ≥ 1.12.0

---

## ✅ Checklist Avant Robot Réel

- [ ] `BBIA_ALLOW_HARDWARE=1` exporté consciemment
- [ ] Connexion stable (même SSID, pas de réseau invité)
- [ ] `BBIA_DISABLE_AUDIO` ajusté selon contexte
- [ ] Dry-run : `python scripts/hardware_dry_run_reachy_mini.py --hardware ...`
- [ ] `emergency_stop()` testé et accessible
- [ ] Amplitudes validées (< 0.3 rad)
- [ ] Daemon / image robot avec `reachy-mini` **≥ 1.12.0** (PIN BT) — BBIA pinne 1.13.0 côté host

---

## 📚 Références

- **Premier jour** : [`PREMIER_JOUR_ROBOT.md`](PREMIER_JOUR_ROBOT.md)
- **Anticipation bugs/GHSA Pollen** : [`POLLEN_ISSUES_ANTICIPATION.md`](POLLEN_ISSUES_ANTICIPATION.md)
- **Garde code** : [`src/bbia_sim/hardware_guard.py`](../../src/bbia_sim/hardware_guard.py)
- **Upload sûr** : [`src/bbia_sim/daemon/app/safe_media_upload.py`](../../src/bbia_sim/daemon/app/safe_media_upload.py)
- **Mapping joints** : [`src/bbia_sim/mapping_reachy.py`](../../src/bbia_sim/mapping_reachy.py)
- **Tests sécurité** : [`tests/test_safety_limits_pid.py`](../../tests/test_safety_limits_pid.py)

---

**Dernière mise à jour** : gardes MIXTE (SDK reconnect, audio, tracking, sleep/yaw) + SDK 1.13
