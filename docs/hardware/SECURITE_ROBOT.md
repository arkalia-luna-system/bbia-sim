# Sécurité Robot - Guide Complet

> Voir aussi : [`PREMIER_JOUR_ROBOT.md`](PREMIER_JOUR_ROBOT.md) (procédure jour J),
> [`docs/reference/INDEX_THEMATIQUE.md`](../reference/INDEX_THEMATIQUE.md)

---

## 🔒 Garde BBIA (obligatoire)

| Variable | Effet |
|----------|--------|
| `BBIA_ALLOW_HARDWARE=1` | Autorise connexion daemon / robot physique |
| absent / autre | `use_sim` forcé, pas de `ReachyMini()` matériel |

Sans ce flag : daemon BBIA, bridge Zenoh, factory `auto` et backends restent en sim.

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

---

## ✅ Checklist Avant Robot Réel

- [ ] `BBIA_ALLOW_HARDWARE=1` exporté consciemment
- [ ] Connexion stable (même SSID, pas de réseau invité)
- [ ] `BBIA_DISABLE_AUDIO` ajusté selon contexte
- [ ] Dry-run : `python scripts/hardware_dry_run_reachy_mini.py --hardware ...`
- [ ] `emergency_stop()` testé et accessible
- [ ] Amplitudes validées (< 0.3 rad)

---

## 📚 Références

- **Premier jour** : [`PREMIER_JOUR_ROBOT.md`](PREMIER_JOUR_ROBOT.md)
- **Garde code** : [`src/bbia_sim/hardware_guard.py`](../../src/bbia_sim/hardware_guard.py)
- **Mapping joints** : [`src/bbia_sim/mapping_reachy.py`](../../src/bbia_sim/mapping_reachy.py)
- **Tests sécurité** : [`tests/test_safety_limits_pid.py`](../../tests/test_safety_limits_pid.py)

---

**Dernière mise à jour** : alignement SDK 1.13 / garde hardware
