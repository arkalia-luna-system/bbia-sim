# 🔒 Sécurité Robot - Guide Complet

**Date** : Oct / Nov. 2025  
**Version** : 1.0

> Voir aussi : [`docs/reference/INDEX_THEMATIQUE.md`](../reference/INDEX_THEMATIQUE.md) et [`docs/reference/project-status.md`](../reference/project-status.md)

---

## ✅ DO (À Faire)

- ✅ Utiliser `yaw_body` pour animations visibles
- ✅ Respecter `safe_amplitude_limit` (±0.3 rad) et mapping joints
- ✅ Tester en simulation avant robot réel
- ✅ Prévoir `emergency_stop()` accessible (logiciel + matériel)

---

## ❌ DON'T (À Éviter)

- ❌ Ne pas dépasser les limites des antennes (-0.3 à 0.3 rad) — protection hardware
- ❌ Ne jamais animer joints `passive_*` (bloqués)
- ❌ Ne pas dépasser les limites matérielles
- ❌ Ne pas désactiver watchdog en production

---

## ✅ Checklist Avant Robot Réel

- [ ] Connexion stable (même SSID, pas de réseau invité)
- [ ] `BBIA_DISABLE_AUDIO` ajusté selon contexte
- [ ] Dry-run 60s : `wake_up` → `look_at` → `stop` → watchdog OK
- [ ] `emergency_stop()` testé et accessible
- [ ] Amplitudes validées (< 0.3 rad)

---

## 📚 Références

- **État par axe** : [`docs/reference/project-status.md`](../reference/project-status.md) → Fonctionnalités robot
- **Mapping joints** : [`src/bbia_sim/mapping_reachy.py`](../../src/bbia_sim/mapping_reachy.py)
- **Tests sécurité** : [`tests/test_safety_limits_pid.py`](../../tests/test_safety_limits_pid.py)

---

**Dernière mise à jour** : Oct / Nov. 2025
