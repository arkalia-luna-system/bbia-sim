# Sécurité robot (DO/DON’T)

> Voir aussi: `docs/references/INDEX_THEMATIQUE.md` et `docs/status.md`

## DO
- Utiliser `yaw_body` pour animations visibles
- Respecter `safe_amplitude_limit` (±0.3 rad) et mapping joints
- Tester en simulation avant robot réel
- Prévoir `emergency_stop()` accessible (logiciel + matériel)

## DON'T
- Ne pas dépasser les limites des antennes (-0.3 à 0.3 rad) — protection hardware
- Ne jamais animer joints `passive_*` (bloqués)
- Ne pas dépasser les limites matérielles
- Ne pas désactiver watchdog en prod

## Check-list avant robot réel
- Connexion stable (même SSID, pas de réseau invité)
- BBIA_DISABLE_AUDIO ajusté selon contexte
- Dry-run 60s: wake_up → look_at → stop → watchdog OK

## Références
- État par axe: `docs/status.md` → Fonctionnalités robot
