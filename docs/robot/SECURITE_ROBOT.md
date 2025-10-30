# Sécurité robot (DO/DON’T)

> Voir aussi: `docs/references/INDEX_THEMATIQUE.md` et `docs/status.md`

## DO
- Utiliser `yaw_body` pour animations visibles
- Respecter `safe_amplitude_limit` (±0.3 rad) et mapping joints
- Tester en simulation avant robot réel
- Prévoir `emergency_stop()` accessible (logiciel + matériel)

## DON’T
- Ne jamais animer `left_antenna`, `right_antenna`, joints `passive_*`
- Ne pas dépasser les limites matérielles
- Ne pas désactiver watchdog en prod

## Check-list avant robot réel
- Connexion stable (même SSID, pas de réseau invité)
- BBIA_DISABLE_AUDIO ajusté selon contexte
- Dry-run 60s: wake_up → look_at → stop → watchdog OK

## Références
- État par axe: `docs/status.md` → Fonctionnalités robot
