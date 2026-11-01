# Anticipation issues & GHSA Pollen — mitigations BBIA

**Date** : 1 Nov 2026  
**Lieu** : dépôt `bbia-reachy-sim` uniquement (pas de patch firmware / daemon Pollen)  
**SDK BBIA** : extra `reachy-mini==1.13.0` (≥ correctifs PIN documentés)  
**Sources** : [issues ouvertes reachy_mini](https://github.com/pollen-robotics/reachy_mini/issues) + GHSA listés ci-dessous

> BBIA **n’a pas corrigé** le firmware ni le daemon Pollen. Ce document classe l’impact
> et décrit les **gardes / docs / tests** côté BBIA pour ne pas reproduire les bugs
> ou pour les contourner quand la surface est exposée.

## Légende classement

| Classe | Signification |
|--------|----------------|
| **N/A BBIA** | Surface absente ou hors stack BBIA |
| **wrapper/guard** | Garde code / API BBIA implémentée ou renforcée |
| **doc seule** | Contournement / précaution documentée |

## GHSA (chaîne compromission BT + upload)

| ID | Sujet | Classe | Mitigation BBIA | Statut |
|----|--------|--------|-----------------|--------|
| [GHSA-vg3x-4hxm-3gxm](https://github.com/pollen-robotics/reachy_mini/security/advisories/GHSA-vg3x-4hxm-3gxm) | Path traversal `CMD_` Bluetooth | N/A BBIA | Aucun service Bluetooth / `CMD_` dans BBIA ; HF paths via `HFAppInstaller._safe_app_path` | documenté |
| [GHSA-m2pc-3q4q-w6jr](https://github.com/pollen-robotics/reachy_mini/security/advisories/GHSA-m2pc-3q4q-w6jr) | Upload dangereux `/api/media/sounds/upload` | wrapper/guard | Pas d’endpoint upload sounds ; module `safe_media_upload.py` (allowlist + magic + confinement) | implémenté |
| [GHSA-993g-hgjh-whmf](https://github.com/pollen-robotics/reachy_mini/security/advisories/GHSA-993g-hgjh-whmf) | PIN BT race (`connected` global) | doc seule | BBIA n’implémente pas le PIN BT ; **exiger SDK `reachy-mini >= 1.12.0`** (BBIA pinne **1.13.0**) | documenté |

Correctif PIN upstream : **1.12.0** (advisory). Upload / traversal : mettre à jour le daemon robot dès qu’une release Pollen les intègre ; BBIA ne les reproduit pas.

## Issues prioritaires (liste citée)

| Issue | Sujet | Classe | Mitigation BBIA | Statut |
|-------|--------|--------|-----------------|--------|
| [#1527](https://github.com/pollen-robotics/reachy_mini/issues/1527) | `GET /state/full?with_target_head_pose` → 500 si target absent | wrapper/guard | `get_full_state` : lectures optionnelles en `_safe_call` ; `as_any_pose` tolérant | implémenté |
| [#1525](https://github.com/pollen-robotics/reachy_mini/issues/1525) | GStreamer audiomixer segfault | N/A BBIA | Pas de GStreamer pour le flux principal ; `BBIA_DISABLE_AUDIO=1` en CI | documenté |
| [#1523](https://github.com/pollen-robotics/reachy_mini/issues/1523) | AppManager stop bloqué « stopping » | wrapper/guard | `stop-current-app` : `finally` libère toujours `current_app` → `stopped` | implémenté |
| [#1521](https://github.com/pollen-robotics/reachy_mini/issues/1521) | ALSA volume / `amixer` via shell | N/A BBIA | Volume Pydantic `[0,1]` ; pas de shell ALSA | documenté |
| [#1518](https://github.com/pollen-robotics/reachy_mini/issues/1518) | « Prepare for the surprise » | doc seule | Suivi communautaire ; pas de surface BBIA | documenté |
| [#1517](https://github.com/pollen-robotics/reachy_mini/issues/1517) | wobbler `AttributeError` si désactivé mid-playback | wrapper/guard | `enable/disable_wobbling` avalent AttributeError/TypeError | implémenté |
| [#1516](https://github.com/pollen-robotics/reachy_mini/issues/1516) / [#1514](https://github.com/pollen-robotics/reachy_mini/issues/1514) | Erreurs JSON-RPC / data-channel sans nom de commande | doc seule | Bridge BBIA loggue la commande ; pas de relay WebRTC Pollen | documenté |
| [#1512](https://github.com/pollen-robotics/reachy_mini/issues/1512) | Move « finished » alors qu’un autre tourne | wrapper/guard | `create_move_task` refuse le concurrent (HTTP 409) sauf `BBIA_ALLOW_CONCURRENT_MOVES=1` | implémenté |
| [#1508](https://github.com/pollen-robotics/reachy_mini/issues/1508) | WebRTC audio queue non flushée | N/A BBIA | Pas de `stop_recording` WebRTC Pollen dans BBIA | documenté |
| [#1507](https://github.com/pollen-robotics/reachy_mini/issues/1507) | USB audio « No Reachy Mini Audio » | doc seule | Premier jour : `media_backend="no_media"` ; `BBIA_DISABLE_AUDIO` | documenté |
| [#1494](https://github.com/pollen-robotics/reachy_mini/issues/1494) | WS keepalive non configurable | wrapper/guard | `BBIA_WS_KEEPALIVE_TIMEOUT` / `settings.ws_keepalive_timeout` (défaut 30 s) | implémenté |
| [#1493](https://github.com/pollen-robotics/reachy_mini/issues/1493) | Pin exact `onnxruntime==1.27.0` | N/A BBIA | BBIA ne pinne pas onnxruntime en cœur ; SDK via extra optionnel | documenté |
| [#1479](https://github.com/pollen-robotics/reachy_mini/issues/1479) | Oscillation yaw ~3 Hz | doc seule | Amplitudes BBIA bornées (`safe_amplitude_limit`) ; gain servo = firmware | documenté |
| [#1478](https://github.com/pollen-robotics/reachy_mini/issues/1478) | Sleep droop après coupe couple | doc seule | Observer sur robot ; pas de re-torque agressif BBIA au sleep | documenté |
| [#1466](https://github.com/pollen-robotics/reachy_mini/issues/1466) / [#1465](https://github.com/pollen-robotics/reachy_mini/issues/1465) | Wireless onboarding / relay après auth | doc seule | Zenoh : `REACHY_ZENOH_CONNECT` / `BBIA_ZENOH_CONNECT` ; reboot robot si relay HS | documenté |
| [#1462](https://github.com/pollen-robotics/reachy_mini/issues/1462) / [#1459](https://github.com/pollen-robotics/reachy_mini/issues/1459) | Audio barge-in / WebRTC phonèmes | N/A BBIA | Stack audio BBIA distincte (sounddevice / SDK media) | documenté |
| [#1453](https://github.com/pollen-robotics/reachy_mini/issues/1453) / [#1451](https://github.com/pollen-robotics/reachy_mini/issues/1451) / [#1450](https://github.com/pollen-robotics/reachy_mini/issues/1450) | Head tracking (freeze / sim / intrinsics) | doc seule | Tracking via SDK si dispo ; sim MuJoCo BBIA non dépendante des loops Pollen | documenté |

## Variables d’environnement utiles

```bash
export BBIA_ALLOW_HARDWARE=1          # robot physique
export BBIA_DISABLE_AUDIO=1           # CI / éviter audio USB / GStreamer host
export BBIA_ALLOW_CONCURRENT_MOVES=1  # désactive la garde #1512 (déconseillé)
export BBIA_WS_KEEPALIVE_TIMEOUT=60   # secondes (défaut 30)
export REACHY_ZENOH_CONNECT="tcp://<IP_ROBOT>:7447"
```

## Fichiers code / tests

| Fichier | Rôle |
|---------|------|
| `src/bbia_sim/daemon/app/routers/state.py` | Garde #1527 |
| `src/bbia_sim/daemon/app/routers/move.py` | Garde #1512 |
| `src/bbia_sim/daemon/app/routers/apps.py` | Garde #1523 |
| `src/bbia_sim/daemon/app/safe_media_upload.py` | Garde GHSA upload |
| `src/bbia_sim/daemon/app/hf_app_installer.py` | Path sanitize (HF) |
| `src/bbia_sim/daemon/config.py` | `ws_keepalive_timeout` |
| `src/bbia_sim/backends/reachy_mini_backend.py` | Garde wobbler #1517 |
| `src/bbia_sim/test_*_regressions.py` / `test_pollen_anticipation_*` | Preuves |

## Voir aussi

- [ISSUES_POLLEN_IMPACT_BBIA.md](ISSUES_POLLEN_IMPACT_BBIA.md) (historique Zenoh / caméra)
- [SECURITE_ROBOT.md](SECURITE_ROBOT.md)
- [PREMIER_JOUR_ROBOT.md](PREMIER_JOUR_ROBOT.md)
