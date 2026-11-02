# Anticipation issues & GHSA Pollen — mitigations BBIA

**Date** : 1 Nov 2026  
**Lieu** : dépôt `bbia-reachy-sim` uniquement (pas de patch firmware / daemon Pollen)  
**SDK BBIA** : extra `reachy-mini==1.13.0` (≥ correctifs PIN documentés) + `certifi>=2026.10.19`  
**Sources** : [issues ouvertes reachy_mini](https://github.com/pollen-robotics/reachy_mini/issues) + GHSA listés ci-dessous

> BBIA **n’a pas corrigé** le firmware ni le daemon Pollen. Ce document classe l’impact
> et décrit les **gardes / docs / tests** côté BBIA pour ne pas reproduire les bugs
> ou pour les contourner quand la surface est exposée.
> **Honnêteté** : plusieurs symptômes (droop sleep, oscillation yaw servo, GStreamer
> host, relay Wireless) restent **exposés firmware / hardware Pollen**.

## Légende classement

| Classe | Signification |
|--------|----------------|
| **N/A BBIA** | Surface absente ou hors stack BBIA |
| **wrapper/guard** | Garde code / API BBIA implémentée ou renforcée |
| **doc seule** | Contournement / précaution documentée |
| **exposé Pollen** | Correctif firmware/daemon requis ; BBIA mitige seulement |

## Tableau BBIA vs Pollen (synthèse)

| Domaine | Anticipé **code BBIA** | Reste **exposé firmware / Pollen** |
|---------|------------------------|-------------------------------------|
| GHSA BT path / PIN / upload sounds | Pas de surface BT/`CMD_` ; `safe_media_upload` ; SDK ≥ 1.12 pin 1.13 | Daemon robot doit être à jour pour PIN/upload upstream |
| WS keepalive / SDK connect timeout+reconnect | `BBIA_WS_KEEPALIVE_TIMEOUT`, `BBIA_SDK_CONNECT_TIMEOUT`, `BBIA_SDK_RECONNECT_*` | Comportement keepalive daemon Pollen lui-même |
| USB audio / GStreamer / amixer | `media_guard` → `no_media` ; `BBIA_DISABLE_AUDIO` ; volume Pydantic ; refus shell | Crash GStreamer / device « No Reachy Mini Audio » sur host/robot |
| Head tracking sim / freeze / intrinsics | Skip sim (`BBIA_HEAD_TRACKING_SIM=0`) ; clamp look ; logs | Loops tracking / calib caméra daemon Pollen |
| Yaw ~3 Hz / sleep droop | `safe_amplitude` + `BBIA_YAW_SAFE_AMPLITUDE` ; watchdog ; garde `goto_sleep` + `BBIA_SLEEP_HOLD_TORQUE` best-effort | Gain servo / coupe couple firmware |
| Wireless onboarding / relay | Timeout+reconnect + heartbeat ; checklist Zenoh | Auth / relay hardware Wireless |
| `certifi` CA bundle | Pin `certifi>=2026.10.19` (alignement 1.13) | — |

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
| [#1525](https://github.com/pollen-robotics/reachy_mini/issues/1525) | GStreamer audiomixer segfault | wrapper/guard | `media_guard` refuse backends gstreamer ; `BBIA_DISABLE_AUDIO` → `no_media` | **code** (+ exposé host si forcé ailleurs) |
| [#1523](https://github.com/pollen-robotics/reachy_mini/issues/1523) | AppManager stop bloqué « stopping » | wrapper/guard | `stop-current-app` : `finally` libère toujours `current_app` → `stopped` | implémenté |
| [#1521](https://github.com/pollen-robotics/reachy_mini/issues/1521) | ALSA volume / `amixer` via shell | wrapper/guard | Volume Pydantic `[0,1]` ; `refuse_shell_audio_command` ; pas de shell ALSA | **code** |
| [#1518](https://github.com/pollen-robotics/reachy_mini/issues/1518) | « Prepare for the surprise » | doc seule | Suivi communautaire ; pas de surface BBIA | documenté |
| [#1517](https://github.com/pollen-robotics/reachy_mini/issues/1517) | wobbler `AttributeError` si désactivé mid-playback | wrapper/guard | `enable/disable_wobbling` avalent AttributeError/TypeError | implémenté |
| [#1516](https://github.com/pollen-robotics/reachy_mini/issues/1516) / [#1514](https://github.com/pollen-robotics/reachy_mini/issues/1514) | Erreurs JSON-RPC / data-channel sans nom de commande | doc seule | Bridge BBIA loggue la commande ; pas de relay WebRTC Pollen | documenté |
| [#1512](https://github.com/pollen-robotics/reachy_mini/issues/1512) | Move « finished » alors qu’un autre tourne | wrapper/guard | `create_move_task` refuse le concurrent (HTTP 409) sauf `BBIA_ALLOW_CONCURRENT_MOVES=1` | implémenté |
| [#1508](https://github.com/pollen-robotics/reachy_mini/issues/1508) | WebRTC audio queue non flushée | N/A BBIA | Pas de `stop_recording` WebRTC Pollen dans BBIA | documenté |
| [#1507](https://github.com/pollen-robotics/reachy_mini/issues/1507) | USB audio « No Reachy Mini Audio » | wrapper/guard | `looks_like_usb_reachy_audio` + `resolve_media_backend` + `BBIA_DISABLE_AUDIO` → `no_media` (logs refus explicites) | **code** (+ exposé device USB host) |
| [#1494](https://github.com/pollen-robotics/reachy_mini/issues/1494) | WS keepalive non configurable | wrapper/guard | `BBIA_WS_KEEPALIVE_TIMEOUT` (uvicorn + receive, mypy-safe) **et** `BBIA_SDK_CONNECT_TIMEOUT` / `BBIA_SDK_RECONNECT_*` + logs reconnect | **code** |
| [#1493](https://github.com/pollen-robotics/reachy_mini/issues/1493) | Pin exact `onnxruntime==1.27.0` | N/A BBIA | BBIA ne pinne pas onnxruntime en cœur ; SDK via extra optionnel | documenté |
| [#1479](https://github.com/pollen-robotics/reachy_mini/issues/1479) | Oscillation yaw ~3 Hz | wrapper/guard + exposé Pollen | `clamp_yaw_amplitude` / `BBIA_YAW_SAFE_AMPLITUDE` + watchdog ; **gain servo = firmware** | **code soft** |
| [#1478](https://github.com/pollen-robotics/reachy_mini/issues/1478) | Sleep droop après coupe couple | wrapper/guard + exposé Pollen | `prepare_goto_sleep` (re-check pose, warning) ; `BBIA_SLEEP_HOLD_TORQUE=1` best-effort `enable_motors` — **ne corrige pas le firmware** | **code soft** |
| [#1466](https://github.com/pollen-robotics/reachy_mini/issues/1466) / [#1465](https://github.com/pollen-robotics/reachy_mini/issues/1465) | Wireless onboarding / relay après auth | wrapper/guard + exposé Pollen | Timeout/reconnect SDK + `BBIA_SDK_HEARTBEAT_TIMEOUT` ; checklist Zenoh ci-dessous | **code soft** |
| [#1462](https://github.com/pollen-robotics/reachy_mini/issues/1462) / [#1459](https://github.com/pollen-robotics/reachy_mini/issues/1459) | Audio barge-in / WebRTC phonèmes | N/A BBIA | Stack audio BBIA distincte (sounddevice / SDK media) | documenté |
| [#1453](https://github.com/pollen-robotics/reachy_mini/issues/1453) / [#1451](https://github.com/pollen-robotics/reachy_mini/issues/1451) / [#1450](https://github.com/pollen-robotics/reachy_mini/issues/1450) | Head tracking (freeze / sim / intrinsics) | wrapper/guard | `BBIA_HEAD_TRACKING_SIM=0` (défaut) → skip/no-op ; clamp UV / direction | **code** |

## Checklist Wireless (#1466 / #1465) — soft BBIA

1. Même SSID ; pas de réseau invité / isolation client.
2. `export REACHY_ZENOH_CONNECT="tcp://<IP_ROBOT>:7447"` (ou `BBIA_ZENOH_CONNECT`).
3. `BBIA_SDK_CONNECT_TIMEOUT` / `BBIA_SDK_RECONNECT_ATTEMPTS` si le relay est lent.
4. Si timeout répétés après auth : **reboot robot** (relay HS = exposé Pollen).
5. Watchdog : `BBIA_SDK_HEARTBEAT_TIMEOUT` (défaut 2 s) → emergency_stop soft si plus de réponse.

## Variables d’environnement utiles

```bash
export BBIA_ALLOW_HARDWARE=1          # robot physique
export BBIA_DISABLE_AUDIO=1           # CI / éviter audio USB / GStreamer host
export BBIA_ALLOW_CONCURRENT_MOVES=1  # désactive la garde #1512 (déconseillé)
export BBIA_WS_KEEPALIVE_TIMEOUT=60   # secondes (défaut 30) — WS daemon BBIA
export BBIA_SDK_CONNECT_TIMEOUT=5     # timeout client SDK → robot
export BBIA_SDK_RECONNECT_ATTEMPTS=3  # tentatives connexion
export BBIA_SDK_RECONNECT_DELAY=0.5   # pause entre tentatives (s)
export BBIA_SDK_HEARTBEAT_TIMEOUT=2   # watchdog sans heartbeat (s)
export BBIA_HEAD_TRACKING_SIM=0       # défaut sûr : pas de tracking loops en sim
export BBIA_SLEEP_HOLD_TORQUE=0       # 1 = enable_motors best-effort avant sleep
export BBIA_YAW_SAFE_AMPLITUDE=0.3    # clamp soft yaw_body (rad)
export REACHY_ZENOH_CONNECT="tcp://<IP_ROBOT>:7447"
```

## Fichiers code / tests

| Fichier | Rôle |
|---------|------|
| `src/bbia_sim/daemon/app/routers/state.py` | Garde #1527 |
| `src/bbia_sim/daemon/app/routers/move.py` | Garde #1512 + receive keepalive #1494 |
| `src/bbia_sim/daemon/app/routers/apps.py` | Garde #1523 |
| `src/bbia_sim/daemon/app/routers/media.py` | Volume Pydantic ; `BBIA_DISABLE_AUDIO` ; pas amixer |
| `src/bbia_sim/daemon/app/safe_media_upload.py` | Garde GHSA upload |
| `src/bbia_sim/daemon/app/hf_app_installer.py` | Path sanitize (HF) |
| `src/bbia_sim/daemon/config.py` | `ws_keepalive_timeout` + `sdk_*` |
| `src/bbia_sim/daemon/ws/keepalive.py` | Câblage runtime #1494 (uvicorn + receive) |
| `src/bbia_sim/sdk_connection.py` | Timeout / reconnect / heartbeat SDK |
| `src/bbia_sim/media_guard.py` | #1507/#1525/#1521 |
| `src/bbia_sim/head_tracking_guard.py` | #1453/#1451/#1450 |
| `src/bbia_sim/sleep_guard.py` | #1478/#1479 soft |
| `src/bbia_sim/backends/reachy_mini_backend.py` | Câblage gardes + wobbler #1517 |
| `src/bbia_sim/test_pollen_anticipation_regressions.py` + suites state/move/apps/media | Preuves |

**Suite pytest ciblée (re-collect après chaque change ; `--cov=` off)** :
- `test_pollen_anticipation_*` + websocket_auth + move + media → compte local courant (voir preuves mission)
- state + apps regressions → suite courte séparée
- Firmware yaw/sleep/Wireless : **exposé Pollen** — docs honnêtes seulement ; soft BBIA ≠ correctif firmware.

## Voir aussi

- [ISSUES_POLLEN_IMPACT_BBIA.md](ISSUES_POLLEN_IMPACT_BBIA.md) (historique Zenoh / caméra)
- [SECURITE_ROBOT.md](SECURITE_ROBOT.md)
- [PREMIER_JOUR_ROBOT.md](PREMIER_JOUR_ROBOT.md)
