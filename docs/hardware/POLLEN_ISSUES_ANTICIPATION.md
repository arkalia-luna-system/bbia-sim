# Anticipation issues & GHSA Pollen — mitigations BBIA

**Date** : 4 Nov 2026 (échantillon `gh issue list` open Pollen)  
**Lieu** : dépôt `bbia-reachy-sim` uniquement (pas de patch firmware / daemon Pollen)  
**SDK BBIA** : extra `reachy-mini==1.13.0` (≥ correctifs PIN documentés) + `certifi>=2026.10.19`  
**Ref locale** : `/Users/athalia/Documents/dev/reachy_mini` @ **v1.13.0** (lecture RO)  
**Sources** : [issues ouvertes reachy_mini](https://github.com/pollen-robotics/reachy_mini/issues) + GHSA listés ci-dessous

> BBIA **n’a pas corrigé** le firmware ni le daemon Pollen. Ce document classe l’impact
> et décrit les **gardes / docs / tests** côté BBIA pour ne pas reproduire les bugs
> ou pour les contourner quand la surface est exposée.
> **Score domaine « hardware Pollen »** : **≤7/10** volontaire (correctifs firmware hors BBIA).
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
| WS keepalive / SDK connect timeout+reconnect | `BBIA_WS_KEEPALIVE_TIMEOUT`, `BBIA_SDK_CONNECT_TIMEOUT`, `BBIA_SDK_RECONNECT_*` ; auth WS prod via `hmac.compare_digest` (`daemon/ws/auth.py`) ; cadence télémétrie = `BBIA_TELEMETRY_FREQUENCY` | Comportement keepalive daemon Pollen lui-même |
| USB audio / GStreamer / amixer | `media_guard` → `no_media` ; `BBIA_DISABLE_AUDIO` ; volume Pydantic ; refus shell | Crash GStreamer / device « No Reachy Mini Audio » sur host/robot |
| Head tracking sim / freeze / intrinsics | Skip sim (`BBIA_HEAD_TRACKING_SIM=0`) ; clamp look ; logs | Loops tracking / calib caméra daemon Pollen |
| Head pitch/roll ±40° (#1449) | `clamp_head_pitch_roll` + `clamp_head_pose_matrix` (4×4) + `_create_cached_head_pose` / `set_target*` / `goto_target` | IK / NaN daemon Pollen si client bypasse BBIA |
| `set_target` post-`enable_motors` (#1430) | `motors_settle_guard` (`BBIA_MOTORS_SETTLE_S`) | Crash daemon si client bypasse BBIA |
| Chaining moves tête basse → jump (#1417) | `play_move_guard` (`BBIA_PLAY_MOVE_INITIAL_GOTO_S`, défaut 1.0) | Gel tête firmware si client bypasse BBIA / force `0.0` |
| Yaw ~3 Hz / sleep droop | `safe_amplitude` + `BBIA_YAW_SAFE_AMPLITUDE` ; watchdog ; garde `goto_sleep` + `BBIA_SLEEP_HOLD_TORQUE` best-effort | Gain servo / coupe couple firmware |
| Wireless onboarding / relay | Timeout+reconnect + heartbeat ; checklist Zenoh | Auth / relay hardware Wireless |
| `certifi` CA bundle | Pin `certifi>=2026.10.19` (alignement 1.13) | — |
| Dep hygiene Pollen (#1551/#1549) | Suivi ; BBIA garde floors `asgiref`/`log-throttling` tant que SDK 1.13 les tire | Drop upstream dans une future release SDK |

## GHSA (chaîne compromission BT + upload)

| ID | Sujet | Classe | Mitigation BBIA | Statut |
|----|--------|--------|-----------------|--------|
| [GHSA-vg3x-4hxm-3gxm](https://github.com/pollen-robotics/reachy_mini/security/advisories/GHSA-vg3x-4hxm-3gxm) | Path traversal `CMD_` Bluetooth | N/A BBIA | Aucun service Bluetooth / `CMD_` dans BBIA ; HF paths via `HFAppInstaller._safe_app_path` | documenté |
| [GHSA-m2pc-3q4q-w6jr](https://github.com/pollen-robotics/reachy_mini/security/advisories/GHSA-m2pc-3q4q-w6jr) | Upload dangereux `/api/media/sounds/upload` | wrapper/guard | Pas d’endpoint upload sounds ; module `safe_media_upload.py` (allowlist + magic + confinement) | implémenté |
| [GHSA-993g-hgjh-whmf](https://github.com/pollen-robotics/reachy_mini/security/advisories/GHSA-993g-hgjh-whmf) | PIN BT race (`connected` global) | doc seule | BBIA n’implémente pas le PIN BT ; **exiger SDK `reachy-mini >= 1.12.0`** (BBIA pinne **1.13.0**) | documenté |

Correctif PIN upstream : **1.12.0** (advisory). Upload / traversal : mettre à jour le daemon robot dès qu’une release Pollen les intègre ; BBIA ne les reproduit pas.

## Issues prioritaires (liste citée + échantillon open 4 nov. 2026)

| Issue | Sujet | Classe | Mitigation BBIA | Statut |
|-------|--------|--------|-----------------|--------|
| [#1551](https://github.com/pollen-robotics/reachy_mini/issues/1551) | Drop dépendance `asgiref` | N/A BBIA / suivi | Pas de `async_to_sync` play_move Pollen dans BBIA ; floors BBIA inchangés jusqu’au drop SDK | documenté |
| [#1549](https://github.com/pollen-robotics/reachy_mini/issues/1549) | Drop `log-throttling` | N/A BBIA / suivi | BBIA ne pinne pas `==0.0.3` en cœur ; suivi release SDK | documenté |
| [#1547](https://github.com/pollen-robotics/reachy_mini/issues/1547) | Rerun viewer : pas de frames caméra remote | N/A BBIA | Pas d’exemple `rerun_viewer` / thread caméra LOCAL Pollen dans BBIA | documenté |
| [#1545](https://github.com/pollen-robotics/reachy_mini/issues/1545) | Wheel Pollen : STL non référencés (~7 MB) | N/A BBIA | Packaging wheel Pollen ; BBIA ne redistribue pas ces assets | documenté |
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
| [#1453](https://github.com/pollen-robotics/reachy_mini/issues/1453) / [#1451](https://github.com/pollen-robotics/reachy_mini/issues/1451) / [#1450](https://github.com/pollen-robotics/reachy_mini/issues/1450) | Head tracking (freeze / sim / intrinsics) | wrapper/guard | `BBIA_HEAD_TRACKING_SIM=0` (défaut) → skip/no-op ; clamp UV / direction ; câblé backend **et** `BackendAdapter` | **code** |
| [#1449](https://github.com/pollen-robotics/reachy_mini/issues/1449) | Clamp doc ±40° pitch/roll absent du SDK | wrapper/guard | `clamp_head_pitch_roll` + `clamp_head_pose_matrix` + câblage `_create_cached_head_pose` / `set_target` / `goto_target` / `set_target_head_pose` ; `BBIA_HEAD_PITCH_ROLL_LIMIT_DEG` | **code soft** |
| [#1430](https://github.com/pollen-robotics/reachy_mini/issues/1430) | Stream `set_target` juste après `enable_motors` (sleep) | wrapper/guard + exposé Pollen | `mark_motors_enabled` + `await_motors_settle_if_needed` ; préférer `goto_target` interpolé | **code soft** |
| [#1501](https://github.com/pollen-robotics/reachy_mini/issues/1501) | Chemin App connexion / rswebrtc (non localisé) | N/A BBIA | Pas de stack rswebrtc Pollen dans BBIA | documenté |
| [#1464](https://github.com/pollen-robotics/reachy_mini/issues/1464) | LED power / bouton rubber Wireless | N/A BBIA | Hardware physique | documenté |
| [#1439](https://github.com/pollen-robotics/reachy_mini/issues/1439) | Relay : backoff reconnect après chaque token change | N/A BBIA / exposé Pollen | Bug `_reconnect_now` / `_token_updated` **dans le daemon relay Pollen** — pas de surface BBIA. `BBIA_SDK_RECONNECT_DELAY` = délai fixe soft client ≠ ce backoff | documenté |
| [#1436](https://github.com/pollen-robotics/reachy_mini/issues/1436) | `play_sound` bloque ~1.1 s (Wireless WebRTC) | doc seule + exposé Pollen | Surface `play_sound` BBIA existe ; blocage = backend WebRTC Pollen — pas de timeout firmware | documenté |
| [#1435](https://github.com/pollen-robotics/reachy_mini/issues/1435) | Licence STL/URDF / redistribution GLB | N/A BBIA | Packaging / assets Pollen | documenté |
| [#1434](https://github.com/pollen-robotics/reachy_mini/issues/1434) | Caméra Wireless : exposure libcamerasrc | N/A BBIA | Pipeline caméra robot | documenté |
| [#1431](https://github.com/pollen-robotics/reachy_mini/issues/1431) | CI physique WebRTC LAN | N/A BBIA | Infra Pollen | documenté |
| [#1425](https://github.com/pollen-robotics/reachy_mini/issues/1425) | Micro Reachy Mini ne capture pas | N/A BBIA / hardware | Garantie / hardware | documenté |
| [#1417](https://github.com/pollen-robotics/reachy_mini/issues/1417) | Saut `waiting`→`yes1` : tête inert jusqu'à re-enable | wrapper/guard + exposé Pollen | `play_move_guard` : défaut `initial_goto_duration=1.0` (`BBIA_PLAY_MOVE_INITIAL_GOTO_S`) sur backend + adapter ; si gel : `disable_motors` puis `enable_motors` (+ settle #1430) | **code soft** |
| [#1416](https://github.com/pollen-robotics/reachy_mini/issues/1416) | webrtcsink deadlock 1ʳᵉ session + IPC caméra | N/A BBIA | Daemon média Pollen | documenté |
| [#1407](https://github.com/pollen-robotics/reachy_mini/issues/1407) | Jitter buffer WebRTC configurable | N/A BBIA | Pas de jitter buffer WebRTC Pollen dans BBIA | documenté |
| [#1402](https://github.com/pollen-robotics/reachy_mini/issues/1402) | Motors Input Voltage Error (0x01) batterie | exposé Pollen | Firmware / couple silencieux ; pas d'API BBIA dédiée | documenté |
| [#1400](https://github.com/pollen-robotics/reachy_mini/issues/1400) | Stop app → idle-reset sleep + `disable_motors` | N/A BBIA (surface absente) | `stop-current-app` BBIA libère le slot sans sleep/disable (pas de reproduction #1400) | documenté |
| [#1398](https://github.com/pollen-robotics/reachy_mini/issues/1398) | Batterie / charge Wireless via REST daemon | N/A BBIA | API daemon Pollen | documenté |
| [#1381](https://github.com/pollen-robotics/reachy_mini/issues/1381) | CI physique loopback speaker→mic | N/A BBIA | Infra Pollen | documenté |
| [#1375](https://github.com/pollen-robotics/reachy_mini/issues/1375) | JS-first hardware access | N/A BBIA | Hors stack Python BBIA | documenté |

> **#1449 matrices 4×4** : le clamp BBIA couvre aussi les poses homogènes passées à `set_target` / `goto_target` / `set_target_head_pose` (Euler XYZ via scipy). Un client qui parle au daemon Pollen **sans** passer par le backend BBIA reste exposé.

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
export BBIA_HEAD_PITCH_ROLL_LIMIT_DEG=40  # #1449 ; 0 = désactive
export BBIA_MOTORS_SETTLE_S=0.35      # #1430 settle post-enable ; 0 = off
export BBIA_PLAY_MOVE_INITIAL_GOTO_S=1.0  # #1417 ; 0 = jump instantané (risqué)
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
| `src/bbia_sim/daemon/app/backend_adapter.py` | #145x tracking + #1417 play_move soft |
| `src/bbia_sim/head_pose_guard.py` | #1449 pitch/roll ±40° (euler + matrices 4×4) |
| `src/bbia_sim/motors_settle_guard.py` | #1430 settle post-enable |
| `src/bbia_sim/play_move_guard.py` | #1417 `initial_goto_duration` soft |
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
