# Vérification alignement BBIA ↔ Pollen (tous domaines)

**Date** : 17 août 2026  
**Référence** : [pollen-robotics/reachy_mini](https://github.com/pollen-robotics/reachy_mini) **v1.9.0** (PyPI ; `main` = 1.10.0.dev0)

Ce document vérifie que BBIA est au même niveau que Pollen dans tous les domaines : SDK, fonctionnalités, outils, connexion, doc.

---

## Résumé en un coup d’œil

| Domaine | Niveau BBIA vs Pollen | Détail |
|--------|------------------------|--------|
| **Version SDK** | ✅ Même | reachy-mini **1.9.0** (référence PyPI), extra `.[reachy]` |
| **Dépendances (pyproject)** | ✅ Aligné | huggingface-hub ≥1.17, websockets &lt;16, aiohttp ≥3.14, pydantic ≥2.12.5, motor_controller ≥1.5.5, kinematics ≥1.0.3 |
| **API robot (méthodes SDK)** | ✅ Même | Surface 1.9/1.10 : tracking, IMU, wobbling, media, `play_move(sound)` |
| **Moteurs / reflash / scan** | ✅ Compatible | Utilise le même SDK ; reflash/scan documentés et CLI officiels utilisables |
| **Caméra / calibration** | ✅ Même niveau | Outils calibration (acquire, calibrate, scale, visualize) ; messages d’erreur clairs (issue #771) |
| **Mouvements / IK / émotions** | ✅ Même | goto_target, look_at_*, set_emotion, Move, RecordedMoves, create_head_pose |
| **Zenoh / Wireless** | ✅ Même niveau | Zenoh configurable (`REACHY_ZENOH_CONNECT` / `BBIA_ZENOH_CONNECT`) — issues #709, #677 |
| **Daemon / Dashboard** | 🔶 Volontairement différent | BBIA = daemon + dashboard propres ; Pollen = reachy-mini-daemon + dashboard officiel |
| **Apps HF** | ✅ Compatible | Référence aux mêmes apps (conversation, vision-demo, etc.) ; pas d’app Pollen à dupliquer |
| **WebRTC** | 🔶 Choix technique | Pollen : WebRTC navigateur ; BBIA : WebSocket/OpenCV pour le flux principal (suffisant pour notre stack) |
| **Documentation** | ✅ À jour | Doc BBIA mise à jour (avril 2026) ; doc officielle = Hugging Face |

Légende : **✅ Même / Aligné** = au même niveau fonctionnel ou mieux ; **🔶 Volontairement différent** = choix d’architecture (BBIA garde son daemon/dashboard).

---

## 1. Version SDK et dépendances

- **Pollen (v1.9.0)** : `reachy_mini` 1.9.0 sur PyPI, Python ≥3.11, `connection_mode`, huggingface-hub 1.17+, websockets &lt;16, zeroconf, rustypot (voir `pyproject.toml` officiel).
- **BBIA** : extra `reachy-mini>=1.9.0` ; floors critiques alignés ; `ReachyMiniBackend` passe `connection_mode` (repli `localhost_only` si SDK plus ancien).

**Verdict** : ✅ **Même niveau (référence 1.9.0).**

---

## 2. API robot (méthodes SDK)

- **Pollen** : ReachyMini — look_at_world, look_at_image, goto_target, set_target, start_head_tracking, get_tracked_face, imu, enable_wobbling, cancel_move, enable_motors(ids), set_automatic_body_yaw(enabled), play_move(..., sound), media.release, connection_mode.
- **BBIA** : `ReachyMiniBackend` et `BackendAdapter` exposent la même surface (wrappers sim-safe) ; `RobotFactory` transmet `robot_name` / `host` / `port` / `connection_mode`.

**Verdict** : ✅ **Même niveau API** (17 août 2026). Daemon/dashboard restent une stack BBIA volontairement distincte.

---

## 3. Moteurs, reflash, scan

- **Pollen** : reflash automatique au démarrage (v1.2.4+), CLI `reachy-mini-reflash-motors`, script scan baudrate/ID, page diagnostic moteurs dans le dashboard officiel.
- **BBIA** : utilise le même SDK (reflash au connect) ; doc et messages renvoient vers `reachy-mini-reflash-motors` et `examples/reachy_mini/scan_motors_baudrate.py` ; API daemon BBIA expose infos moteurs et erreurs.

**Verdict** : ✅ **Même niveau** (BBIA s’appuie sur le SDK et les outils officiels).

---

## 4. Caméra et calibration

- **Pollen** : calibration caméra (fix #741), set_resolution, outils calibration.
- **BBIA** : `src/bbia_sim/tools/camera_calibration/` (acquire, calibrate, scale_calibration, visualize_undistorted, analyze_crop) ; messages d’erreur caméra explicites (issue #771).

**Verdict** : ✅ **Même niveau.**

---

## 5. Mouvements, IK, émotions

- **Pollen** : Move, RecordedMoves, create_head_pose, interpolation, IK (rust-kinematics), 6 émotions officielles.
- **BBIA** : imports `reachy_mini.utils.create_head_pose`, `reachy_mini.motion.move.Move`, `reachy_mini.motion.recorded_move.RecordedMoves`, interpolation ; 6 émotions + extensions ; backend appelle goto_target, set_target_head_pose, set_emotion, etc.

**Verdict** : ✅ **Même niveau.**

---

## 6. Zenoh / Wireless

- **Pollen** : Zenoh pour état/télémétrie/commandes ; connexion configurable (WSL2, robot sur autre IP).
- **BBIA** : Zenoh dans `daemon/bridge.py` et `robot_registry.py` ; `REACHY_ZENOH_CONNECT` ou `BBIA_ZENOH_CONNECT` (ex. `tcp://<IP>:7447`) pour WSL2 / Wireless (mitigations #709, #677).

**Verdict** : ✅ **Même niveau** pour la connexion et la config.

---

## 7. Daemon et dashboard

- **Pollen** : `reachy-mini-daemon`, dashboard web officiel (HF auth en v1.3.0).
- **BBIA** : daemon propre (FastAPI) + dashboard propre ; même robot physique via `ReachyMini` et Zenoh.

**Verdict** : 🔶 **Volontairement différent** — même capacités robot, stack BBIA conservée.

---

## 8. Apps Hugging Face

- **Pollen** : apps (conversation, vision-demo, movements, ai-assistant, etc.) installables via l’assistant.
- **BBIA** : référence les mêmes apps / HF spaces ; pas de réimplémentation des apps officielles.

**Verdict** : ✅ **Compatible** ; pas de retard fonctionnel côté “apps”.

---

## 9. WebRTC

- **Pollen** : WebRTC pour apps dans le navigateur (v1.3.0).
- **BBIA** : flux vidéo principal via WebSocket/OpenCV ; pas de WebRTC côté BBIA (choix stack).

**Verdict** : 🔶 **Choix technique** — pas requis pour atteindre “même niveau” sur les cas d’usage BBIA (simu + robot + dashboard actuel).

---

## 10. Documentation

- **Pollen** : doc sur Hugging Face ; README, install, wireless, reflash macOS, etc.
- **BBIA** : docs hardware/quality/reference à jour (v1.9.0, 17 août 2026) ; [ISSUES_POLLEN_IMPACT_BBIA.md](../hardware/ISSUES_POLLEN_IMPACT_BBIA.md), [ANALYSE_REPO_OFFICIEL_JANVIER_2026.md](../hardware/ANALYSE_REPO_OFFICIEL_JANVIER_2026.md), conformité, audits.

**Verdict** : ✅ **À jour et cohérent.**

---

## Conclusion

- **Au même niveau que Pollen** dans : SDK, deps, API robot, moteurs/reflash/scan, caméra/calibration, mouvements/IK/émotions, Zenoh/Wireless, doc.
- **Volontairement différent** (sans retard) : daemon/dashboard BBIA, pas de WebRTC dans notre stack.
- **À faire côté robot physique** : sur le Pi, `pip install --upgrade reachy-mini` après installation des moteurs pour être aligné en pratique.

Pour les détails par fichier ou endpoint : [CONFORMITE_REACHY_MINI_COMPLETE.md](compliance/CONFORMITE_REACHY_MINI_COMPLETE.md), [CHECKLIST_FINALE_CONFORMITE.md](compliance/CHECKLIST_FINALE_CONFORMITE.md), [ISSUES_POLLEN_IMPACT_BBIA.md](../hardware/ISSUES_POLLEN_IMPACT_BBIA.md).
