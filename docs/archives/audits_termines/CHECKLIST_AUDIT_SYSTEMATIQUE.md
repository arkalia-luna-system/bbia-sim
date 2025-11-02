---
**⚠️ ARCHIVE - DOCUMENT HISTORIQUE ⚠️**

Ce document a été archivé car il est devenu obsolète ou a été remplacé par une version plus récente.
Il est conservé à des fins de référence historique uniquement.

**Date d'archivage** : Oct / No2025025025025025
**Raison** : Document terminé/obsolète/remplacé
---

# Checklist Finale - Audit Systématique BBIA-SIM vs SDK Officiel

**Date:** Oct / No2025025025025025
**Total éléments vérifiés:** 94

## Résumé

- **OK:** 68
- **DIFF:** 9
- **MISSING:** 17
- **EXTRA:** 0
- **Corrigés:** 0
- **Testés:** 0

## Checklist Détaillée

### API

| Nature | Fichier | Ligne | Status | Fix | Test | Description |
|--------|---------|-------|--------|-----|------|-------------|
| endpoint | `src/bbia_sim/daemon/app/routers/apps.py` | 82 | ⚠️ DIFF | ❌ | ❌ | GET /list-available/{source_kind} |
| endpoint | `src/bbia_sim/daemon/app/routers/apps.py` | 99 | ⚠️ DIFF | ❌ | ❌ | GET /list-available |
| endpoint | `src/bbia_sim/daemon/app/routers/apps.py` | 110 | ⚠️ DIFF | ❌ | ❌ | POST /install |
| endpoint | `src/bbia_sim/daemon/app/routers/apps.py` | 139 | ⚠️ DIFF | ❌ | ❌ | POST /remove/{app_name} |
| endpoint | `src/bbia_sim/daemon/app/routers/apps.py` | 166 | ⚠️ DIFF | ❌ | ❌ | GET /job-status/{job_id} |
| endpoint | `src/bbia_sim/daemon/app/routers/apps.py` | 217 | ⚠️ DIFF | ❌ | ❌ | POST /start-app/{app_name} |
| endpoint | `src/bbia_sim/daemon/app/routers/apps.py` | 250 | ⚠️ DIFF | ❌ | ❌ | POST /restart-current-app |
| endpoint | `src/bbia_sim/daemon/app/routers/apps.py` | 271 | ⚠️ DIFF | ❌ | ❌ | POST /stop-current-app |
| endpoint | `src/bbia_sim/daemon/app/routers/apps.py` | 293 | ⚠️ DIFF | ❌ | ❌ | GET /current-app-status |
| endpoint | `src/bbia_sim/daemon/app/routers/daemon.py` | 39 | ✅ OK | ❌ | ❌ | POST /start |
| endpoint | `src/bbia_sim/daemon/app/routers/daemon.py` | 96 | ✅ OK | ❌ | ❌ | POST /stop |
| endpoint | `src/bbia_sim/daemon/app/routers/daemon.py` | 139 | ✅ OK | ❌ | ❌ | POST /restart |
| endpoint | `src/bbia_sim/daemon/app/routers/daemon.py` | 178 | ✅ OK | ❌ | ❌ | GET /status |
| endpoint | `src/bbia_sim/daemon/app/routers/ecosystem.py` | 65 | ✅ OK | ❌ | ❌ | GET /capabilities |
| endpoint | `src/bbia_sim/daemon/app/routers/ecosystem.py` | 95 | ✅ OK | ❌ | ❌ | GET /status |
| endpoint | `src/bbia_sim/daemon/app/routers/ecosystem.py` | 133 | ✅ OK | ❌ | ❌ | POST /emotions/apply |
| endpoint | `src/bbia_sim/daemon/app/routers/ecosystem.py` | 190 | ✅ OK | ❌ | ❌ | POST /behaviors/execute |
| endpoint | `src/bbia_sim/daemon/app/routers/ecosystem.py` | 245 | ✅ OK | ❌ | ❌ | GET /emotions/available |
| endpoint | `src/bbia_sim/daemon/app/routers/ecosystem.py` | 291 | ✅ OK | ❌ | ❌ | GET /behaviors/available |
| endpoint | `src/bbia_sim/daemon/app/routers/ecosystem.py` | 333 | ✅ OK | ❌ | ❌ | GET /demo/modes |
| endpoint | `src/bbia_sim/daemon/app/routers/ecosystem.py` | 380 | ✅ OK | ❌ | ❌ | POST /demo/start |
| endpoint | `src/bbia_sim/daemon/app/routers/kinematics.py` | 26 | ✅ OK | ❌ | ❌ | GET /info |
| endpoint | `src/bbia_sim/daemon/app/routers/kinematics.py` | 78 | ✅ OK | ❌ | ❌ | GET /urdf |
| endpoint | `src/bbia_sim/daemon/app/routers/kinematics.py` | 119 | ✅ OK | ❌ | ❌ | GET /stl/{filename:path} |
| endpoint | `src/bbia_sim/daemon/app/routers/motion.py` | 31 | ✅ OK | ❌ | ❌ | POST /goto_pose |
| endpoint | `src/bbia_sim/daemon/app/routers/motion.py` | 109 | ✅ OK | ❌ | ❌ | POST /home |
| endpoint | `src/bbia_sim/daemon/app/routers/motion.py` | 126 | ✅ OK | ❌ | ❌ | POST /joints |
| endpoint | `src/bbia_sim/daemon/app/routers/motion.py` | 173 | ✅ OK | ❌ | ❌ | POST /gripper/{side} |
| endpoint | `src/bbia_sim/daemon/app/routers/motion.py` | 207 | ✅ OK | ❌ | ❌ | POST /head |
| endpoint | `src/bbia_sim/daemon/app/routers/motion.py` | 237 | ✅ OK | ❌ | ❌ | POST /wake_up |
| endpoint | `src/bbia_sim/daemon/app/routers/motion.py` | 275 | ✅ OK | ❌ | ❌ | POST /goto_sleep |
| endpoint | `src/bbia_sim/daemon/app/routers/motion.py` | 315 | ✅ OK | ❌ | ❌ | POST /stop |
| endpoint | `src/bbia_sim/daemon/app/routers/motion.py` | 359 | ✅ OK | ❌ | ❌ | GET /status |
| endpoint | `src/bbia_sim/daemon/app/routers/motion.py` | 377 | ✅ OK | ❌ | ❌ | POST /custom |
| endpoint | `src/bbia_sim/daemon/app/routers/motors.py` | 42 | ✅ OK | ❌ | ❌ | GET /status |
| endpoint | `src/bbia_sim/daemon/app/routers/motors.py` | 83 | ✅ OK | ❌ | ❌ | POST /set_mode/{mode} |
| endpoint | `src/bbia_sim/daemon/app/routers/move.py` | 144 | ✅ OK | ❌ | ❌ | GET /running |
| endpoint | `src/bbia_sim/daemon/app/routers/move.py` | 150 | ✅ OK | ❌ | ❌ | POST /goto |
| endpoint | `src/bbia_sim/daemon/app/routers/move.py` | 167 | ✅ OK | ❌ | ❌ | POST /play/wake_up |
| endpoint | `src/bbia_sim/daemon/app/routers/move.py` | 175 | ✅ OK | ❌ | ❌ | POST /play/goto_sleep |
| endpoint | `src/bbia_sim/daemon/app/routers/move.py` | 194 | ✅ OK | ❌ | ❌ | POST /stop |
| endpoint | `src/bbia_sim/daemon/app/routers/move.py` | 219 | ✅ OK | ❌ | ❌ | POST /set_target |
| endpoint | `src/bbia_sim/daemon/app/routers/sanity.py` | 36 | ✅ OK | ❌ | ❌ | GET /status |
| endpoint | `src/bbia_sim/daemon/app/routers/sanity.py` | 46 | ✅ OK | ❌ | ❌ | POST /emergency_stop |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 137 | ✅ OK | ❌ | ❌ | GET /full |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 257 | ✅ OK | ❌ | ❌ | GET /position |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 274 | ✅ OK | ❌ | ❌ | GET /battery |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 309 | ✅ OK | ❌ | ❌ | GET /temperature |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 338 | ✅ OK | ❌ | ❌ | GET /status |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 357 | ✅ OK | ❌ | ❌ | POST /simulation/start |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 385 | ✅ OK | ❌ | ❌ | POST /simulation/stop |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 406 | ✅ OK | ❌ | ❌ | GET /joints |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 425 | ✅ OK | ❌ | ❌ | GET /present_head_pose |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 536 | ✅ OK | ❌ | ❌ | GET /present_body_yaw |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 580 | ✅ OK | ❌ | ❌ | GET /present_antenna_joint_positions |
| endpoint | `src/bbia_sim/daemon/app/routers/state.py` | 698 | ✅ OK | ❌ | ❌ | GET /sensors |

### CLASS

| Nature | Fichier | Ligne | Status | Fix | Test | Description |
|--------|---------|-------|--------|-----|------|-------------|
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: async_play_move |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: disable_gravity_compensation |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: disable_motors |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: enable_gravity_compensation |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: enable_motors |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: get_current_head_pose |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: get_current_joint_positions |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: get_present_antenna_joint_positions |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: goto_sleep |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: goto_target |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: look_at_image |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: look_at_world |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: play_move |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: set_automatic_body_yaw |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: set_target |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: set_target_antenna_joint_positions |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: set_target_body_yaw |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: set_target_head_pose |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: start_recording |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: stop_recording |
| method | `src/bbia_sim/backends/reachy_mini_backend.py` | N/A | ✅ OK | ❌ | ❌ | Méthode SDK: wake_up |

### MODEL

| Nature | Fichier | Ligne | Status | Fix | Test | Description |
|--------|---------|-------|--------|-----|------|-------------|
| mujoco_xml | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Modèle MuJoCo: additional.xml |
| mujoco_xml | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Modèle MuJoCo: ._additional.xml |
| mujoco_xml | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Modèle MuJoCo: joints_properties.xml |
| mujoco_xml | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Modèle MuJoCo: ._joints_properties.xml |
| mujoco_xml | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Modèle MuJoCo: reachy_mini.xml |
| mujoco_xml | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Modèle MuJoCo: ._reachy_mini.xml |
| mujoco_xml | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Modèle MuJoCo: scene.xml |
| mujoco_xml | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Modèle MuJoCo: ._scene.xml |

### TEST

| Nature | Fichier | Ligne | Status | Fix | Test | Description |
|--------|---------|-------|--------|-----|------|-------------|
| test_file | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Test: test_analytical_kinematics.py |
| test_file | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Test: test_app.py |
| test_file | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Test: test_audio.py |
| test_file | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Test: test_collision.py |
| test_file | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Test: test_daemon.py |
| test_file | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Test: test_import.py |
| test_file | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Test: test_placo.py |
| test_file | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Test: test_video.py |
| test_file | `MISSING` | N/A | ❌ MISSING | ❌ | ❌ | Test: test_wireless.py |
