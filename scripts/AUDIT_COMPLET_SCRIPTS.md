# 🔍 Audit Complet du Dossier Scripts

> Analyse exhaustive des scripts pour détecter doublons, obsolescence, dangers potentiels  
> **Date**: Novembre 2024  
> **⚠️ AUCUNE DÉCISION PRISE - ANALYSE SEULEMENT**

---

## 📊 Résumé Exécutif

### Statistiques Globales
- **Total scripts Python**: ~48 fichiers
- **Total scripts Shell**: ~21 fichiers
- **Total scripts analysés**: 69 fichiers

### Catégories Identifiées

| Catégorie | Nombre | Description |
|-----------|--------|-------------|
| 🚀 **Lanceurs API** | 3 | `start_api.py`, `start_public_api.py`, `demo_public_api.py` |
| 🤖 **Lanceurs Robot** | 4 | `launch_robot.py`, `launch_complete_robot.py`, `launch_robot_3d.sh`, `quick_start.sh` |
| 🧹 **Nettoyage Processus** | 3 | `kill_greedy_processes.sh`, `kill_mujoco_viewers.sh`, `smart_process_cleanup.sh` |
| 📊 **Dashboard** | 3 | `bbia_dashboard_server.py`, `bbia_advanced_dashboard_server.py`, `dashboard_gradio.py` |
| 🔧 **Hardware Tests** | 2 | `hardware_dry_run.py`, `hardware_dry_run_reachy_mini.py` |
| 🔍 **Vérification Joints** | 3 | `check_joints.py`, `analyze_joints_detailed.py`, `diagnose_joints.py` |
| 📋 **API Tests** | 2 | `test_public_api.py`, `demo_public_api.py` |
| 📚 **Documentation** | 3 | Scripts dans `docs/` |
| 🎤 **Voice/Audio** | 4 | `voice_demo.py`, `stt_demo.py`, scripts `voice_clone/` |
| 📹 **Vision** | 3 | `test_vision_webcam.py`, `test_webcam_simple.py`, `test_pose_detection.py` |
| 🧪 **Tests/Demos** | 6 | `test_deepface.py`, `demo_mode_complete.py`, etc. |
| 🔄 **Utilitaires** | 6 | `process_manager.py`, `veille_reachy_mini.py`, `verify_project.py`, etc. |

---

## 🚨 DOUBLONS IDENTIFIÉS

### 1. **Lanceurs API** - 3 fichiers (POTENTIEL DOUBLON)

#### `start_api.py` vs `start_public_api.py`
- **`start_api.py`** (34 lignes):
  - Simple wrapper uvicorn
  - Variables d'environnement: `BBIA_HOST`, `BBIA_PORT`, `BBIA_RELOAD`
  - Import direct: `"bbia_sim.daemon.app.main:app"`
  - ✅ **Fonctionnel mais basique**

- **`start_public_api.py`** (189 lignes):
  - Interface complète avec argparse
  - Modes: `--dev`, `--prod`, `--log-level`, `--workers`, `--sdk-telemetry`
  - Gestion logs dans `log/`
  - Messages détaillés de démarrage
  - ✅ **Plus complet et recommandé**

**Verdict**: `start_api.py` semble obsolète, `start_public_api.py` est la version complète.

---

### 2. **Lanceurs Robot** - 4 fichiers (POTENTIEL DOUBLON)

#### `launch_robot.py` vs `launch_complete_robot.py`
- **`launch_robot.py`** (48 lignes):
  - Simple wrapper qui appelle `launch_complete_robot.py`
  - Mode par défaut: `graphical`
  - ✅ **Alias pratique**

- **`launch_complete_robot.py`** (191 lignes):
  - Lanceur complet avec argparse
  - Support `--headless`, `--duration`, `--model`
  - Gestion simulation MuJoCo complète
  - ✅ **Fichier principal**

**Verdict**: `launch_robot.py` est un wrapper léger, pas un doublon réel. Complémentaire.

#### `launch_robot_3d.sh` vs `launch_complete_robot.py`
- **`launch_robot_3d.sh`** (162 lignes):
  - Script bash avec couleurs et menu
  - Support macOS (`mjpython`)
  - Mode interactif avec help
  - ✅ **Interface utilisateur améliorée**

**Verdict**: Complémentaire, offre une interface utilisateur.

---

### 3. **Tests Hardware** - 2 fichiers (DOUBLON PARTIEL)

#### `hardware_dry_run.py` vs `hardware_dry_run_reachy_mini.py`
- **`hardware_dry_run.py`** (385 lignes):
  - Tests pour backend générique `"reachy"` (Reachy 2)
  - Joints de test: `ReachyMapping.get_recommended_joints()`
  - Artefacts: `latency.csv`, `test_results.json`
  - ✅ **Pour Reachy 2**

- **`hardware_dry_run_reachy_mini.py`** (385 lignes):
  - Tests pour backend `"reachy_mini"` (SDK officiel)
  - Joints de test: `["head_1", "head_2", "body_yaw"]`
  - Artefacts: `latency_reachy_mini.csv`, `test_results_reachy_mini.json`
  - Tests émotions/comportements en plus
  - ✅ **Pour Reachy Mini spécifiquement**

**Verdict**: Pas vraiment un doublon, mais structure très similaire. Pourraient être refactorisés en un seul avec paramètre backend.

---

### 4. **Vérification Joints** - 3 fichiers (DOUBLON PARTIEL)

#### `check_joints.py` vs `analyze_joints_detailed.py` vs `diagnose_joints.py`
- **`check_joints.py`** (100 lignes):
  - Vérification simple des joints mobiles vs bloqués
  - Suggestions d'animation
  - ✅ **Basique**

- **`analyze_joints_detailed.py`** (107 lignes):
  - Analyse complète avec classification (safe/risky/forbidden)
  - Tableau complet avec types, ranges, degrés
  - ✅ **Détaillé**

- **`diagnose_joints.py`** (95 lignes):
  - Diagnostic par sécurité (safe/problématique/bloqué)
  - Recommandations d'utilisation
  - ✅ **Focalisé sécurité**

**Verdict**: Fonctions similaires avec focus différents. Pourraient être consolidés, mais chacun a une valeur.

---

### 5. **Nettoyage Processus** - 3 fichiers (DOUBLON PARTIEL)

#### `kill_greedy_processes.sh` vs `kill_mujoco_viewers.sh` vs `smart_process_cleanup.sh`
- **`kill_greedy_processes.sh`** (90 lignes):
  - Tue processus gourmands (CPU >10% ou RAM >500MB)
  - Garde: Cursor, ChatGPT, Perplexity, VS Code
  - Confirmation utilisateur
  - ⚠️ **DANGEREUX si mal utilisé**

- **`kill_mujoco_viewers.sh`** (38 lignes):
  - Tue spécifiquement les processus MuJoCo
  - Utilise `pkill -9 -f mujoco`
  - ⚠️ **Force kill (-9)**

- **`smart_process_cleanup.sh`** (106 lignes):
  - Nettoyage intelligent avec liste processus critiques
  - Seuil plus élevé (CPU >15% ou RAM >1GB)
  - Intègre `process_manager.py`
  - ✅ **Plus sûr et intelligent**

**Verdict**: `kill_greedy_processes.sh` et `kill_mujoco_viewers.sh` sont redondants avec `smart_process_cleanup.sh`.

---

### 6. **Dashboard** - 3 fichiers (COMPLÉMENTAIRES)

#### `bbia_dashboard_server.py` vs `bbia_advanced_dashboard_server.py` vs `dashboard_gradio.py`
- **`bbia_dashboard_server.py`** (72 lignes):
  - Dashboard web minimal
  - Backends: `mujoco`, `reachy`
  - ✅ **Basique**

- **`bbia_advanced_dashboard_server.py`** (69 lignes):
  - Dashboard avancé avec métriques temps réel
  - Backends: `mujoco`, `reachy`, `reachy_mini`
  - Chart.js, WebSocket
  - ✅ **Avancé**

- **`dashboard_gradio.py`** (264 lignes):
  - Interface Gradio no-code
  - Upload images, chat, DeepFace
  - ✅ **Interface utilisateur différente**

**Verdict**: Complémentaires, pas de doublons.

---

### 7. **API Tests** - 2 fichiers (COMPLÉMENTAIRES)

#### `test_public_api.py` vs `demo_public_api.py`
- **`test_public_api.py`** (328 lignes):
  - Tests automatisés de l'API
  - 9 tests: root, health, info, capabilities, status, emotions, behaviors, modes, OpenAPI
  - Code de sortie pour CI
  - ✅ **Tests**

- **`demo_public_api.py`** (402 lignes):
  - Démonstration interactive de l'API
  - Affichage détaillé des résultats
  - Pas de code de sortie strict
  - ✅ **Démonstration**

**Verdict**: Complémentaires, pas de doublons.

---

## ⚠️ SCRIPTS POTENTIELLEMENT DANGEREUX

### 🔴 **TRÈS DANGEREUX**

1. **`kill_greedy_processes.sh`**
   - **Risque**: Tue des processus système si mal utilisé
   - **Problème**: Seuils bas (CPU >10%, RAM >500MB)
   - **Recommandation**: Utiliser `smart_process_cleanup.sh` à la place

2. **`kill_mujoco_viewers.sh`**
   - **Risque**: Force kill (-9) peut corrompre des données
   - **Problème**: Pas de vérification avant kill
   - **Recommandation**: Utiliser `process_manager.py stop` à la place

3. **`cleanup_project.sh`**
   - **Risque**: Supprime des fichiers potentiellement importants
   - **Problème**: Suppression automatique de `.log`, `.tmp`, `__pycache__`
   - **Recommandation**: Ajouter confirmation avant suppression

### 🟡 **MODÉRÉMENT DANGEREUX**

4. **`hardware_dry_run.py` / `hardware_dry_run_reachy_mini.py`**
   - **Risque**: Peut essayer de se connecter à un robot physique
   - **Problème**: Pas de vérification explicite du mode simulation
   - **Recommandation**: Ajouter flag `--simulation-only`

5. **`bbia_safe.sh`**
   - **Risque**: Mode `kill-all` peut tuer des processus importants
   - **Problème**: Confirmation mais peut être accidentellement confirmé
   - **Recommandation**: ✅ Déjà sécurisé avec confirmation

---

## 📦 SCRIPTS POTENTIELLEMENT OBSOLÈTES

### 1. **`start_api.py`**
- **Raison**: Remplacé par `start_public_api.py` (plus complet)
- **Action suggérée**: Marquer comme deprecated ou supprimer

### 2. **`kill_greedy_processes.sh`**
- **Raison**: Remplacé par `smart_process_cleanup.sh` (plus intelligent)
- **Action suggérée**: Marquer comme deprecated

### 3. **`kill_mujoco_viewers.sh`**
- **Raison**: Remplacé par `process_manager.py stop` (plus sûr)
- **Action suggérée**: Marquer comme deprecated

---

## 🔄 UTILISATIONS D'OUTILS EN DOUBLE

### 1. **MuJoCo - Plusieurs lanceurs**
- `launch_complete_robot.py` - Lanceur Python
- `launch_robot_3d.sh` - Wrapper bash
- `launch_robot.py` - Wrapper Python
- **Impact**: Faible, complémentaires

### 2. **Uvicorn - Démarrage API**
- `start_api.py` - Simple
- `start_public_api.py` - Complet
- **Impact**: Moyen, `start_api.py` obsolète

### 3. **Processus Kill - Plusieurs méthodes**
- `kill_greedy_processes.sh` - Force kill
- `kill_mujoco_viewers.sh` - Force kill MuJoCo
- `smart_process_cleanup.sh` - Intelligent
- `process_manager.py` - Gestionnaire Python
- **Impact**: Moyen, redondance

### 4. **Joints Analysis - 3 scripts**
- `check_joints.py` - Basique
- `analyze_joints_detailed.py` - Détaillé
- `diagnose_joints.py` - Sécurité
- **Impact**: Faible, focus différents

---

## 📋 RECOMMANDATIONS (SANS ACTION)

### Priorité Haute

1. **Marquer comme deprecated**:
   - `start_api.py` → Utiliser `start_public_api.py`
   - `kill_greedy_processes.sh` → Utiliser `smart_process_cleanup.sh`
   - `kill_mujoco_viewers.sh` → Utiliser `process_manager.py stop`

### Priorité Moyenne

2. **Considérer consolidation**:
   - `hardware_dry_run.py` + `hardware_dry_run_reachy_mini.py` → Un seul script avec paramètre backend
   - `check_joints.py` + `analyze_joints_detailed.py` + `diagnose_joints.py` → Un seul avec modes

### Priorité Basse

3. **Documenter les rôles**:
   - Clarifier la différence entre `launch_robot.py` et `launch_complete_robot.py`
   - Documenter quand utiliser chaque dashboard

---

## 📊 TABLEAU RÉCAPITULATIF

| Script | Doublon ? | Obsolète ? | Dangereux ? | Action Recommandée |
|--------|-----------|------------|-------------|-------------------|
| `start_api.py` | ⚠️ Oui | ✅ Oui | ❌ Non | Deprecated |
| `start_public_api.py` | ❌ Non | ❌ Non | ❌ Non | ✅ Garder |
| `launch_robot.py` | ⚠️ Wrapper | ❌ Non | ❌ Non | ✅ Garder |
| `launch_complete_robot.py` | ❌ Non | ❌ Non | ❌ Non | ✅ Garder |
| `kill_greedy_processes.sh` | ⚠️ Oui | ✅ Oui | 🔴 Oui | Deprecated |
| `kill_mujoco_viewers.sh` | ⚠️ Oui | ✅ Oui | 🔴 Oui | Deprecated |
| `smart_process_cleanup.sh` | ❌ Non | ❌ Non | 🟡 Modéré | ✅ Garder |
| `hardware_dry_run.py` | ⚠️ Partiel | ❌ Non | 🟡 Modéré | Consolider |
| `hardware_dry_run_reachy_mini.py` | ⚠️ Partiel | ❌ Non | 🟡 Modéré | Consolider |
| `check_joints.py` | ⚠️ Partiel | ❌ Non | ❌ Non | Consolider |
| `analyze_joints_detailed.py` | ⚠️ Partiel | ❌ Non | ❌ Non | Consolider |
| `diagnose_joints.py` | ⚠️ Partiel | ❌ Non | ❌ Non | Consolider |

---

## ✅ SCRIPTS SANS PROBLÈME

- `process_manager.py` - Gestionnaire robuste
- `verify_project.py` - Vérification complète
- `veille_reachy_mini.py` - Veille automatique
- `check_official_alignment.py` - Alignement officiel
- `demo_mode_complete.py` - Démo complète
- `bbia_safe.sh` - Wrapper sécurisé
- `dashboard_gradio.py` - Interface unique
- Tous les scripts dans `docs/`, `onboarding/`, `voice_clone/`

---

## 🎯 CONCLUSION

### Doublons Majeurs Identifiés
- 3 scripts de nettoyage processus (dont 2 dangereux)
- 2 scripts de démarrage API (1 obsolète)
- 2 scripts de test hardware (structure similaire)
- 3 scripts d'analyse joints (focus différents)

### Actions Suggérées (à décider plus tard)
1. Marquer 3 scripts comme deprecated
2. Consolider 2 paires de scripts
3. Améliorer sécurité de 2 scripts dangereux

**⚠️ AUCUNE ACTION AUTOMATIQUE EFFECTUÉE**

---

**Date de l'audit**: Novembre 2024  
**Scripts analysés**: 69 fichiers  
**Doublons identifiés**: ~8  
**Scripts dangereux**: 3  
**Scripts obsolètes**: 3

