# 🔍 PROMPT D'AUDIT EXHAUSTIF WINDSURF - BBIA-SIM v1.3.2

## ⚠️ RÈGLES ABSOLUES - À RESPECTER IMPÉRATIVEMENT

**🚫 INTERDICTION FORMELLE :**

- **NE MODIFIE AUCUN FICHIER**
- **NE CRÉE AUCUN FICHIER**
- **NE SUPPRIME AUCUN FICHIER**
- **NE PROPOSE AUCUN CORRECTIF DE CODE**

**✅ TON RÔLE UNIQUE : AUDITEUR EXPERT**

- Analyse en profondeur
- Identifie les problèmes
- Liste les incohérences
- Documente les découvertes
- Fournis un rapport d'audit complet

---

## 🎯 CONTEXTE DU PROJET

**Projet :** BBIA-SIM v1.3.2  
**Type :** Moteur cognitif pour robot humanoïde Reachy Mini  
**Développeur :** Pollen Robotics  
**Repo officiel :** https://github.com/pollen-robotics/reachy_mini  
**SDK officiel :** `reachy_mini` (Python 3.10-3.13)

**Technologies clés :**
- MuJoCo (simulation physique 3D)
- Zenoh (middleware communication robot réel)
- SDK Reachy Mini officiel
- Vision AI (OpenCV, MediaPipe, Hugging Face)
- Dashboard Web (FastAPI + WebSocket)

**Chemin racine du projet :** `/Volumes/T7/bbia-reachy-sim/`

---

## 📂 CARTE DES FICHIERS À ANALYSER (CHEMINS EXACTS)

### 🔴 FICHIERS CRITIQUES (Priorité 1 - Analyser en premier)

#### Backends Robot (Communication Hardware)
```
src/bbia_sim/backends/reachy_mini_backend.py          # Backend Reachy Mini (715 lignes)
src/bbia_sim/backends/mujoco_backend.py                # Backend MuJoCo (223 lignes)
src/bbia_sim/backends/reachy_backend.py                # Backend Reachy classique (195 lignes)
src/bbia_sim/backends/simulation_shims.py              # Shims simulation (56 lignes)
src/bbia_sim/robot_factory.py                         # Factory pattern (32 lignes)
src/bbia_sim/robot_api.py                              # API unifiée robot (178 lignes)
```

#### Daemon & Communication Zenoh
```
src/bbia_sim/daemon/bridge.py                          # Bridge Zenoh (388 lignes) - CRITIQUE
src/bbia_sim/daemon/simulation_service.py              # Service simulation (121 lignes)
src/bbia_sim/daemon/app/main.py                        # API FastAPI principale (91 lignes)
src/bbia_sim/daemon/app/backend_adapter.py            # Adapter backend (277 lignes)
src/bbia_sim/daemon/ws/telemetry.py                   # WebSocket télémétrie (147 lignes)
src/bbia_sim/daemon/ws/__init__.py                    # WebSocket init (111 lignes)
```

#### Routers API REST
```
src/bbia_sim/daemon/app/routers/state.py               # Router état robot (245 lignes)
src/bbia_sim/daemon/app/routers/move.py                # Router mouvements (160 lignes)
src/bbia_sim/daemon/app/routers/motion.py              # Router motion (135 lignes)
src/bbia_sim/daemon/app/routers/ecosystem.py           # Router écosystème (219 lignes)
src/bbia_sim/daemon/app/routers/metrics.py             # Router métriques (115 lignes)
src/bbia_sim/daemon/app/routers/kinematics.py          # Router cinématique (36 lignes)
src/bbia_sim/daemon/app/routers/apps.py                # Router apps (90 lignes)
src/bbia_sim/daemon/app/routers/daemon.py              # Router daemon (83 lignes)
```

### 🟠 FICHIERS IMPORTANTS (Priorité 2)

#### Vision & IA
```
src/bbia_sim/bbia_vision.py                           # Vision principale (520 lignes)
src/bbia_sim/vision_yolo.py                           # YOLO detection (186 lignes)
src/bbia_sim/face_recognition.py                      # Reconnaissance faciale (139 lignes)
src/bbia_sim/pose_detection.py                        # Détection pose (95 lignes)
src/bbia_sim/bbia_huggingface.py                      # Hugging Face integration (900 lignes) - TRÈS GROS
```

#### Voice & Audio
```
src/bbia_sim/voice_whisper.py                         # Whisper STT (361 lignes)
src/bbia_sim/bbia_voice.py                            # Voice principal (263 lignes)
src/bbia_sim/bbia_voice_advanced.py                    # Voice avancé (174 lignes)
src/bbia_sim/bbia_audio.py                            # Audio processing (169 lignes)
```

#### Behavior & Emotions
```
src/bbia_sim/bbia_behavior.py                         # Behavior manager (518 lignes)
src/bbia_sim/bbia_emotions.py                         # Emotions (81 lignes)
src/bbia_sim/bbia_emotion_recognition.py              # Emotion recognition (239 lignes)
src/bbia_sim/bbia_idle_animations.py                  # Animations idle (187 lignes)
src/bbia_sim/bbia_adaptive_behavior.py                # Behavior adaptatif (260 lignes)
```

#### Dashboard & UI
```
src/bbia_sim/dashboard_advanced.py                    # Dashboard avancé (3678 lignes) - TRÈS GROS
src/bbia_sim/dashboard.py                             # Dashboard simple (147 lignes)
```

### 🟡 FICHIERS SUPPORT (Priorité 3)

```
src/bbia_sim/bbia_integration.py                       # Intégration (249 lignes)
src/bbia_sim/bbia_memory.py                           # Mémoire (107 lignes)
src/bbia_sim/bbia_tools.py                             # Outils BBIA (190 lignes)
src/bbia_sim/ai_backends.py                           # Backends IA (204 lignes)
src/bbia_sim/bbia_awake.py                             # Module awake (15 lignes)
src/bbia_sim/mapping_reachy.py                         # Mapping joints (55 lignes)
src/bbia_sim/model_optimizer.py                        # Optimiseur modèles (23 lignes)
src/bbia_sim/global_config.py                          # Config globale (49 lignes)
src/bbia_sim/troubleshooting.py                        # Troubleshooting (195 lignes)
src/bbia_sim/telemetry.py                              # Télémétrie (66 lignes)
src/bbia_sim/unity_reachy_controller.py               # Unity controller (140 lignes)
```

#### Simulation MuJoCo
```
src/bbia_sim/sim/simulator.py                          # Simulateur MuJoCo (152 lignes)
src/bbia_sim/sim/joints.py                             # Gestion joints (18 lignes)
src/bbia_sim/sim/models/reachy_mini.xml                # Modèle MuJoCo
src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml  # Modèle officiel
src/bbia_sim/sim/scenes/minimal.xml                    # Scène minimale
```

### 📋 FICHIERS DE CONFIGURATION

```
pyproject.toml                                         # Configuration projet (371 lignes)
README.md                                              # Documentation principale
CHANGELOG.md                                           # Historique versions
requirements.txt                                       # Dépendances
```

### 📚 DOCUMENTATION À VÉRIFIER

```
docs/development/architecture/ARCHITECTURE_OVERVIEW.md
docs/quality/compliance/CONFORMITE_REACHY_MINI_COMPLETE.md
docs/quality/performance/OPTIMISATIONS_PERFORMANCE_DEC2025.md
docs/hardware/reachy-mini/REACHY_MINI_REFERENCE.md
```

### 🧪 TESTS À ANALYSER

```
tests/test_reachy_mini_backend.py                      # Tests backend Reachy Mini
tests/test_daemon_bridge.py                            # Tests bridge Zenoh
tests/test_dashboard_advanced.py                       # Tests dashboard
tests/test_bbia_vision.py                              # Tests vision
tests/test_bbia_huggingface.py                         # Tests Hugging Face
tests/test_robot_api.py                                # Tests API robot
```

---

## 🔬 MISSION : AUDIT EN 10 PHASES (ANALYSE EXHAUSTIVE)

### 📋 PHASE 1 : ARCHITECTURE ET STRUCTURE DU CODE

**Objectif :** Cartographie complète de l'architecture et identification des incohérences structurelles.

**Actions concrètes à effectuer :**

1. **Cartographie des modules - INSTRUCTIONS PRÉCISES**

   **Action 1.1 : Analyser les imports dans `src/bbia_sim/backends/reachy_mini_backend.py`**
   
   **INSTRUCTION EXACTE :**
   1. Ouvre le fichier `src/bbia_sim/backends/reachy_mini_backend.py`
   2. Lis les lignes 14-27 (section imports)
   3. Vérifie si ces imports existent :
      - Ligne 15 : `from reachy_mini import ReachyMini`
      - Ligne 16 : `from reachy_mini.utils import create_head_pose`
      - Ligne 27 : `from ..robot_api import RobotAPI` (import relatif)
   
   **RÉSULTAT ATTENDU :**
   - Liste les imports trouvés
   - Identifie les imports relatifs (commençant par `.` ou `..`)
   - Note si `ReachyMini` et `create_head_pose` sont importés correctement

   **Action 1.2 : Chercher les dépendances circulaires**
   
   **INSTRUCTION EXACTE :**
   1. Pour chaque fichier dans `src/bbia_sim/`, extrais TOUS les imports
   2. Crée un tableau : Fichier A → Importe Fichier B
   3. Cherche les cycles : A → B → A (dépendance circulaire)
   
   **PATTERNS EXACTS À CHERCHER :**
   - `from . import` (import relatif même niveau)
   - `from .. import` (import relatif niveau parent)
   - `from ... import` (import relatif 2 niveaux parent)
   
   **EXEMPLE CONCRET :**
   Dans `src/bbia_sim/backends/reachy_mini_backend.py` ligne 27 :
   ```python
   from ..robot_api import RobotAPI
   ```
   Vérifie si `robot_api.py` importe quelque chose de `backends/`

2. **Fichiers orphelins et redondance - INSTRUCTIONS PRÉCISES**

   **Action 1.3 : Fichiers orphelins macOS**
   
   **INSTRUCTION EXACTE :**
   1. Liste TOUS les fichiers dans `src/bbia_sim/` qui commencent par `._`
   2. Exemple attendu : `._dashboard_advanced.py`, `._vision_yolo.py`, etc.
   
   **RÉSULTAT ATTENDU :**
   - Liste complète des fichiers `._*.py` trouvés
   - Ces fichiers sont des artifacts macOS à supprimer
   
   **Action 1.4 : Fichiers non importés**
   - Pour chaque fichier Python, vérifie s'il est importé ailleurs
   - Cherche EXACTEMENT le nom du fichier (sans extension) dans TOUT le projet
   - Exemple : `mapping_reachy.py` → cherche `mapping_reachy` ou `from .mapping_reachy`

3. **Organisation des dossiers**
   - Compare la structure avec les standards Python (PEP 8)
   - Vérifie si `daemon/`, `backends/`, `sim/` sont bien organisés
   - Identifie les modules mal placés (ex: `utils/` vs `bbia_*`)

4. **Analyse des imports**
   - Pour chaque fichier, liste les imports inutilisés (fonction non appelée)
   - Détecte les imports relatifs vs absolus incohérents
   - Vérifie les imports conditionnels manquants (ex: `FASTAPI_AVAILABLE`)

**Livrables attendus :**
- Diagramme ASCII de l'architecture (graphe de dépendances)
- Liste des fichiers orphelins avec justification
- Tableau des dépendances circulaires (fichier1 ↔ fichier2)
- Score de cohérence structurelle (/10)

**Fichiers à analyser en priorité :**
```
src/bbia_sim/__init__.py
src/bbia_sim/backends/__init__.py
src/bbia_sim/daemon/__init__.py
src/bbia_sim/daemon/app/__init__.py
src/bbia_sim/daemon/app/routers/__init__.py
```

---

### 🤝 PHASE 2 : COMPATIBILITÉ SDK REACHY MINI OFFICIEL

**Objectif :** Vérification ligne par ligne de la compatibilité avec le repo officiel `pollen-robotics/reachy_mini`.

**📌 RÉFÉRENCE OFFICIELLE :** https://github.com/pollen-robotics/reachy_mini

**Structure officielle à comparer :**
- Repo officiel : `src/reachy_mini/` (pas `bbia_sim/`)
- Daemon officiel : `reachy_mini.daemon.app.main` (module `reachy_mini.daemon.app.main`)
- SDK officiel : `from reachy_mini import ReachyMini`
- Utils officiels : `from reachy_mini.utils import create_head_pose`
- Commande CLI officielle : `reachy-mini-daemon` (pas `bbia-sim`)

**Actions concrètes à effectuer :**

1. **Comparaison des versions de dépendances - INSTRUCTIONS ULTRA-PRÉCISES**

   **Action 2.6 : Comparer les versions de dépendances**
   
   **INSTRUCTION EXACTE :**
   1. Ouvre `pyproject.toml`
   2. Lis les lignes 31-71 (section `[project]` → `dependencies`)
   3. Pour CHAQUE ligne, note :
      - Nom du package
      - Version dans BBIA (ex: `>=1.0.0`)
      - Version dans le repo officiel (à comparer avec https://github.com/pollen-robotics/reachy_mini/blob/develop/pyproject.toml)
   
   **EXEMPLES CONCRETS :**
   Ligne 48 dans `pyproject.toml` :
   ```toml
   "reachy_mini_motor_controller>=1.0.0",
   ```
   
   **VÉRIFICATIONS À FAIRE :**
   - [ ] Quelle version exacte est dans le repo officiel ?
   - [ ] Y a-t-il un écart de version majeur/mineur/patch ?
   
   **RÉSULTAT ATTENDU :**
   Tableau :
   | Package | Version BBIA | Version officielle | Écart | Impact |
   |---------|--------------|---------------------|-------|--------|
   | reachy_mini_motor_controller | >=1.0.0 | ? | ? | ? |

2. **API SDK Reachy Mini - INSTRUCTIONS ULTRA-PRÉCISES**

   **Action 2.1 : Vérifier l'utilisation de `ReachyMini` dans le code**
   
   **INSTRUCTION EXACTE :**
   1. Ouvre `src/bbia_sim/backends/reachy_mini_backend.py`
   2. Cherche EXACTEMENT la chaîne `ReachyMini(` (avec parenthèse ouvrante)
   3. Note le numéro de ligne de chaque occurrence
   
   **EXEMPLE CONCRET TROUVÉ :**
   Ligne 204 dans `reachy_mini_backend.py` :
   ```python
   self.robot = ReachyMini(
       localhost_only=self.localhost_only,
       spawn_daemon=self.spawn_daemon,
       use_sim=False,
       timeout=min(self.timeout, 3.0),
       automatic_body_yaw=self.automatic_body_yaw,
       log_level=self.log_level,
       media_backend=self.media_backend,
   )
   ```
   
   **VÉRIFICATIONS À FAIRE :**
   - [ ] Est-ce que `localhost_only` est passé en paramètre ? (OUI - ligne 205)
   - [ ] Est-ce que `timeout` est passé ? (OUI - ligne 208-211)
   - [ ] Est-ce que `use_sim` est passé ? (OUI - ligne 207)
   - [ ] Compare avec l'exemple officiel du README Reachy Mini
   
   **RÉSULTAT ATTENDU :**
   Tableau :
   | Ligne | Code | Paramètres utilisés | Conforme officiel ? |
   |-------|------|---------------------|---------------------|
   | 204   | `ReachyMini(...)` | localhost_only, timeout, use_sim, etc. | À vérifier |

   **Action 2.2 : Vérifier l'utilisation de `create_head_pose`**
   
   **INSTRUCTION EXACTE :**
   1. Cherche EXACTEMENT la chaîne `create_head_pose(` dans TOUT le projet
   2. Pour chaque occurrence, note :
      - Fichier
      - Ligne
      - Paramètres utilisés
   
   **EXEMPLES CONCRETS TROUVÉS :**
   - `src/bbia_sim/backends/reachy_mini_backend.py` ligne 680 : `create_head_pose(pitch=0.1, degrees=False)`
   - `src/bbia_sim/daemon/bridge.py` ligne 365 : `create_head_pose(pitch=0.1, yaw=0.0, degrees=False)`
   
   **VÉRIFICATIONS À FAIRE :**
   - [ ] Est-ce que `degrees=False` est utilisé ? (OUI dans les exemples)
   - [ ] Est-ce que `pitch` et `yaw` sont utilisés ? (OUI)
   - [ ] Est-ce que `z=`, `roll=`, `mm=True` sont utilisés ? (À vérifier - pas dans les exemples)
   
   **RÉSULTAT ATTENDU :**
   Liste de toutes les utilisations avec paramètres

3. **Backend Reachy Mini - INSTRUCTIONS ULTRA-PRÉCISES**

   **Action 2.3 : Vérifier les arguments CLI du daemon**
   
   **INSTRUCTION EXACTE :**
   1. Ouvre `src/bbia_sim/daemon/app/main.py`
   2. Cherche les arguments CLI avec `argparse` ou `click`
   3. Compare avec les arguments officiels :
      - `--localhost-only` (officiel)
      - `--no-localhost-only` (officiel)
      - `--sim` (officiel)
      - `--scene <empty|minimal>` (officiel)
      - `-p <serial_port>` (officiel)
   
   **PATTERNS EXACTS À CHERCHER :**
   - `add_argument("--localhost-only"`
   - `add_argument("--sim"`
   - `add_argument("--scene"`
   - `add_argument("-p"`
   
   **RÉSULTAT ATTENDU :**
   - Liste des arguments CLI trouvés dans BBIA
   - Liste des arguments officiels manquants
   
   **Action 2.4 : Vérifier la gestion des connexions**
   - Cherche EXACTEMENT la chaîne `localhost_only` dans le code BBIA
   - Vérifie si BBIA accepte connexions réseau (sécurité)
   - Compare la logique de détection automatique du port série

4. **Communication Zenoh - INSTRUCTIONS ULTRA-PRÉCISES**

   **Action 2.5 : Analyser le bridge Zenoh ligne par ligne**
   
   **INSTRUCTION EXACTE :**
   1. Ouvre `src/bbia_sim/daemon/bridge.py`
   2. Lis TOUTES les lignes de 1 à 388
   3. Pour chaque utilisation de `zenoh`, note :
      - Ligne
      - Code exact
      - Type d'opération (open, publish, subscribe)
   
   **EXEMPLES CONCRETS À CHERCHER :**
   - `zenoh.open(` ou `Session.open(`
   - `session.declare_publisher(`
   - `session.declare_subscriber(`
   
   **RÉSULTAT ATTENDU :**
   Liste complète des opérations Zenoh avec lignes

5. **Simulation vs Robot Réel (COHÉRENCE API)**
   - Analyse la logique de switch `use_sim=True/False` dans `robot_factory.py`
   - **Compare les APIs :**
     - `mujoco_backend.py` : méthodes disponibles
     - `reachy_mini_backend.py` : méthodes disponibles
     - Vérifie si les deux backends ont la même interface
   - **Vérifications spécifiques :**
     - `goto_target()` : même signature dans les deux backends ?
     - `get_joint_pos()` : même comportement ?
     - `get_image()` : même format de retour ?
     - `get_telemetry()` : même structure de données ?
   - Identifie les incohérences de comportement (sim vs réel)

6. **Structure du Daemon (CONFORMITÉ OFFICIELLE)**
   - **Compare la structure :**
     - Officiel : `reachy_mini.daemon.app.main` → `python -m reachy_mini.daemon.app.main`
     - BBIA : `bbia_sim.daemon.app.main` → vérifie si c'est équivalent
   - **Vérifie les endpoints REST :**
     - Officiel : `/api/state/full` (exemple donné dans README)
     - BBIA : compare tous les endpoints dans `daemon/app/routers/`
     - Vérifie si les endpoints BBIA sont compatibles avec l'API officielle
   - **Dashboard :**
     - Officiel : dashboard à `http://localhost:8000/`
     - BBIA : vérifie si le dashboard est accessible au même endpoint
     - Compare les fonctionnalités du dashboard

7. **Installation et CLI (CONFORMITÉ)**
   - **Vérifie les commandes CLI :**
     - Officiel : `reachy-mini-daemon` (entry point)
     - BBIA : vérifie s'il y a un équivalent dans `pyproject.toml` section `[project.scripts]`
   - **Vérifie l'installation :**
     - Officiel : `pip install reachy-mini` ou `pip install -e ./reachy_mini`
     - BBIA : compare avec l'installation BBIA
   - **Dépendances optionnelles :**
     - Officiel : `pip install reachy-mini[mujoco]` pour simulation
     - BBIA : vérifie si les extras sont bien définis dans `pyproject.toml`

**Livrables attendus :**
- Tableau de compatibilité des versions (package | version BBIA | version officielle | écart)
- Liste des API manquantes avec exemples d'usage
- Rapport d'écarts SDK officiel vs implémentation BBIA
- Liste des arguments CLI manquants avec impact
- Score de compatibilité (/10)

---

### 🔬 PHASE 2B : MICRO-DÉTAILS CRITIQUES (CE QUI FAIT TOUTE LA DIFFÉRENCE)

**Objectif :** Identifier les petits détails qui peuvent causer des bugs subtils mais critiques.

**Actions concrètes à effectuer :**

1. **Gestion des erreurs silencieuses - INSTRUCTIONS PRÉCISES**

   **Action 2B.1 : Chercher les exceptions silencieuses**
   
   **INSTRUCTION EXACTE :**
   1. Ouvre `src/bbia_sim/backends/reachy_mini_backend.py`
   2. Cherche EXACTEMENT le pattern : `except Exception as e:`
   3. Pour chaque occurrence, vérifie :
      - Y a-t-il un `logger.error()` ou `logger.warning()` après ?
      - Y a-t-il un `pass` sans log ?
      - Y a-t-il un `raise` pour remonter l'erreur ?
   
   **EXEMPLES CONCRETS TROUVÉS :**
   Ligne 239 dans `reachy_mini_backend.py` :
   ```python
   except Exception as e:
       logger.warning(f"Erreur lors de la connexion: {e}")
   ```
   
   **PATTERNS EXACTS À CHERCHER :**
   - `except Exception:` suivi de `pass`
   - `except Exception as e:` sans `logger.`
   - `except:` (bare except) - TRÈS DANGEREUX
   
   **RÉSULTAT ATTENDU :**
   Liste avec fichier, ligne, code, problème, impact

2. **Gestion des timeouts - INSTRUCTIONS PRÉCISES**

   **Action 2B.2 : Chercher les timeouts manquants**
   
   **INSTRUCTION EXACTE :**
   1. Cherche EXACTEMENT la chaîne `time.sleep(` dans TOUT le projet
   2. Cherche EXACTEMENT la chaîne `timeout=` dans TOUT le projet
   3. Pour chaque `time.sleep()`, vérifie :
      - Est-ce dans une boucle `while True` ?
      - Y a-t-il un mécanisme de timeout global ?
   
   **RÉSULTAT ATTENDU :**
   Liste des timeouts trouvés et manquants

3. **Context managers - INSTRUCTIONS PRÉCISES**

   **Action 2B.3 : Chercher les context managers manquants**
   
   **INSTRUCTION EXACTE :**
   1. Cherche EXACTEMENT la chaîne `ReachyMini(` (instanciation)
   2. Vérifie si c'est dans un `with` statement
   3. Cherche `with ReachyMini(`
   
   **EXEMPLE OFFICIEL :**
   ```python
   with ReachyMini() as reachy_mini:
       # code
   ```
   
   **EXEMPLE TROUVÉ DANS BBIA :**
   Ligne 204 dans `reachy_mini_backend.py` :
   ```python
   self.robot = ReachyMini(...)  # PAS de with statement
   ```
   
   **RÉSULTAT ATTENDU :**
   Liste des instanciations sans context manager

4. **Validation des entrées - INSTRUCTIONS PRÉCISES**

   **Action 2B.4 : Chercher les validations manquantes**
   
   **INSTRUCTION EXACTE :**
   1. Ouvre `src/bbia_sim/backends/reachy_mini_backend.py`
   2. Cherche les fonctions publiques (pas `_private`)
   3. Pour chaque fonction, vérifie :
      - Y a-t-il une validation des paramètres ?
      - Y a-t-il une gestion de `None` ?
      - Y a-t-il une vérification de type ?
   
   **RÉSULTAT ATTENDU :**
   Liste des fonctions sans validation

5. **Thread safety et concurrence**
   - Cherche toutes les utilisations de `threading.Thread`, `asyncio.create_task`
   - Identifie les race conditions potentielles (variables partagées sans lock)
   - Détecte les deadlocks potentiels (plusieurs locks acquis dans le mauvais ordre)
   - Vérifie si les daemon threads sont correctement gérés

6. **Validation des entrées**
   - Cherche toutes les fonctions qui acceptent des paramètres utilisateur
   - Identifie les validations manquantes (types, ranges, formats)
   - Détecte les cas où des valeurs None ne sont pas gérées
   - Vérifie les validations des joint names, positions, durées

7. **Logging et observabilité**
   - Cherche tous les `logger.` dans le code
   - Identifie les opérations critiques non loggées
   - Détecte les logs manquants dans les chemins d'erreur
   - Vérifie la cohérence des niveaux de log (debug, info, warning, error)

8. **Configuration et environnement**
   - Cherche les variables d'environnement utilisées
   - Identifie les configs hardcodées qui devraient être configurables
   - Détecte les chemins hardcodés (devraient être relatifs ou configurables)
   - Vérifie la gestion des chemins Windows vs Linux vs macOS

9. **Compatibilité Python versions**
   - Cherche les syntaxes Python 3.11+ (ex: `X | Y` au lieu de `Union[X, Y]`)
   - Identifie les incompatibilités avec Python 3.10
   - Détecte les imports conditionnels manquants pour différentes versions
   - Vérifie la compatibilité avec Python 3.12 et 3.13

10. **Noms de variables et fonctions**
    - Cherche les noms de variables non explicites (`x`, `tmp`, `data`, `obj`)
    - Identifie les fonctions avec des noms trompeurs
    - Détecte les incohérences de nommage (snake_case vs camelCase)
    - Vérifie la cohérence avec les conventions du SDK officiel

**Livrables attendus :**
- Top 30 des micro-problèmes identifiés (fichier | ligne | problème | impact)
- Liste des timeouts manquants ou inappropriés
- Liste des validations manquantes
- Liste des ressources non fermées
- Score de qualité micro-détails (/10)

**Fichiers à analyser en priorité :**
```
src/bbia_sim/backends/reachy_mini_backend.py          # Lignes 1-715
src/bbia_sim/daemon/bridge.py                         # Lignes 1-388
src/bbia_sim/robot_factory.py                         # Lignes 1-98
pyproject.toml                                         # Lignes 31-71
```

---

### 🧹 PHASE 3 : QUALITÉ DU CODE PYTHON (DEEP DIVE)

**Objectif :** Audit exhaustif de la qualité du code selon les standards industriels.

**Actions concrètes à effectuer :**

1. **Typing et annotations**
   - Pour chaque fichier Python, compte les fonctions sans `-> ReturnType`
   - Identifie les `Any` utilisés (cherche `: Any` ou `Any |`)
   - Vérifie la conformité PEP 484
   - Liste les fichiers avec typing < 50%

2. **Code smells et anti-patterns**
   - Recherche les fonctions trop longues (>50 lignes) - compte les lignes entre `def` et prochain `def`
   - Identifie les classes God Object (>500 lignes, beaucoup de méthodes)
   - Détecte les duplications de code (même logique répétée 3+ fois)
   - Liste les magic numbers non constantes (ex: `0.1`, `100`, `300` sans constante)

3. **Gestion des erreurs**
   - Cherche les blocs `try/except` vides (ligne `except:` suivie de `pass`)
   - Identifie les exceptions trop génériques (`except Exception:`)
   - Détecte les erreurs non loggées (pas de `logger.error()` dans `except`)
   - Liste les `pass` suspects (dans `except` ou `if`)

4. **Performance et optimisation**
   - Identifie les boucles inefficaces (boucles imbriquées, appels répétés)
   - Détecte les appels répétés évitables (ex: `self.robot.get_joint_pos()` appelé 10 fois)
   - Liste les allocations mémoire inutiles (listes créées dans des boucles)
   - Vérifie l'utilisation de `@lru_cache` où approprié (fonctions pures non cachées)

5. **Conventions et style**
   - Vérifie PEP 8 (noms de variables, longueur de ligne)
   - Identifie les noms de variables non explicites (ex: `x`, `tmp`, `data`)
   - Détecte les docstrings manquantes ou incomplètes
   - Liste les commentaires obsolètes ou inutiles

6. **Imports et dépendances**
   - Pour chaque fichier, liste les imports inutilisés (fonction importée mais jamais appelée)
   - Vérifie les imports conditionnels manquants (ex: `try/except ImportError`)
   - Identifie les imports non triés
   - Détecte les star imports (`from x import *`)

**Livrables attendus :**
- Top 20 des fichiers avec le plus de problèmes (fichier | nb problèmes | types)
- Liste exhaustive des typing manquants (fichier | fonction | ligne)
- Rapport d'anti-patterns avec fichiers/lignes
- Score de qualité code (/10)

**Fichiers à analyser en priorité (les plus gros) :**
```
src/bbia_sim/dashboard_advanced.py                    # 3678 lignes - TRÈS GROS
src/bbia_sim/bbia_huggingface.py                      # 900 lignes
src/bbia_sim/backends/reachy_mini_backend.py          # 715 lignes
src/bbia_sim/bbia_vision.py                           # 520 lignes
src/bbia_sim/bbia_behavior.py                         # 518 lignes
```

---

### 🧪 PHASE 4 : TESTS ET COUVERTURE DE CODE

**Objectif :** Évaluation complète de la stratégie de tests et identification des zones non couvertes.

**Actions concrètes à effectuer :**

1. **Couverture actuelle**
   - Liste TOUS les fichiers dans `tests/` (voir liste ci-dessus)
   - Pour chaque module dans `src/bbia_sim/`, cherche s'il existe un test correspondant
   - Calcule la couverture théorique par module (fichier testé = 1, non testé = 0)
   - Liste les modules sans aucun test

2. **Qualité des tests existants**
   - Ouvre quelques tests représentatifs (ex: `test_reachy_mini_backend.py`)
   - Vérifie la structure des tests (Arrange/Act/Assert)
   - Identifie les tests sans assertions (pas de `assert`)
   - Détecte les tests dupliqués (même test dans plusieurs fichiers)

3. **Tests manquants prioritaires**
   - Identifie les modules critiques sans tests :
     - `backends/reachy_mini_backend.py` → `test_reachy_mini_backend.py` existe ?
     - `daemon/bridge.py` → `test_daemon_bridge.py` existe ?
     - `bbia_vision.py` → `test_bbia_vision.py` existe ?
   - Liste les cas limites non testés (ex: erreurs réseau, timeouts)

4. **Tests de performance et stress**
   - Cherche les tests avec `@pytest.mark.slow` ou `benchmark`
   - Identifie les tests de charge manquants
   - Détecte les tests de fuites mémoire

5. **Configuration pytest**
   - Ouvre `pyproject.toml` lignes 129-183 (section `[tool.pytest.ini_options]`)
   - Vérifie les markers définis
   - Identifie les configurations manquantes

**Livrables attendus :**
- Matrice de couverture par module (module | test existe | % estimé)
- Top 15 des modules prioritaires à tester
- Liste des tests unitaires manquants (descriptions précises)
- Score de couverture (/10)

**Fichiers à analyser :**
```
tests/test_reachy_mini_backend.py
tests/test_daemon_bridge.py
tests/test_bbia_vision.py
tests/test_dashboard_advanced.py
pyproject.toml                                         # Lignes 129-183
```

---

### 🎮 PHASE 5 : SIMULATION MUJOCO (ANALYSE PHYSIQUE)

**Objectif :** Audit approfondi de l'intégration MuJoCo et optimisation de la simulation physique.

**Actions concrètes à effectuer :**

1. **Configuration des modèles**
   - Ouvre `src/bbia_sim/sim/models/reachy_mini.xml`
   - Ouvre `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`
   - Compare les deux modèles (masses, inertie, joints)
   - Vérifie la cohérence avec les specs Reachy Mini officielles

2. **Performance de simulation**
   - Ouvre `src/bbia_sim/sim/simulator.py`
   - Cherche les boucles de simulation (fonctions `step()`, `render()`)
   - Identifie les calculs redondants (IK/FK calculés plusieurs fois)
   - Vérifie le timestep de simulation

3. **Physique et collisions**
   - Analyse les geoms dans les fichiers XML
   - Vérifie les propriétés de collision
   - Identifie les problèmes potentiels (pénétrations, instabilités)

4. **Rendu et visualisation**
   - Ouvre `src/bbia_sim/backends/mujoco_backend.py`
   - Cherche les fonctions de rendu (`render()`, `viewer()`)
   - Identifie les optimisations possibles

5. **Switch Simulation ↔ Robot Réel**
   - Analyse `src/bbia_sim/robot_factory.py` (logique de création backend)
   - Compare `mujoco_backend.py` vs `reachy_mini_backend.py` (cohérence API)

**Livrables attendus :**
- Rapport d'analyse du modèle MuJoCo
- Liste des optimisations de performance
- Identification des bugs physiques
- Score de qualité simulation (/10)

**Fichiers à analyser :**
```
src/bbia_sim/sim/simulator.py
src/bbia_sim/sim/models/reachy_mini.xml
src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml
src/bbia_sim/backends/mujoco_backend.py
src/bbia_sim/robot_factory.py
```

---

### 👁️ PHASE 6 : VISION ET INTELLIGENCE ARTIFICIELLE

**Objectif :** Audit complet des modules vision, IA et traitement temps réel.

**Actions concrètes à effectuer :**

1. **Architecture vision**
   - Ouvre `src/bbia_sim/bbia_vision.py` (520 lignes)
   - Ouvre `src/bbia_sim/vision_yolo.py` (186 lignes)
   - Ouvre `src/bbia_sim/face_recognition.py` (139 lignes)
   - Analyse les pipelines de traitement d'image
   - Identifie les goulots d'étranglement

2. **Modèles IA et versions**
   - Ouvre `src/bbia_sim/bbia_huggingface.py` (900 lignes)
   - Cherche tous les modèles Hugging Face utilisés (lignes avec `from_pretrained`)
   - Vérifie les versions des modèles (obsolètes ?)
   - Compare avec les modèles SOTA 2025

3. **Performance temps réel**
   - Cherche les boucles de traitement vidéo (fonctions avec `while True` ou `for frame in`)
   - Identifie les latences inacceptables (>100ms)
   - Détecte les blocages ou freeze

4. **Intégration caméra Reachy**
   - Ouvre `src/bbia_sim/backends/reachy_mini_backend.py`
   - Cherche la gestion des caméras (fonctions `get_image()`, `camera`)
   - Vérifie la synchronisation stéréo

5. **Modules Hugging Face**
   - Analyse `src/bbia_sim/bbia_huggingface.py` ligne par ligne
   - Cherche le système d'auto-unload (RAM) - fonctions `unload_model()`, `_cleanup()`
   - Identifie les fuites mémoire (modèles non libérés)

6. **Dépendances IA obsolètes**
   - Ouvre `pyproject.toml` lignes 62-70 (dépendances IA)
   - Compare versions avec les dernières releases 2025
   - Identifie les breaking changes non gérés

**Livrables attendus :**
- Audit des modèles IA avec versions actuelles vs recommandées
- Rapport de performance vision (FPS, latence, RAM)
- Liste des optimisations critiques
- Score de qualité vision/IA (/10)

**Fichiers à analyser :**
```
src/bbia_sim/bbia_vision.py
src/bbia_sim/vision_yolo.py
src/bbia_sim/face_recognition.py
src/bbia_sim/bbia_huggingface.py                       # 900 lignes - TRÈS GROS
src/bbia_sim/backends/reachy_mini_backend.py          # Section caméra
pyproject.toml                                         # Lignes 62-70
```

---

### 📡 PHASE 7 : COMMUNICATION ET ARCHITECTURE DISTRIBUÉE

**Objectif :** Analyse approfondie de la communication Zenoh, API REST et WebSocket.

**Actions concrètes à effectuer :**

1. **Bridge Zenoh**
   - Ouvre `src/bbia_sim/daemon/bridge.py` (388 lignes) - LIGNE PAR LIGNE
   - Cherche toutes les utilisations de `zenoh` (imports, sessions, publishers, subscribers)
   - Vérifie la gestion des topics Zenoh (lignes avec `pub` ou `sub`)
   - Identifie les problèmes de reconnexion (gestion d'erreurs réseau)
   - Détecte les timeouts ou deadlocks

2. **API REST (FastAPI)**
   - Ouvre `src/bbia_sim/daemon/app/main.py` (91 lignes)
   - Ouvre tous les routers dans `src/bbia_sim/daemon/app/routers/`
   - Vérifie la sécurité des endpoints (validation, auth)
   - Identifie les routes non documentées
   - Détecte les problèmes CORS ou auth

3. **WebSocket temps réel**
   - Ouvre `src/bbia_sim/dashboard_advanced.py` (3678 lignes)
   - Cherche les classes `BBIAAdvancedWebSocketManager` (lignes ~49-460)
   - Vérifie le système de cleanup WebSocket (fonctions `disconnect()`, `_cleanup_inactive_connections()`)
   - Identifie les fuites de connexions (connexions non fermées)
   - Détecte les problèmes de broadcast

4. **Gestion des connexions**
   - Cherche les pools de connexions (listes `active_connections`, `_connection_last_activity`)
   - Identifie les connexions non fermées (pas de `close()` ou `disconnect()`)
   - Détecte les problèmes de thread safety (utilisation de `threading.Lock()`)

**Livrables attendus :**
- Diagramme de communication (Zenoh/REST/WS)
- Liste des vulnérabilités de sécurité
- Rapport de robustesse réseau
- Score de qualité communication (/10)

**Fichiers à analyser :**
```
src/bbia_sim/daemon/bridge.py                         # 388 lignes - CRITIQUE
src/bbia_sim/daemon/app/main.py
src/bbia_sim/daemon/app/routers/*.py                  # Tous les routers
src/bbia_sim/dashboard_advanced.py                     # Lignes 49-460 (WebSocket)
src/bbia_sim/daemon/ws/telemetry.py
```

---

### 🚀 PHASE 8 : PERFORMANCE ET OPTIMISATION RAM/CPU

**Objectif :** Profiling approfondi et identification des optimisations critiques.

**Actions concrètes à effectuer :**

1. **Consommation RAM**
   - Cherche les modules gourmands en mémoire (listes, dictionnaires volumineux)
   - Analyse les caches non libérés (objets en mémoire non supprimés)
   - Détecte les fuites mémoire (objets non garbage collected)
   - Vérifie l'utilisation de `deque` vs `list` pour buffers (cherche `deque` et `list`)

2. **Utilisation CPU**
   - Identifie les boucles bloquantes (boucles `while True` sans `await` ou `sleep`)
   - Détecte les calculs synchrones qui devraient être async (fonctions lourdes sans `async`)
   - Liste les opérations non vectorisées (boucles Python au lieu de NumPy)

3. **Optimisations existantes**
   - Cherche l'utilisation de `@lru_cache` et `@cache` (décorateurs)
   - Analyse les singletons (pattern correctement implémenté ?)
   - Identifie les lazy loading manquants (imports au début vs à la demande)

4. **Concurrence et parallélisme**
   - Cherche l'utilisation de `threading` vs `asyncio` (imports)
   - Identifie les daemon threads non gérés (threads sans `daemon=True`)
   - Détecte les deadlocks potentiels (plusieurs locks acquis)

5. **Garbage Collection**
   - Identifie les références circulaires (objets qui se référencent mutuellement)
   - Détecte les objets immortels (références globales)
   - Vérifie les contextes managers (`with`) manquants (ressources non fermées)

**Livrables attendus :**
- Top 10 des fichiers gourmands en RAM/CPU
- Liste des optimisations de performance (avec impact estimé)
- Rapport de profiling théorique
- Score de performance (/10)

**Fichiers à analyser (les plus gros) :**
```
src/bbia_sim/dashboard_advanced.py                    # 3678 lignes
src/bbia_sim/bbia_huggingface.py                      # 900 lignes
src/bbia_sim/backends/reachy_mini_backend.py          # 715 lignes
src/bbia_sim/bbia_vision.py                           # 520 lignes
src/bbia_sim/bbia_behavior.py                         # 518 lignes
```

---

### 📚 PHASE 9 : DOCUMENTATION ET MAINTENABILITÉ

**Objectif :** Évaluation exhaustive de la documentation et de la maintenabilité du projet.

**Actions concrètes à effectuer :**

1. **Documentation code (docstrings)**
   - Pour chaque fichier Python, compte les fonctions sans docstring
   - Vérifie la conformité format (Google, NumPy, Sphinx ?)
   - Identifie les docstrings obsolètes ou incorrectes

2. **README et guides**
   - Ouvre `README.md`
   - Ouvre `docs/README.md`
   - Vérifie tous les fichiers dans `docs/`
   - Identifie les liens cassés ou obsolètes

3. **Documentation technique**
   - Ouvre `docs/development/architecture/ARCHITECTURE_OVERVIEW.md`
   - Vérifie la cohérence avec le code actuel
   - Identifie les diagrammes obsolètes

4. **Commentaires dans le code**
   - Identifie les sections complexes sans commentaires
   - Détecte les commentaires obsolètes ou trompeurs
   - Liste les `TODO`, `FIXME`, `HACK` non résolus (cherche ces mots-clés)

5. **Audits et rapports existants**
   - Liste TOUS les fichiers dans `docs/quality/audits/`
   - Vérifie leur pertinence et actualité

6. **Changelog et versioning**
   - Ouvre `CHANGELOG.md` (existe-t-il ?)
   - Vérifie la cohérence des versions (`pyproject.toml` ligne 7)

**Livrables attendus :**
- Liste exhaustive des docstrings manquantes (fichier/fonction)
- Rapport de qualité documentation (complétude %)
- Liste des TODO/FIXME à traiter
- Score de documentation (/10)

**Fichiers à analyser :**
```
README.md
CHANGELOG.md
docs/development/architecture/ARCHITECTURE_OVERVIEW.md
docs/quality/compliance/CONFORMITE_REACHY_MINI_COMPLETE.md
```

---

### 🔧 PHASE 10 : CI/CD, DÉPENDANCES ET SÉCURITÉ

**Objectif :** Audit des workflows CI/CD, gestion des dépendances et sécurité.

**Actions concrètes à effectuer :**

1. **Configuration CI/CD**
   - Cherche les workflows dans `.github/workflows/` (s'il existe)
   - Vérifie les jobs de tests, linting, déploiement

2. **Gestion des dépendances**
   - Ouvre `pyproject.toml` ligne par ligne
   - Compare versions avec les dernières releases 2025
   - Identifie les dépendances non épinglées (`>=` vs `==`)
   - Détecte les conflits de versions

3. **Dépendances obsolètes**
   - Liste les packages avec des versions obsolètes
   - Identifie les breaking changes dans les nouvelles versions
   - Détecte les dépréciations (warnings)

4. **Dépendances inutilisées**
   - Pour chaque package dans `pyproject.toml`, cherche s'il est importé dans le code
   - Identifie les dépendances dev vs prod mal classées

5. **Configuration Python**
   - Analyse `pyproject.toml` sections `[tool.*]`
   - Vérifie configuration `ruff`, `mypy`, `pytest`

6. **Sécurité**
   - Recherche les secrets hardcodés (tokens, passwords) - cherche `password=`, `token=`, `api_key=`
   - Identifie les imports dangereux (`pickle`, `eval`)

**Livrables attendus :**
- Rapport de sécurité avec vulnérabilités identifiées
- Liste des dépendances à mettre à jour (avec priorité)
- Analyse des workflows CI/CD (efficacité)
- Score de qualité CI/CD (/10)

**Fichiers à analyser :**
```
pyproject.toml                                         # TOUT LE FICHIER
.github/workflows/*.yml                                # Si existe
```

---

## 🎯 LIVRABLES FINAUX (SYNTHÈSE GLOBALE)

À la fin de l'audit complet (10 phases), fournis :

### 1. 📊 TABLEAU DE BORD EXÉCUTIF

```
┌─────────────────────────────────────────────────┐
│     AUDIT BBIA-SIM - SYNTHÈSE GLOBALE          │
├─────────────────────────────────────────────────┤
│ Phase 1  - Architecture          : X/10  ✅/⚠️/❌│
│ Phase 2  - Compatibilité SDK     : X/10  ✅/⚠️/❌│
│ Phase 3  - Qualité Code          : X/10  ✅/⚠️/❌│
│ Phase 4  - Tests/Couverture      : X/10  ✅/⚠️/❌│
│ Phase 5  - Simulation MuJoCo     : X/10  ✅/⚠️/❌│
│ Phase 6  - Vision/IA             : X/10  ✅/⚠️/❌│
│ Phase 7  - Communication         : X/10  ✅/⚠️/❌│
│ Phase 8  - Performance           : X/10  ✅/⚠️/❌│
│ Phase 9  - Documentation           : X/10  ✅/⚠️/❌│
│ Phase 10 - CI/CD/Sécurité        : X/10  ✅/⚠️/❌│
├─────────────────────────────────────────────────┤
│ SCORE GLOBAL                     : XX/100       │
└─────────────────────────────────────────────────┘
```

### 2. 🔥 TOP 20 PROBLÈMES CRITIQUES (PAR PRIORITÉ)

Liste numérotée avec :
- **Priorité** : 🔴 Critique | 🟠 Haute | 🟡 Moyenne
- **Catégorie** : Architecture | Compatibilité | Performance | Sécurité | etc.
- **Description** : Problème détaillé
- **Impact** : Conséquences si non résolu
- **Fichiers concernés** : Chemins exacts
- **Effort estimé** : Heures de travail

### 3. ✅ TOP 15 POINTS FORTS DU PROJET

Liste des éléments bien implémentés

### 4. 🚀 QUICK WINS (< 1 HEURE CHACUN)

Liste de 10-15 améliorations rapides

### 5. 🗺️ ROADMAP RECOMMANDÉE (3 MOIS)

Stratégie de correction par phases

### 6. 📋 FICHIERS À RETRAVAILLER (LISTE EXHAUSTIVE)

Tableau avec colonnes :
- **Fichier** : Chemin complet
- **Problèmes identifiés** : Liste numérotée
- **Priorité** : 1 (Critique) à 5 (Cosmétique)
- **Effort estimé** : Heures

### 7. 🧪 TESTS MANQUANTS CRITIQUES

Liste détaillée de 20-30 tests à créer

### 8. 📦 DÉPENDANCES OBSOLÈTES OU PROBLÉMATIQUES

Tableau avec versions actuelles vs recommandées

### 9. 🔍 DOUBLONS ET REDONDANCES IDENTIFIÉS

Liste exhaustive

### 10. 💡 IDÉES D'AMÉLIORATION (INNOVATION)

Suggestions de nouvelles fonctionnalités

---

## 🎨 FORMAT DE RAPPORT ATTENDU

**Pour chaque phase :**

```markdown
## PHASE X : [TITRE]

### ✅ Points forts identifiés
1. Point fort 1 avec justification
2. Point fort 2 avec justification

### ❌ Problèmes critiques
| Priorité | Fichier | Ligne | Description | Impact |
|----------|---------|-------|-------------|--------|
| 🔴       | path    | L123  | Problème    | Élevé  |

### 🔧 Améliorations recommandées
| Priorité | Action | Fichiers concernés | Effort | Impact |
|----------|--------|-------------------|--------|--------|

### 💡 Observations et insights
- Observation 1 avec analyse

### 📊 Score de la phase : X/10
Justification du score avec détails.
```

---

## ⚙️ MÉTHODOLOGIE D'ANALYSE

**Tu dois :**

1. ✅ Analyser TOUS les fichiers Python listés ci-dessus
2. ✅ Comparer systématiquement avec le repo officiel Reachy Mini
3. ✅ Lire TOUS les fichiers de documentation dans `docs/`
4. ✅ Analyser TOUS les tests dans `tests/`
5. ✅ Examiner `pyproject.toml` ligne par ligne

**Tu ne dois PAS :**

- ❌ Modifier aucun fichier
- ❌ Créer aucun fichier
- ❌ Proposer des correctifs de code (seulement identifier les problèmes)
- ❌ Exécuter du code (analyse statique uniquement)

---

## 🚀 DÉBUT DE L'AUDIT

**Commence immédiatement par la Phase 1 : Architecture et Structure.**

Analyse chaque fichier méthodiquement, prends des notes détaillées, et fournis un rapport exhaustif pour chaque phase.

**ORDRE RECOMMANDÉ D'ANALYSE :**

1. **Phase 1** : Architecture (fichiers `__init__.py` et structure)
2. **Phase 2** : Compatibilité SDK (fichiers backends)
3. **Phase 3** : Qualité code (fichiers les plus gros d'abord)
4. **Phase 4** : Tests (comparer tests vs modules)
5. **Phase 5** : Simulation (fichiers MuJoCo)
6. **Phase 6** : Vision/IA (modules IA)
7. **Phase 7** : Communication (bridge, routers, WebSocket)
8. **Phase 8** : Performance (analyse mémoire/CPU)
9. **Phase 9** : Documentation (README, docs/)
10. **Phase 10** : CI/CD (pyproject.toml)

**Prends tout le temps nécessaire pour un audit exhaustif et approfondi.**

---

## 🎯 INSTRUCTIONS SPÉCIFIQUES POUR WINDSURF

### 🔍 COMMENT CHERCHER EFFICACEMENT

**1. Utilise la recherche sémantique :**
- Pour chaque phase, pose des questions précises à Windsurf
- Exemple : "Cherche tous les usages de `ReachyMini(` dans le code"
- Exemple : "Trouve tous les `try/except` qui capturent `Exception` sans logger"

**2. Analyse ligne par ligne pour les fichiers critiques :**
- `src/bbia_sim/backends/reachy_mini_backend.py` (715 lignes) - LIRE TOUT
- `src/bbia_sim/daemon/bridge.py` (388 lignes) - LIRE TOUT
- `src/bbia_sim/dashboard_advanced.py` (3678 lignes) - ANALYSER PAR SECTIONS

**3. Compare systématiquement avec le repo officiel :**
- Pour chaque fonctionnalité, cherche l'équivalent dans le repo officiel
- Utilise les exemples du README officiel comme référence
- Compare les signatures de fonctions, les paramètres, les valeurs par défaut

**4. Cherche les patterns spécifiques :**
- `localhost_only` : sécurité réseau
- `timeout=` : gestion des timeouts
- `with ReachyMini()` : context managers
- `create_head_pose` : utils officiels
- `goto_target` : API de mouvement
- `zenoh` : communication middleware

**5. Identifie les incohérences :**
- Compare `mujoco_backend.py` vs `reachy_mini_backend.py` (même interface ?)
- Compare les endpoints REST avec l'API officielle
- Compare les arguments CLI avec le daemon officiel

### 📋 CHECKLIST DE VÉRIFICATIONS PAR FICHIER CRITIQUE

**Pour `src/bbia_sim/backends/reachy_mini_backend.py` :**
- [ ] Utilise-t-il `from reachy_mini import ReachyMini` ?
- [ ] Utilise-t-il `from reachy_mini.utils import create_head_pose` ?
- [ ] Utilise-t-il le context manager `with ReachyMini()` ?
- [ ] Gère-t-il `localhost_only` ?
- [ ] Gère-t-il les timeouts ?
- [ ] Gère-t-il les erreurs de connexion ?
- [ ] A-t-il les mêmes méthodes que `mujoco_backend.py` ?

**Pour `src/bbia_sim/daemon/bridge.py` :**
- [ ] Utilise-t-il `zenoh` correctement ?
- [ ] Gère-t-il la reconnexion automatique ?
- [ ] A-t-il des timeouts définis ?
- [ ] Logge-t-il toutes les erreurs ?
- [ ] Ferme-t-il les sessions proprement ?

**Pour `src/bbia_sim/daemon/app/main.py` :**
- [ ] A-t-il les mêmes endpoints que l'API officielle ?
- [ ] Le dashboard est-il accessible à `/` ?
- [ ] La documentation OpenAPI est-elle à `/docs` ?
- [ ] Gère-t-il CORS correctement ?

**Pour `pyproject.toml` :**
- [ ] Les versions de dépendances sont-elles compatibles avec l'officiel ?
- [ ] Y a-t-il un entry point CLI (`[project.scripts]`) ?
- [ ] Y a-t-il des extras définis (`[project.optional-dependencies]`) ?
- [ ] La version Python est-elle compatible (3.10-3.13) ?

### 🎯 QUESTIONS SPÉCIFIQUES À POSER À WINDSURF

**Pour la Phase 2 (Compatibilité SDK) :**
1. "Compare chaque dépendance dans `pyproject.toml` avec le repo officiel `pollen-robotics/reachy_mini`"
2. "Cherche tous les usages de `ReachyMini` dans le code et compare avec l'exemple officiel"
3. "Vérifie si BBIA utilise `create_head_pose` du SDK officiel"
4. "Compare les arguments CLI du daemon BBIA avec le daemon officiel"
5. "Vérifie si les endpoints REST BBIA sont compatibles avec l'API officielle"

**Pour la Phase 2B (Micro-détails) :**
1. "Cherche tous les `try/except` qui capturent `Exception` sans logger l'erreur"
2. "Trouve tous les `time.sleep()` et `timeout=` dans le code"
3. "Identifie tous les context managers manquants (`with` statements)"
4. "Cherche toutes les validations manquantes dans les fonctions publiques"
5. "Trouve tous les noms de variables non explicites (`x`, `tmp`, `data`)"

**Pour la Phase 3 (Qualité code) :**
1. "Liste tous les fichiers avec typing < 50%"
2. "Trouve toutes les fonctions > 50 lignes"
3. "Identifie tous les `Any` utilisés dans les type hints"
4. "Cherche tous les imports inutilisés"
5. "Trouve tous les magic numbers non constantes"

**Pour la Phase 7 (Communication) :**
1. "Analyse ligne par ligne `daemon/bridge.py` pour les problèmes Zenoh"
2. "Vérifie tous les endpoints REST dans `daemon/app/routers/`"
3. "Cherche les fuites de connexions WebSocket dans `dashboard_advanced.py`"
4. "Identifie les problèmes de thread safety dans le code asynchrone"

### 💡 ASTUCES POUR TROUVER DES PROBLÈMES CACHÉS

1. **Cherche les patterns anti-patterns :**
   - `except: pass` (erreurs silencieuses)
   - `time.sleep()` dans du code async (blocage)
   - Variables globales modifiables (race conditions)
   - `eval()` ou `exec()` (sécurité)

2. **Compare avec les exemples officiels :**
   - Le README officiel donne des exemples d'usage
   - Compare le code BBIA avec ces exemples
   - Identifie les différences (bugs potentiels)

3. **Vérifie la cohérence :**
   - Même fonctionnalité implémentée différemment dans 2 fichiers ?
   - Même constante définie à 2 endroits avec des valeurs différentes ?
   - Même logique répétée 3+ fois (DRY violation) ?

4. **Cherche les cas limites :**
   - Que se passe-t-il si `robot` est `None` ?
   - Que se passe-t-il si la connexion réseau est perdue ?
   - Que se passe-t-il si un timeout est dépassé ?
   - Que se passe-t-il si une valeur est hors limites ?

### 🚨 SIGNAUX D'ALERTE À IDENTIFIER

**🔴 Critique (à signaler immédiatement) :**
- Exceptions silencieuses (pas de log, pas de raise)
- Ressources non fermées (fuites mémoire)
- Race conditions (variables partagées sans lock)
- Secrets hardcodés (tokens, passwords)
- Timeouts manquants (opérations bloquantes)

**🟠 Haute priorité :**
- API incompatibles avec le SDK officiel
- Validations manquantes (sécurité)
- Code dupliqué (maintenance)
- Typing manquant (bugs potentiels)
- Documentation obsolète (confusion)

**🟡 Moyenne priorité :**
- Noms de variables non explicites
- Magic numbers
- Commentaires obsolètes
- Imports inutilisés
- Code mort (non utilisé)

---

## 📊 RAPPORT FINAL ATTENDU

À la fin de l'audit, Windsurf doit fournir :

1. **Un rapport exhaustif par phase** (10 phases + Phase 2B)
2. **Un tableau de bord exécutif** avec scores
3. **Top 20 problèmes critiques** avec priorités
4. **Top 30 micro-problèmes** identifiés
5. **Liste exhaustive des fichiers à retravailler**
6. **Roadmap de correction** par priorité

**Format du rapport :** Markdown structuré avec tableaux, listes, et exemples de code.

**Prends tout le temps nécessaire - un audit exhaustif peut prendre plusieurs heures.**

---

## 🎯 FORMAT DE RÉPONSE ATTENDU

Pour chaque action, fournis :

1. **Résultat concret** : Liste, tableau, ou nombre exact
2. **Exemples de code** : Extrait du code réel trouvé
3. **Problèmes identifiés** : Avec fichier et ligne exacte
4. **Score** : X/10 pour la phase

**EXEMPLE DE RÉPONSE :**

```
## Action 2.1 : Utilisation de ReachyMini

**Résultat :**
- 3 occurrences de `ReachyMini(` trouvées
- Toutes dans `src/bbia_sim/backends/reachy_mini_backend.py`

**Détails :**
| Ligne | Code | Paramètres | Conforme ? |
|-------|------|------------|------------|
| 204   | `ReachyMini(localhost_only=..., timeout=...)` | localhost_only, timeout, use_sim | ✅ OUI |
| 132   | `self.robot: ReachyMini | None` | Type hint | ✅ OUI |

**Problèmes :**
- Aucun usage de `with ReachyMini()` trouvé - ressource peut ne pas être fermée proprement

**Score : 7/10**
```

---

## 🚀 COMMENCE PAR LA PHASE 1

**Action immédiate :**
1. Ouvre `src/bbia_sim/backends/reachy_mini_backend.py`
2. Lis les lignes 14-27
3. Liste tous les imports trouvés
4. Identifie les imports relatifs

**Rapporte le résultat AVANT de continuer.**

