# Exemples BBIA-SIM

Ce dossier contient des exemples pratiques pour utiliser BBIA-SIM.

## 📊 **Résumé des Examples** (22 Nov. 2025)

- **✅ Examples utiles** : **44 exemples fonctionnels et maintenus** (39 existants + 5 nouveaux créés 22 Nov. 2025)
- **⚠️ Examples à améliorer** : ~5 exemples à consolider
- **❌ Examples inutiles** : ~4 exemples à archiver
- **🚀 Examples prioritaires** : ~10 exemples essentiels
- **📈 Exploitation capacités** : Complète (tous les comportements, endpoints et modules ont des exemples)

### 🎯 **Examples Prioritaires**

**Immédiat - Démos Principales :**
1. `demo_mujoco_amelioree.py` ⭐ - Meilleure démo 3D (RECOMMANDÉ)
2. `demo_chat_bbia_3d.py` - Chat + 3D interactif
3. `demo_emotion_ok.py` - Émotions robot

**Formation - SDK Officiel :**
1. `reachy_mini/minimal_demo.py` - Point de départ
2. `reachy_mini/sequence.py` - Mouvements complexes
3. `reachy_mini/look_at_image.py` - Intégration vision

**Utilitaires - API :**
1. `goto_pose.py` - Contrôle API REST
2. `subscribe_telemetry.py` - WebSocket temps réel
3. `hello_sim.py` - Test conformité

### ⚠️ **Examples à Consolider**

- **`demo_chat_bbia.py`**, **`demo_chat_simple.py`**, **`demo_chat_with_tools.py`** → Fusionner en `demo_chat_bbia_3d.py`
- **`demo_reachy_mini_corrigee.py`** → Utiliser `reachy_mini/*` officiels à la place

### ❌ **Examples Obsolètes (Archivés)**

- `demo_chat_simple.py` → Remplacé par `demo_chat_bbia_3d.py` (marqué déprécié)
- `demo_chat_with_tools.py` → Fusionner dans version 3D (marqué déprécié)
- `demo_reachy_mini_corrigee.py` → Utiliser `reachy_mini/` officiels (marqué déprécié)
- `demo_bbia_phase2_integration.py` → Test intégration, archivé dans `examples/_archived/` ✅
- `surprise_3d_mujoco_viewer.py` → Démo spécifique, archivé dans `examples/_archived/` ✅

## 📚 Exemples Reachy Mini (SDK Officiel)

Les exemples dans `reachy_mini/` sont adaptés du repo officiel `pollen-robotics/reachy_mini` :

- `minimal_demo.py` - Demo minimale (mouvements tête + antennes)
- `look_at_image.py` - Vision interactive (cliquer pour regarder)
- `sequence.py` - Séquences de mouvements animés
- `recorded_moves_example.py` - Jouer mouvements enregistrés
- `goto_interpolation_playground.py` - Découvrir méthodes d'interpolation

📖 **Voir** : [`reachy_mini/README.md`](reachy_mini/README.md) pour détails complets

---

## Scripts disponibles

### `hello_sim.py` - Test Conformité SDK Officiel

Test complet de la conformité avec le SDK officiel Reachy-Mini.

```bash
python examples/hello_sim.py
```

**Résultat attendu** : Test de toutes les méthodes SDK officiel, conformité validée

### `demo_mujoco_continue.py` - Simulation MuJoCo Continue ⭐ **Source des vidéos**

Simulation MuJoCo continue avec contrôle temps réel. Le robot bouge en continu (tête + corps).

```bash
# Mode graphique (voir 3D) - RECOMMANDÉ
mjpython examples/demo_mujoco_continue.py

# Mode headless (test)
python examples/demo_mujoco_continue.py --duration 10 --headless
```

**Résultat attendu** : Simulation continue, robot animé (yaw_body + stewart joints), viewer MuJoCo ouvert  
**📹 Note** : Les vidéos disponibles dans `assets/videos/` ont été enregistrées depuis ce script.

### `demo_mujoco_amelioree.py` - Simulation MuJoCo Améliorée ⭐ **RECOMMANDÉ**

Version améliorée avec mouvements plus visibles et corrections des indices de joints. Le robot bouge de manière très visible avec des amplitudes augmentées.

```bash
# Mode graphique (voir 3D) - RECOMMANDÉ
mjpython examples/demo_mujoco_amelioree.py
```

**Résultat attendu** : Simulation avec mouvements très visibles (amplitude 0.3 rad), robot animé (yaw_body + stewart joints), viewer MuJoCo ouvert  
**✨ Améliorations** : 
- Mouvements plus visibles (amplitude augmentée)
- Correction des indices de joints (utilisation de `model.jnt_qposadr`)
- Meilleure synchronisation (ordre correct mj_forward/mj_step)
- Vérifications améliorées avec affichage des joints trouvés

### `goto_pose.py` - Contrôle mouvement robot

Contrôle une articulation du robot via l'API REST.

```bash
python examples/goto_pose.py --token dev --joint neck_yaw --pos 0.6
```

**Résultat attendu** : Position articulation changée, confirmation API

### `subscribe_telemetry.py` - Télémétrie WebSocket

S'abonne à la télémétrie temps réel via WebSocket.

```bash
python examples/subscribe_telemetry.py --token dev --count 5
```

**Résultat attendu** : 5 messages de télémétrie affichés

### `demo_emotion_ok.py` - Démo Émotion → Pose (RobotAPI)

Démo BBIA utilisant le backend unifié RobotAPI.

```bash
# Mode headless (test)
python examples/demo_emotion_ok.py --emotion happy --duration 5 --headless --backend mujoco

# Mode graphique (voir 3D)
mjpython examples/demo_emotion_ok.py --emotion happy --duration 10 --backend mujoco
```

**Résultat attendu** : Animation émotion → joint, backend unifié

### `demo_chat_bbia_3d.py` - Démo 3D Chat BBIA ⭐ **RECOMMANDÉ**

Démo 3D avec chat intelligent BBIA (version consolidée).

```bash
# Voir la 3D avec chat
mjpython examples/demo_chat_bbia_3d.py --duration 10
```

**Résultat attendu** : Chat intelligent avec robot 3D

**Note** : Cette version remplace `demo_chat_bbia.py`, `demo_chat_simple.py` et `demo_chat_with_tools.py`.

### `demo_voice_ok.py` - Démo Voix → Action

Démo BBIA Voix utilisant RobotAPI.

```bash
python examples/demo_voice_ok.py --command "regarde-moi" --duration 5 --headless --backend mujoco
```

**Résultat attendu** : Commande vocale → action robot

### `demo_vision_ok.py` - Démo Vision → Tracking

Démo BBIA Vision utilisant RobotAPI.

```bash
python examples/demo_vision_ok.py --target "virtual_target" --duration 5 --headless --backend mujoco
```

**Résultat attendu** : Tracking visuel → mouvement robot

### `demo_behavior_ok.py` - Démo Comportement → Scénario

Démo BBIA Comportement utilisant RobotAPI.

```bash
python examples/demo_behavior_ok.py --behavior "wake_up" --duration 5 --headless --backend mujoco
```

**Résultat attendu** : Comportement complexe → séquence d'actions

---

## 🎭 **Nouveaux Exemples - Comportements Avancés** (22 Nov. 2025)

### `demo_dance.py` - Danse synchronisée avec musique

Démonstration du comportement DanceBehavior avec différents types de musique.

```bash
python examples/demo_dance.py --music-type happy --duration 30 --backend mujoco
```

**Résultat attendu** : Danse synchronisée selon type musique

### `demo_emotion_show.py` - Démonstration des 12 émotions BBIA

Parcourt toutes les émotions avec transitions fluides et explications vocales.

```bash
python examples/demo_emotion_show.py --emotions happy sad excited --backend mujoco
```

**Résultat attendu** : Démonstration complète des émotions

### `demo_photo_booth.py` - Mode photo avec poses expressives

Démonstration du comportement PhotoBoothBehavior avec détection visage.

```bash
python examples/demo_photo_booth.py --pose happy --num-photos 3 --backend mujoco
```

**Résultat attendu** : Photos avec poses expressives

### `demo_storytelling.py` - Raconter histoires avec mouvements expressifs

Démonstration du comportement StorytellingBehavior avec histoires pré-enregistrées.

```bash
python examples/demo_storytelling.py --story petit_chaperon_rouge --interactive --backend mujoco
```

**Résultat attendu** : Narration histoire avec mouvements synchronisés

### `demo_teaching.py` - Mode éducatif interactif

Démonstration du comportement TeachingBehavior avec leçons et questions.

```bash
python examples/demo_teaching.py --subject maths --level beginner --backend mujoco
```

**Résultat attendu** : Leçon interactive avec questions/réponses

### `demo_meditation.py` - Guide méditation avec mouvements lents

Démonstration du comportement MeditationBehavior avec guidage vocal.

```bash
python examples/demo_meditation.py --duration 5 --backend mujoco
```

**Résultat attendu** : Séance de méditation guidée

### `demo_exercise.py` - Guide exercices physiques

Démonstration du comportement ExerciseBehavior avec mouvements démonstratifs.

```bash
python examples/demo_exercise.py --exercise head_rotation --repetitions 5 --backend mujoco
```

**Résultat attendu** : Guide exercice avec démonstration

### `demo_music_reaction.py` - Réagir à la musique avec mouvements

Démonstration du comportement MusicReactionBehavior avec synchronisation rythme.

```bash
python examples/demo_music_reaction.py --genre pop --duration 30 --backend mujoco
```

**Résultat attendu** : Réaction musique avec mouvements synchronisés

### `demo_alarm_clock.py` - Réveil intelligent avec interactions

Démonstration du comportement AlarmClockBehavior avec séquence progressive.

```bash
python examples/demo_alarm_clock.py --hour 8 --minute 0 --snooze-minutes 5 --backend mujoco
```

**Résultat attendu** : Configuration réveil intelligent

### `demo_weather_report.py` - Rapport météo avec gestes expressifs

Démonstration du comportement WeatherReportBehavior avec mouvements selon conditions.

```bash
python examples/demo_weather_report.py --city Paris --backend mujoco
```

**Résultat attendu** : Rapport météo avec gestes expressifs

### `demo_news_reader.py` - Lecture actualités avec réactions

Démonstration du comportement NewsReaderBehavior avec réactions émotionnelles.

```bash
python examples/demo_news_reader.py --max-items 5 --backend mujoco
```

**Résultat attendu** : Lecture actualités avec réactions

### `demo_game.py` - Jeux interactifs avec réactions émotionnelles

Démonstration du comportement GameBehavior avec différents jeux.

```bash
python examples/demo_game.py --game rock_paper_scissors --rounds 3 --backend mujoco
```

**Résultat attendu** : Jeu interactif avec score

---

## 🌐 **Nouveaux Exemples - Endpoints API** (22 Nov. 2025)

### `demo_motors.py` - Contrôle des moteurs

Démonstration des endpoints `/api/motors/*` pour contrôler les moteurs.

```bash
python examples/demo_motors.py --token dev --mode enabled --url http://localhost:8000
```

**Résultat attendu** : Statut et contrôle moteurs

### `demo_daemon.py` - Contrôle du daemon

Démonstration des endpoints `/api/daemon/*` pour contrôler le daemon.

```bash
python examples/demo_daemon.py --action status --url http://localhost:8000
python examples/demo_daemon.py --action start --wake-up --url http://localhost:8000
```

**Résultat attendu** : Contrôle daemon (start/stop/restart/status)

### `demo_kinematics.py` - Informations cinématique

Démonstration des endpoints `/api/kinematics/*` pour la cinématique.

```bash
python examples/demo_kinematics.py --token dev --endpoint info --url http://localhost:8000
```

**Résultat attendu** : Informations cinématique (info/urdf/stl)

### `demo_media.py` - Contrôle audio/vidéo

Démonstration des endpoints `/api/media/*` pour contrôler audio/vidéo.

```bash
python examples/demo_media.py --action volume --volume 0.7 --url http://localhost:8000
python examples/demo_media.py --action camera --camera-enabled True --url http://localhost:8000
```

**Résultat attendu** : Contrôle volume et caméra

### `demo_apps.py` - Gestion applications HuggingFace

Démonstration des endpoints `/api/apps/*` pour gérer les apps HF.

```bash
python examples/demo_apps.py --token dev --action list --url http://localhost:8000
python examples/demo_apps.py --token dev --action status --app-name reachy-mini-conversation --url http://localhost:8000
```

**Résultat attendu** : Gestion apps (list/install/start/stop/status)

### `demo_metrics.py` - Métriques Prometheus

Démonstration des endpoints `/metrics/*` pour les métriques.

```bash
python examples/demo_metrics.py --endpoint health --url http://localhost:8000
python examples/demo_metrics.py --endpoint prometheus --url http://localhost:8000
```

**Résultat attendu** : Métriques Prometheus (healthz/readyz/health/prometheus)

### `demo_state_ws.py` - État complet via WebSocket

Démonstration du WebSocket `/api/state/ws/full` pour l'état complet.

```bash
python examples/demo_state_ws.py --count 5 --url ws://localhost:8000
```

**Résultat attendu** : Messages d'état via WebSocket

---

## 🧠 **Nouveaux Exemples - Modules Avancés** (22 Nov. 2025)

### `demo_emotion_recognition.py` - Reconnaissance émotions humaines

Démonstration du module de reconnaissance d'émotions faciales et vocales.

```bash
python examples/demo_emotion_recognition.py --mode facial --device auto
python examples/demo_emotion_recognition.py --mode vocal --device auto
python examples/demo_emotion_recognition.py --mode multimodal --device auto
```

**Résultat attendu** : Reconnaissance émotions (faciale/vocale/multimodale)

### `demo_integration.py` - Intégration complète BBIA ↔ Robot

Démonstration du module d'intégration complet connectant tous les modules BBIA.

```bash
python examples/demo_integration.py --action emotion --backend mujoco
python examples/demo_integration.py --action vision --backend mujoco
python examples/demo_integration.py --action voice --backend mujoco
python examples/demo_integration.py --action behavior --backend mujoco
```

**Résultat attendu** : Intégration complète (émotion/vision/voix/comportement)

### `demo_voice_advanced.py` - Synthèse vocale avancée

Démonstration du module de synthèse vocale avancée avec contrôle pitch/émotion.

```bash
python examples/demo_voice_advanced.py --text "Bonjour" --emotion happy --backend mujoco
python examples/demo_voice_advanced.py --text "Bonjour" --emotion sad --pitch -0.2 --backend mujoco
```

**Résultat attendu** : Synthèse vocale avec contrôle émotion et pitch

### `demo_follow_object.py` - Suivi d'objet avec priorisation intelligente

Démonstration du comportement FollowObjectBehavior avec détection YOLO.

```bash
python examples/demo_follow_object.py --target-object person --duration 10 --backend mujoco
python examples/demo_follow_object.py --duration 15 --backend mujoco
```

**Résultat attendu** : Suivi d'objet avec priorisation automatique

### `demo_memory.py` - Mémoire persistante BBIA

Démonstration du module mémoire pour sauvegarder et charger l'historique conversation, les préférences et les apprentissages.

```bash
python examples/demo_memory.py --action demo
python examples/demo_memory.py --action save
python examples/demo_memory.py --action load
```

**Résultat attendu** : Sauvegarde/chargement mémoire (conversation, préférences, apprentissages)

### `demo_adaptive_behavior.py` - Comportements adaptatifs contextuels

Démonstration du module de comportements adaptatifs qui génère des comportements dynamiques basés sur le contexte et l'émotion.

```bash
python examples/demo_adaptive_behavior.py --context greeting --emotion happy --duration 5 --backend mujoco
python examples/demo_adaptive_behavior.py --context conversation --emotion curious --duration 10 --backend mujoco
```

**Résultat attendu** : Comportement adaptatif généré et exécuté selon contexte/émotion

### `demo_awake.py` - Séquence de réveil optimisée

Démonstration de la séquence de réveil BBIA avec intelligence et variété.

```bash
python examples/demo_awake.py
```

**Résultat attendu** : Séquence de réveil avec messages variés

### `demo_touch_detection.py` - Détection tactile (Issue #251)

Démonstration de la détection d'interactions tactiles via analyse audio (tap, caress, pat).

```bash
python examples/demo_touch_detection.py
```

**Résultat attendu** : Détection tactile en temps réel (tap/caress/pat)

### `demo_sleeping_pose.py` - Pose de sommeil améliorée (Issue #410)

Démonstration de la pose de sommeil naturelle avec `set_sleeping_pose()`.

```bash
python examples/demo_sleeping_pose.py
```

**Résultat attendu** : Robot en position de sommeil naturelle

### `demo_collision_detection.py` - Détection collision (Issue #183)

Démonstration de la détection de collision dans la simulation MuJoCo.

```bash
python examples/demo_collision_detection.py
```

**Résultat attendu** : Vérification collision en simulation

**Note** : Disponible uniquement en simulation MuJoCo.

### `demo_robot_registry.py` - Registre multi-robots (Issue #30)

Démonstration du registre multi-robots pour gestion future de plusieurs robots.

```bash
python examples/demo_robot_registry.py
```

**Résultat attendu** : Informations registre robots et backends disponibles

**Note** : Infrastructure pour support multi-robots futur.

---

## 🔧 **Nouveaux Exemples - Endpoints API Complémentaires** (22 Nov. 2025)

### `demo_sanity.py` - Vérification statut et arrêt d'urgence

Démonstration des endpoints `/api/sanity/*` pour vérifier le statut et déclencher l'arrêt d'urgence.

```bash
python examples/demo_sanity.py --action status --url http://localhost:8000
python examples/demo_sanity.py --action emergency_stop --url http://localhost:8000
```

**Résultat attendu** : Vérification statut système et arrêt d'urgence

---

## 🎯 **Backend Unifié**

Toutes les démos supportent le backend unifié :

```bash
# Simulation MuJoCo
--backend mujoco

# Robot réel (mock)
--backend reachy
```

## 📊 **Métriques**

- **Tests** : 38 tests Reachy-Mini SDK officiel passent (100% conformité)
- **Coverage** : 100% des fonctionnalités SDK officiel
- **Performance** : <1ms latence en simulation
- **Conformité** : 21/21 méthodes SDK officiel implémentées

## Prérequis

1. **API démarrée** :

   ```bash
   BBIA_ENV=prod BBIA_TOKEN=dev uvicorn src.bbia_sim.daemon.app.main:app --port 8000
   ```

2. **Dépendances installées** :

   ```bash
   pip install httpx websockets
   ```

## Codes de sortie

- `0` : Succès
- `1` : Erreur (détails dans la sortie)

## Troubleshooting

- **Erreur connexion API** : Vérifier que l'API est démarrée sur le bon port
- **Token invalide** : Utiliser le même token que l'API (`dev` par défaut)
- **Timeout WebSocket** : Vérifier que la télémétrie est activée
