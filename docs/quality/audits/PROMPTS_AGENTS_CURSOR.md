# 🤖 PROMPTS POUR AGENTS CURSOR - Travail Parallèle

**Date** : Novembre 2024  
**Dernière mise à jour :** 19 Novembre 2025  
**Objectif** : Prompts optimisés pour agents Cursor travaillant simultanément sur l'évolution BBIA

---

## 📋 INSTRUCTIONS GÉNÉRALES (À inclure dans chaque prompt)

### ⚠️ VÉRIFICATION OBLIGATOIRE AVANT CRÉATION

**TOUJOURS vérifier que quelque chose n'existe pas déjà avant de le créer :**

1. ✅ **Fichiers** : Vérifier avec `glob_file_search` ou `list_dir` si le fichier existe déjà
2. ✅ **Fonctions/Classes** : Vérifier avec `grep` ou `codebase_search` si la fonction/classe existe déjà
3. ✅ **Tests** : Vérifier dans `tests/` si des tests similaires existent déjà
4. ✅ **Endpoints API** : Vérifier dans `src/bbia_sim/daemon/app/routers/` si l'endpoint existe déjà
5. ✅ **Imports** : Vérifier si les dépendances sont déjà dans `pyproject.toml`

**Si quelque chose existe déjà :**
- ✅ **Modifier** au lieu de créer
- ✅ **Améliorer** au lieu de dupliquer
- ✅ **Intégrer** au lieu de créer séparément
- ✅ **Documenter** les modifications apportées

### ✅ Qualité Code Obligatoire

- **Linting** : Exécuter `black`, `ruff`, `mypy` après chaque modification
- **Sécurité** : Exécuter `bandit` et corriger toutes les vulnérabilités
- **Tests** : Créer tests unitaires pour chaque nouvelle fonctionnalité
- **Documentation** : Ajouter docstrings type hints pour toutes les fonctions
- **Type Hints** : Utiliser `typing` pour tous les paramètres et retours

### ✅ Vérifications Finales

Quand une étape est terminée :
1. ✅ Exécuter `black .` et corriger
2. ✅ Exécuter `ruff check .` et corriger
3. ✅ Exécuter `mypy src/bbia_sim/` et corriger
4. ✅ Exécuter `bandit -r src/bbia_sim/` et corriger
5. ✅ Vérifier que tous les tests passent : `pytest tests/`
6. ✅ Vérifier imports : pas de doublons, pas d'imports inutilisés

### ✅ Références Documents

- **Plan principal** : `docs/quality/audits/PLAN_EVOLUTION_BBIA_COMPLET.md`
- **Résumé** : `docs/quality/audits/RESUME_PLANS_EVOLUTION.md`
- **Conformité SDK** : `docs/quality/audits/COMPATIBILITE_REACHY_MINI_OFFICIEL.md`

---

## 🤖 PROMPT 1 : AGENT INTELLIGENCE CONVERSATIONNELLE

```
Tu es un agent Cursor spécialisé dans l'intégration de LLM conversationnels.

🎯 MISSION :
Implémenter l'intégration d'un LLM conversationnel (Phi-2 ou TinyLlama) dans BBIA-SIM pour remplacer le système de règles basiques actuel.

📚 DOCUMENTS DE RÉFÉRENCE :
- Plan détaillé : docs/quality/audits/PLAN_INTELLIGENCE_CONVERSATIONNELLE.md
- Plan global : docs/quality/audits/PLAN_EVOLUTION_BBIA_COMPLET.md
- Code actuel : src/bbia_sim/bbia_huggingface.py (lignes 200-300 environ)

📋 TÂCHES À EFFECTUER :

⚠️ **IMPORTANT : Vérifier que chaque fichier/fonction n'existe pas déjà avant de créer !**

PHASE 1 : Intégration LLM de Base (Semaine 1-2)

1. **VÉRIFIER** si src/bbia_sim/bbia_chat.py existe déjà
   - Si oui : Modifier le fichier existant
   - Si non : Créer nouveau module : src/bbia_sim/bbia_chat.py
   - Classe BBIAChat avec chargement Phi-2 ou TinyLlama
   - Méthode _load_llm() avec fallback si échec
   - Méthode generate() pour génération réponse
   - Gestion mémoire optimisée (torch.float16, device_map="auto")

2. Modifier src/bbia_sim/bbia_huggingface.py
   - Importer BBIAChat
   - Remplacer méthode chat() pour utiliser LLM au lieu de règles
   - Garder analyse sentiment existante
   - Intégrer avec robot_api si disponible

3. Ajouter dépendances dans pyproject.toml
   - "accelerate>=0.20.0" (optimisation)
   - "bitsandbytes>=0.41.0" (quantification 8-bit, optionnel)
   - "sentencepiece>=0.1.99" (tokenisation)

4. Créer tests : tests/test_bbia_chat_llm.py
   - Test chargement modèle
   - Test génération réponse
   - Test mémoire RAM (<6GB)
   - Test latence (<2s)

PHASE 2 : Compréhension Contextuelle (Semaine 3-4)

5. Ajouter historique conversation dans BBIAChat
   - deque(maxlen=10) pour stocker 10 derniers messages
   - Méthode _build_context_prompt() pour inclure contexte dans prompt LLM
   - Format : "Utilisateur: ... BBIA: ..."

6. Implémenter détection actions robot
   - Méthode _detect_action() avec patterns regex
   - Actions : look_right, look_left, look_up, look_down, wake_up, sleep
   - Méthode _execute_action() pour exécuter via robot_api
   - Utiliser create_head_pose() du SDK officiel

7. Intégrer émotions BBIA
   - Méthode _extract_emotion() pour détecter émotions dans message
   - Méthode _apply_emotion() pour appliquer au robot
   - Utiliser BBIAEmotions existant

PHASE 3 : Personnalités Avancées (Semaine 5-6)

8. Système personnalités
   - 5 personnalités : friendly, professional, playful, calm, enthusiastic
   - Dictionnaire PERSONALITIES avec system_prompt et tone
   - Méthode set_personality() pour changer personnalité
   - Méthode _update_system_prompt()

9. Apprentissage préférences
   - Dictionnaire user_preferences
   - Méthode learn_preference() pour apprendre
   - Méthode _adapt_to_preferences() pour adapter réponses
   - Sauvegarde JSON des préférences

✅ QUALITÉ CODE OBLIGATOIRE :
- Exécuter black, ruff, mypy, bandit après chaque modification
- Ajouter docstrings complètes avec type hints
- Créer tests pour chaque nouvelle fonctionnalité
- Vérifier conformité SDK : utiliser create_head_pose() du SDK officiel
- Gestion erreurs : try/except pour chargement modèle, fallback gracieux

✅ SÉCURITÉ :
- Valider tous les inputs utilisateur
- Pas de code injection dans prompts LLM
- Limiter longueur prompts (max 2000 tokens)
- Sanitizer réponses LLM (pas de code exécutable)

✅ PERFORMANCE :
- Cache modèle (ne pas recharger à chaque appel)
- Limiter contexte historique (max 10 messages)
- Timeout génération (max 5s)
- Gestion mémoire (libérer GPU si disponible)

✅ QUAND UNE ÉTAPE EST TERMINÉE :
1. Exécuter : black . && ruff check . && mypy src/bbia_sim/ && bandit -r src/bbia_sim/
2. Exécuter : pytest tests/test_bbia_chat_llm.py -v
3. Vérifier : pas d'imports inutilisés, pas de doublons
4. Documenter : ajouter exemples dans docstrings

🚀 COMMENCE PAR : 
1. **VÉRIFIER** si src/bbia_sim/bbia_chat.py existe déjà (utiliser glob_file_search ou list_dir)
2. Si existe : Lire le fichier et améliorer/modifier
3. Si n'existe pas : Créer src/bbia_sim/bbia_chat.py avec classe BBIAChat basique et chargement Phi-2
```

---

## 🤖 PROMPT 2 : AGENT DASHBOARD MODERNE

```
Tu es un agent Cursor spécialisé dans le développement d'interfaces web modernes.

🎯 MISSION :
Améliorer le dashboard BBIA pour qu'il soit aussi épuré et moderne que celui des testeurs Reachy Mini, avec contrôles media visuels et vue 3D robot.

📚 DOCUMENTS DE RÉFÉRENCE :
- Comparaison : docs/dashboard/COMPARAISON_DASHBOARD_TESTEURS.md
- Plan global : docs/quality/audits/PLAN_EVOLUTION_BBIA_COMPLET.md (Section Plan 2)
- Dashboard actuel : src/bbia_sim/daemon/app/dashboard/
- Dashboard avancé : src/bbia_sim/dashboard_advanced.py

📋 TÂCHES À EFFECTUER :

⚠️ **IMPORTANT : Vérifier que chaque fichier/fonction n'existe pas déjà avant de créer !**

PHASE 1 : Contrôles Media Visuels ✅ **TERMINÉ** (19 Novembre 2025)

1. ✅ **FAIT** - src/bbia_sim/daemon/app/dashboard/templates/sections/media.html créé
   - ✅ Section SPEAKER : Built-in Speaker + slider volume + waveform
   - ✅ Section MICROPHONE : USB Microphone + slider volume + waveform
   - ✅ Section CAMERA : Toggle ON/OFF + statut
   - ✅ Design : Fond blanc, sections arrondies, icônes

2. ✅ **FAIT** - src/bbia_sim/daemon/app/dashboard/static/js/media.js créé
   - ✅ Gestion sliders volume (debounce 200ms)
   - ✅ Appels API : POST /development/api/media/speaker/volume
   - ✅ Appels API : POST /development/api/media/microphone/volume
   - ✅ Appels API : POST /development/api/media/camera/toggle

3. ✅ **FAIT** - src/bbia_sim/daemon/app/dashboard/static/js/waveform.js créé
   - ✅ Utiliser Web Audio API pour analyser audio
   - ✅ Canvas pour dessiner waveform
   - ✅ Animation temps réel (30 FPS pour performance)

4. ✅ **FAIT** - src/bbia_sim/daemon/app/routers/media.py créé
   - ✅ POST /development/api/media/speaker/volume (body: {"volume": 0.0-1.0})
   - ✅ POST /development/api/media/microphone/volume (body: {"volume": 0.0-1.0})
   - ✅ POST /development/api/media/camera/toggle (body: {"enabled": true/false})
   - ✅ GET /development/api/media/status (retourne statut media)

5. ✅ **FAIT** - Intégration dans index.html
   - ✅ Ajout {% include "sections/media.html" %} dans templates/index.html
   - ✅ Ajout <script src="/static/js/media.js"></script>
   - ✅ Ajout <script src="/static/js/waveform.js"></script>

**✅ STATUT COMPLET :**
- ✅ Tests unitaires : **FAIT** (`tests/test_dashboard_media.py` - 8 tests complets)
- ✅ Intégration robot réel : **TERMINÉ** (19 nov 2025) - `_get_robot_media()` implémenté avec support robot réel

PHASE 2 : Vue 3D Robot ✅ **TERMINÉ** (19 Novembre 2025)

**Implémenté :**
- ✅ Tests créés : `tests/test_dashboard_3d.py` existe (5 tests)
- ✅ Implémentation : `robot_3d.js` créé
- ✅ Three.js : Intégré dans base.html (CDN v0.160.0)

6. ✅ **FAIT** - Three.js installé
   - ✅ CDN Three.js ajouté dans base.html : <script src="https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.min.js"></script>

7. ✅ **FAIT** - Render 3D créé : src/bbia_sim/daemon/app/dashboard/static/js/robot_3d.js
   - ✅ Placeholder robot créé (géométrie basique)
   - ⚠️ Modèle STL : À charger ultérieurement (src/bbia_sim/sim/assets/reachy_official/*.stl)
   - ✅ Scène Three.js avec caméra et lumière
   - ✅ Animation selon état : running, stopped, error
   - ✅ Synchronisation avec daemon status (polling)

8. ✅ **FAIT** - Intégré dans daemon.html
   - ✅ Canvas 3D ajouté : <canvas id="robot-3d-canvas">
   - ✅ Script ajouté : <script src="/static/js/robot_3d.js"></script>

PHASE 3 : Design Épuré ✅ **TERMINÉ** (19 Novembre 2025)

**Implémenté :**
- ✅ Fond blanc : `bg-white` dans base.html
- ✅ Image floutée : SVG avec filter blur en arrière-plan
- ✅ Quick Actions : Section créée (quick_actions.html)
- ✅ Indicateurs FPS : fps_display.js créé

9. ✅ **FAIT** - Design général amélioré
   - ✅ Fond blanc (#ffffff) avec image floutée en arrière-plan (SVG blur)
   - ✅ Sections avec ombres légères (box-shadow via app-section)
   - ✅ Espacement cohérent (gap-4, padding-6)
   - ✅ Polices : Archivo (titre) + Asap (texte) - déjà présentes ✅

10. ✅ **FAIT** - Quick Actions en grille
    - ✅ 15 boutons emoji : 😊 😢 😕 😮 😠 🕶️ 🤔 👋 🙏 😴 🎉 🎭 🎨 🎪 🎬
    - ✅ Grid layout : grid-cols-5 (3 lignes)
    - ✅ Hover effects : scale-105, transition
    - ⚠️ Actions : Structure créée, intégration WebSocket à compléter

11. ✅ **FAIT** - Indicateurs FPS visibles
    - ✅ Afficher "60 FPS" en haut à droite (comme testeurs)
    - ✅ Mise à jour temps réel (requestAnimationFrame)
    - ✅ Couleur verte si ≥30 FPS, orange si <30 FPS

✅ QUALITÉ CODE OBLIGATOIRE :
- Exécuter black, ruff, mypy, bandit après chaque modification
- Valider tous les inputs utilisateur (volume 0.0-1.0, etc.)
- Gestion erreurs : try/except pour WebSocket, fallback gracieux
- Tests : créer tests/test_dashboard_media.py pour endpoints API

✅ SÉCURITÉ :
- Valider volumes (clamp 0.0-1.0)
- Sanitizer inputs utilisateur (pas de XSS)
- CORS configuré correctement
- Rate limiting sur endpoints API

✅ PERFORMANCE :
- WebSocket optimisé (pas de spam messages)
- Waveform : limiter fréquence mise à jour (30 FPS max)
- Three.js : optimiser rendu (frustum culling, LOD si nécessaire)
- Lazy loading : charger Three.js seulement si nécessaire

✅ QUAND UNE ÉTAPE EST TERMINÉE :
1. Exécuter : black . && ruff check . && mypy src/bbia_sim/ && bandit -r src/bbia_sim/
2. Exécuter : pytest tests/test_dashboard_media.py -v
3. Tester manuellement : lancer dashboard et vérifier contrôles media
4. Vérifier : pas d'erreurs console JavaScript, pas de warnings

🚀 COMMENCE PAR : 
1. **VÉRIFIER** si src/bbia_sim/daemon/app/dashboard/templates/sections/media.html existe déjà
2. **VÉRIFIER** si src/bbia_sim/daemon/app/routers/media.py existe déjà
3. Si existent : Lire et améliorer/modifier
4. Si n'existent pas : Créer avec structure basique (sections SPEAKER, MICROPHONE, CAMERA)
```

---

## 🤖 PROMPT 3 : AGENT COMPORTEMENTS AVANCÉS

```
Tu es un agent Cursor spécialisé dans la création de comportements robotiques intelligents.

🎯 MISSION :
Créer 15+ comportements avancés pour BBIA, utilisant l'IA (vision, voice, émotions) et l'expressivité BBIA (12 émotions).

📚 DOCUMENTS DE RÉFÉRENCE :
- Plan détaillé : docs/quality/audits/PLAN_COMPORTEMENTS_AVANCES.md
- Plan global : docs/quality/audits/PLAN_EVOLUTION_BBIA_COMPLET.md (Section Plan 4)
- Comportements existants : src/bbia_sim/bbia_behavior.py
- Architecture : docs/development/architecture/ARCHITECTURE_DETAILED.md

📋 TÂCHES À EFFECTUER :

⚠️ **IMPORTANT : Vérifier que chaque fichier/fonction n'existe pas déjà avant de créer !**

PHASE 1 : Structure Organisationnelle (Semaine 1)

1. **VÉRIFIER** si src/bbia_sim/behaviors/ existe déjà
   - Si oui : Vérifier contenu et améliorer
   - Si non : Créer structure : src/bbia_sim/behaviors/
   - Créer __init__.py avec exports
   - Déplacer comportements existants : follow_face.py, follow_object.py, conversation.py
   - Créer base.py avec BBIABehavior (copier depuis bbia_behavior.py)

2. Améliorer comportements existants
   - FollowFace : améliorer précision suivi, ajouter émotions selon distance
   - FollowObject : suivi multi-objets, priorisation (personne > objet)
   - Conversation : préparer pour intégration LLM (voir Agent 1)

PHASE 2 : Comportements Expressifs (Semaine 2)

3. Créer src/bbia_sim/behaviors/emotion_show.py
   - Parcourir toutes les 12 émotions BBIA
   - Transitions fluides avec interpolation minjerk
   - Explications vocales ("Maintenant je suis heureux")
   - Durée adaptative selon émotion

4. Créer src/bbia_sim/behaviors/dance.py
   - Détection rythme audio (analyse FFT)
   - Mouvements chorégraphiés (head + body + antennas)
   - Synchronisation musique
   - Émotions selon type musique (happy pour pop, calm pour classique)

5. Créer src/bbia_sim/behaviors/photo_booth.py
   - Poses pré-définies (happy, cool, surprised, etc.)
   - Détection visage pour cadrage (MediaPipe)
   - Compte à rebours ("3, 2, 1, souriez !")
   - Capture photo automatique (robot.media.camera.get_image())

PHASE 3 : Comportements Interactifs (Semaine 3-4)

6. Créer src/bbia_sim/behaviors/storytelling.py
   - Histoires pré-enregistrées (petit_chaperon_rouge, trois_petits_cochons)
   - Mouvements synchronisés avec narration
   - Émotions selon scènes histoire
   - Interaction utilisateur (questions via conversation)

7. Créer src/bbia_sim/behaviors/teaching.py
   - Leçons pré-définies (maths, sciences, etc.)
   - Mouvements explicatifs (pointer, montrer)
   - Questions/réponses interactives
   - Encouragements selon performance (émotions happy/sad)

8. Créer src/bbia_sim/behaviors/game.py
   - Jeux pré-définis (pierre-papier-ciseaux, devine nombre)
   - Détection gestes utilisateur (via vision MediaPipe)
   - Réactions selon résultat (happy si gagne, sad si perd)
   - Score et statistiques

PHASE 4 : Comportements Utilitaires (Semaine 5-6)

9. Créer src/bbia_sim/behaviors/meditation.py
   - Séances méditation guidées
   - Mouvements lents et fluides
   - Voix calme et apaisante (TTS avec voix douce)
   - Respiration synchronisée (mouvements tête)

10. Créer src/bbia_sim/behaviors/exercise.py
    - Exercices pré-définis (étirements, mouvements tête)
    - Mouvements démonstratifs
    - Comptage répétitions
    - Encouragements (émotions happy, excited)

11. Créer src/bbia_sim/behaviors/alarm_clock.py
    - Réveil à heure définie
    - Séquence réveil progressive (mouvements + voix)
    - Détection si utilisateur se réveille (vision)
    - Mode snooze

12. Créer src/bbia_sim/behaviors/weather_report.py
    - Récupération météo (API openweathermap ou similaire)
    - Mouvements selon météo (soleil = happy, pluie = sad)
    - Narration météo
    - Recommandations (parapluie, etc.)

13. Créer src/bbia_sim/behaviors/news_reader.py
    - Récupération actualités (RSS feed ou API)
    - Narration actualités
    - Réactions émotionnelles selon contenu
    - Résumé actualités

14. Créer src/bbia_sim/behaviors/music_reaction.py
    - Détection genre musical (analyse audio)
    - Mouvements selon rythme
    - Émotions selon musique
    - Synchronisation audio

PHASE 5 : Intégration & Tests (Semaine 7-8)

15. Intégrer tous comportements dans BBIABehaviorManager
    - Modifier src/bbia_sim/bbia_behavior.py
    - Ajouter tous nouveaux comportements dans liste
    - Gestion priorité et conflits

16. Créer tests : tests/test_behaviors_advanced.py
    - Test chaque comportement individuellement
    - Test intégration avec robot_api
    - Test gestion erreurs

17. Documentation : docs/guides/GUIDE_COMPORTEMENTS.md
    - Liste tous comportements
    - Utilisation basique
    - Configuration avancée
    - Création nouveaux comportements

✅ QUALITÉ CODE OBLIGATOIRE :
- Exécuter black, ruff, mypy, bandit après chaque modification
- Tous comportements héritent de BBIABehavior
- Implémenter can_execute() et execute() pour chaque comportement
- Gestion erreurs : try/except, fallback gracieux
- Tests : créer test pour chaque comportement

✅ SÉCURITÉ :
- Valider tous les inputs (heures, volumes, etc.)
- Pas d'exécution code externe
- Limiter ressources (timeout comportements)
- Sanitizer données API externes (météo, actualités)

✅ PERFORMANCE :
- Comportements asynchrones si nécessaire (async/await)
- Limiter durée comportements (max 5 minutes)
- Gestion mémoire (libérer ressources après exécution)
- Cache données externes (météo, actualités)

✅ QUAND UNE ÉTAPE EST TERMINÉE :
1. Exécuter : black . && ruff check . && mypy src/bbia_sim/ && bandit -r src/bbia_sim/
2. Exécuter : pytest tests/test_behaviors_advanced.py -v
3. Tester manuellement : lancer chaque comportement et vérifier fonctionnement
4. Vérifier : pas d'imports inutilisés, pas de doublons

🚀 COMMENCE PAR : 
1. **VÉRIFIER** si src/bbia_sim/behaviors/ existe déjà (list_dir)
2. **VÉRIFIER** quels comportements existent déjà (grep "class.*Behavior" src/bbia_sim/)
3. Si structure existe : Vérifier contenu et améliorer
4. Si n'existe pas : Créer structure et déplacer comportements existants
```

---

## 🤖 PROMPT 4 : AGENT PERFORMANCE & OPTIMISATION

```
Tu es un agent Cursor spécialisé dans l'optimisation de performance et latence.

🎯 MISSION :
Optimiser les performances de BBIA pour atteindre ou dépasser les performances de l'officiel (latence minimale, réactivité maximale).

📚 DOCUMENTS DE RÉFÉRENCE :
- Plan global : docs/quality/audits/PLAN_EVOLUTION_BBIA_COMPLET.md (Section Plan 3)
- Architecture : docs/development/architecture/ARCHITECTURE_DETAILED.md
- Code actuel : src/bbia_sim/bbia_vision.py, bbia_audio.py, bbia_voice.py

📋 TÂCHES À EFFECTUER :

⚠️ **IMPORTANT : Vérifier que chaque optimisation n'a pas déjà été faite !**

PHASE 1 : Optimisation Latence (Semaine 1-2)

**Date mise à jour : 19 novembre 2025**

1. ✅ **TERMINÉ** - Optimiser latence vision (objectif : <50ms, actuel ~100ms)
   - ✅ Cache modèle YOLO (déjà présent, vérifié)
   - ✅ Threading asynchrone pour détection objets - **IMPLÉMENTÉ 19/11/2025**
   - ✅ Réduire résolution image (640x480 au lieu de 1080p) - **IMPLÉMENTÉ 19/11/2025**
   - ✅ Utiliser YOLOv8n (nano) - **DÉJÀ EN PLACE**

2. ✅ **TERMINÉ** - Optimiser latence audio (objectif : <100ms, actuel ~200ms)
   - ✅ Cache modèle Whisper (déjà présent, vérifié)
   - ✅ Utiliser Whisper "tiny" au lieu de "base" - **DÉJÀ EN PLACE**
   - ✅ Fonction transcribe_audio() créée pour utiliser cache - **IMPLÉMENTÉ 19/11/2025**
   - ✅ Threading asynchrone pour STT - **DÉJÀ IMPLÉMENTÉ** (vérifié 19/11/2025)
   - ✅ Buffer audio optimisé (taille minimale) - **DÉJÀ EN PLACE**

3. ✅ **TERMINÉ** - Optimiser latence mouvements (objectif : <10ms, actuel ~20ms)
   - ✅ goto_target() déjà direct (pas de wrapper inutile) - **VÉRIFIÉ 19/11/2025**
   - ✅ Cache poses fréquentes (lru_cache) - **IMPLÉMENTÉ 19/11/2025**
   - ⏳ Réduire overhead RobotAPI (à vérifier si nécessaire)
   - ⏳ Éviter conversions inutiles (numpy → list) (à vérifier)

PHASE 2 : Streaming Optimisé (Semaine 3-4)

4. Implémenter stream vidéo optimisé
   - WebSocket ou WebRTC pour stream caméra
   - Compression adaptative (JPEG quality selon bande passante)
   - Frame rate adaptatif (30 FPS max)
   - Buffer optimisé (deque maxlen=5)

5. Implémenter stream audio optimisé
   - WebSocket pour stream microphone
   - Compression audio (Opus ou G.711)
   - Buffer optimisé (deque maxlen=10)
   - Latence minimale (<50ms)

6. Optimiser WebSocket dashboard
   - Réduire fréquence messages (pas de spam)
   - Batching messages (grouper updates)
   - Compression JSON si nécessaire
   - Heartbeat optimisé (30s au lieu de 10s)

PHASE 3 : Optimisation Mémoire (Semaine 5-6)

7. Optimiser chargement modèles IA
   - Lazy loading : charger modèles seulement si nécessaire
   - Cache modèles en mémoire (ne pas recharger)
   - Quantification modèles (8-bit si possible)
   - Libérer GPU si disponible

8. Optimiser gestion images
   - Réduire taille images en mémoire
   - Utiliser PIL pour resize avant traitement
   - Libérer images après traitement
   - Pas de copies inutiles (utiliser views numpy)

9. Optimiser gestion audio
   - Buffer audio optimisé (taille minimale)
   - Libérer buffers après traitement
   - Pas de copies inutiles
   - Utiliser sounddevice directement (pas de wrapper)

✅ QUALITÉ CODE OBLIGATOIRE :
- Exécuter black, ruff, mypy, bandit après chaque modification
- Benchmarks : créer tests/benchmarks/ pour mesurer latence
- Profiling : utiliser cProfile pour identifier bottlenecks
- Documentation : documenter optimisations appliquées

✅ SÉCURITÉ :
- Valider tous les inputs (résolutions, formats)
- Limiter ressources (timeout, max memory)
- Pas de code injection
- Gestion erreurs : try/except, fallback gracieux

✅ PERFORMANCE :
- Benchmarks avant/après chaque optimisation
- Mesurer latence : time.time() avant/après opérations
- Profiling : identifier fonctions lentes
- Tests performance : créer tests/benchmarks/test_performance.py

✅ QUAND UNE ÉTAPE EST TERMINÉE :
1. Exécuter : black . && ruff check . && mypy src/bbia_sim/ && bandit -r src/bbia_sim/
2. Exécuter : pytest tests/benchmarks/test_performance.py -v
3. Vérifier : latence réduite (mesurer avant/après)
4. Documenter : ajouter résultats benchmarks dans docs

🚀 COMMENCE PAR : 
1. ✅ **TERMINÉ** - tests/benchmarks/test_performance.py créé le 19/11/2025
2. ✅ **TERMINÉ** - Benchmarks créés pour mesurer latence (vision, audio, mouvements)
3. ✅ **TERMINÉ** - Tests de performance consolidés avec p50/p95/p99

**Prochaines étapes :**
- ✅ Optimiser latence mouvements (cache poses) - **TERMINÉ 19/11/2025**
- ✅ Threading asynchrone pour vision - **TERMINÉ 19/11/2025**
- ✅ Threading asynchrone pour audio - **DÉJÀ IMPLÉMENTÉ** (vérifié 19/11/2025)
- ⏳ PHASE 2 : Streaming optimisé (WebSocket/WebRTC)
```

---

## 🤖 PROMPT 5 : AGENT TESTS & QUALITÉ

```
Tu es un agent Cursor spécialisé dans les tests et la qualité de code.

🎯 MISSION :
Créer une suite de tests complète pour toutes les nouvelles fonctionnalités et garantir la qualité du code.

📚 DOCUMENTS DE RÉFÉRENCE :
- Plan global : docs/quality/audits/PLAN_EVOLUTION_BBIA_COMPLET.md
- Tests existants : tests/
- Qualité : docs/quality/audits/windsurf/WINDSURF_AUDIT_PHASE4.md

📋 TÂCHES À EFFECTUER :

⚠️ **IMPORTANT : Vérifier que chaque fichier de test n'existe pas déjà avant de créer !**

PHASE 1 : Tests Intelligence Conversationnelle (Semaine 1-2)

1. **VÉRIFIER** si tests/test_bbia_chat_llm.py existe déjà
   - Si oui : Lire et améliorer/étendre
   - Si non : Créer tests/test_bbia_chat_llm.py
   - Test chargement modèle (Phi-2 ou TinyLlama)
   - Test génération réponse
   - Test mémoire RAM (<6GB)
   - Test latence (<2s)
   - Test contexte conversation (historique)
   - Test détection actions robot
   - Test intégration émotions

2. Créer tests/test_bbia_chat_personalities.py
   - Test chaque personnalité (5 personnalités)
   - Test changement personnalité
   - Test adaptation style selon personnalité
   - Test apprentissage préférences

PHASE 2 : Tests Dashboard (Semaine 3-4)

3. Créer tests/test_dashboard_media.py
   - Test endpoints API media (volume, toggle)
   - Test WebSocket media updates
   - Test validation inputs (volume 0.0-1.0)
   - Test gestion erreurs

4. Créer tests/test_dashboard_3d.py
   - Test chargement modèle 3D (Three.js)
   - Test animation selon état
   - Test synchronisation robot réel
   - Test performance rendu (60 FPS)

PHASE 3 : Tests Comportements (Semaine 5-6)

5. Créer tests/test_behaviors_advanced.py
   - Test chaque comportement individuellement (15 comportements)
   - Test intégration avec robot_api
   - Test gestion erreurs
   - Test priorité et conflits

6. Créer tests/test_behaviors_integration.py
   - Test interactions entre comportements
   - Test gestion ressources (mémoire, CPU)
   - Test timeout comportements
   - Test arrêt propre comportements

PHASE 4 : Tests Performance (Semaine 7-8)

7. Créer tests/benchmarks/test_performance.py
   - Benchmark latence vision (objectif <50ms)
   - Benchmark latence audio (objectif <100ms)
   - Benchmark latence mouvements (objectif <10ms)
   - Benchmark mémoire RAM
   - Benchmark CPU usage

8. Créer tests/benchmarks/test_streaming.py
   - Test stream vidéo (latence, qualité)
   - Test stream audio (latence, qualité)
   - Test WebSocket dashboard (latence, throughput)

PHASE 5 : Tests Qualité Code (Semaine 9-10)

9. Créer tests/test_code_quality.py
   - Test imports (pas de doublons, pas d'inutilisés)
   - Test docstrings (présence, format)
   - Test type hints (présence, cohérence)
   - Test conformité SDK (utiliser create_head_pose(), etc.)

10. Créer tests/test_security.py
    - Test validation inputs (XSS, injection)
    - Test rate limiting
    - Test CORS
    - Test authentification (si applicable)

✅ QUALITÉ CODE OBLIGATOIRE :
- Tous tests doivent passer : pytest tests/ -v
- Couverture code : pytest --cov=src/bbia_sim tests/ --cov-report=html
- Tests rapides : marquer tests lents avec @pytest.mark.slow
- Tests hardware : marquer tests nécessitant hardware avec @pytest.mark.hardware

✅ SÉCURITÉ :
- Tests sécurité : valider tous les inputs
- Tests injection : tester XSS, SQL injection, etc.
- Tests rate limiting : tester limites
- Tests CORS : tester configuration

✅ PERFORMANCE :
- Benchmarks : mesurer avant/après optimisations
- Tests performance : créer tests/benchmarks/
- Profiling : utiliser cProfile pour identifier bottlenecks
- Documentation : documenter résultats benchmarks

✅ QUAND UNE ÉTAPE EST TERMINÉE :
1. Exécuter : pytest tests/ -v --cov=src/bbia_sim --cov-report=term-missing
2. Vérifier : tous tests passent, couverture >70%
3. Exécuter : black . && ruff check . && mypy src/bbia_sim/ && bandit -r src/bbia_sim/
4. Documenter : ajouter résultats tests dans docs

🚀 COMMENCE PAR : 
1. **VÉRIFIER** si tests/test_bbia_chat_llm.py existe déjà (glob_file_search)
2. **VÉRIFIER** si des tests LLM existent déjà (grep "llm\|LLM\|phi-2\|TinyLlama" tests/)
3. **VÉRIFIER** si src/bbia_sim/bbia_chat.py existe (pour savoir quoi tester)
4. Si tests existent : Lire et améliorer/étendre
5. Si n'existent pas : Créer avec tests basiques (chargement modèle, génération réponse)
```

---

## 📋 UTILISATION

### Pour chaque agent :

1. **Copier le prompt** correspondant à la tâche
2. **Ouvrir Cursor** et coller le prompt
3. **L'agent DOIT d'abord vérifier** que les fichiers/fonctions n'existent pas déjà
4. **Lancer l'agent** et laisser travailler
5. **Vérifier** que les vérifications finales sont faites
6. **Tester** manuellement si nécessaire

### ⚠️ RÈGLE D'OR : Toujours vérifier avant de créer !

**L'agent DOIT utiliser ces outils pour vérifier :**
- `glob_file_search` pour chercher des fichiers
- `list_dir` pour lister un répertoire
- `grep` pour chercher dans le code
- `codebase_search` pour chercher sémantiquement
- `read_file` pour lire un fichier existant avant modification

### Coordination :

- **Agent 1** (Intelligence) et **Agent 3** (Comportements) doivent communiquer pour intégration Conversation
- **Agent 2** (Dashboard) peut travailler indépendamment
- **Agent 4** (Performance) doit attendre que les autres agents aient terminé leurs fonctionnalités
- **Agent 5** (Tests) doit tester toutes les fonctionnalités créées par les autres agents

### Ordre recommandé :

1. **Semaine 1-2** : Agent 1 (Intelligence Phase 1) + Agent 3 (Comportements Phase 1)
2. **Semaine 3-4** : Agent 1 (Intelligence Phase 2) + Agent 2 (Dashboard Phase 1) + Agent 3 (Comportements Phase 2)
3. **Semaine 5-6** : Agent 1 (Intelligence Phase 3) + Agent 2 (Dashboard Phase 2-3) + Agent 3 (Comportements Phase 3-4)
4. **Semaine 7-8** : Agent 4 (Performance) + Agent 5 (Tests) + Agent 3 (Comportements Phase 5)

---

**Document créé le :** Novembre 2024  
**Version BBIA :** 1.3.2  
**Auteur :** Arkalia Luna System

