# 👥 TRAVAIL TECHNIQUE DES CONTRIBUTEURS - Inspiration pour BBIA

**Date** : 8 Décembre 2025 (Mise à jour)  
**Source** : [pollen-robotics/reachy_mini](https://github.com/pollen-robotics/reachy_mini)  
**Version SDK** : v1.1.1 (Latest - Nov 25, 2025)  
**Objectif** : Analyser le travail technique de chaque contributeur et identifier ce dont BBIA peut s'inspirer

---

## 🎯 APPROCHE

Ce document analyse le **travail technique concret** de chaque contributeur, leurs **patterns et techniques**, et ce que **BBIA peut s'inspirer** de leur approche.

**Pas de statistiques de commits** - Seulement le travail technique et les innovations.

---

## 👨‍💻 CONTRIBUTEURS PRINCIPAUX - TRAVAIL TECHNIQUE

### 1. @pierre-rouanet - Architecture SDK & Daemon

**Travail technique principal** :

#### Architecture Daemon FastAPI
- **Pattern** : Service d'arrière-plan avec FastAPI pour API REST + WebSocket
- **Approche** : Séparation claire entre daemon (communication hardware) et SDK (interface Python)
- **Innovation** : Support simultané simulation MuJoCo ET robot réel via même daemon
- **Technique** : Lifespan context manager pour gestion cycle de vie (startup/shutdown)

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a daemon FastAPI similaire (`src/bbia_sim/daemon/app/main.py`)
- 💡 **À améliorer** : Lifespan context manager plus robuste (gestion erreurs startup)
- ⏳ **INFRASTRUCTURE CRÉÉE** (8 Déc 2025) : Support simultané sim/robot réel (infrastructure créée, routing à finaliser)

#### Architecture Zenoh pour Communication
- **Pattern** : Communication distribuée via Zenoh (pub/sub, discovery automatique)
- **Approche** : Abstraction réseau pour communication robot (USB, wireless, réseau)
- **Innovation** : Découverte automatique robots sur réseau local
- **Technique** : Configuration Zenoh via variables d'environnement

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a bridge Zenoh (`src/bbia_sim/daemon/bridge.py`)
- ⏳ **INFRASTRUCTURE CRÉÉE** (8 Déc 2025) : Découverte automatique robots (infrastructure créée, découverte complète à finaliser)
- 💡 **À améliorer** : Support multi-robots simultanés via Zenoh

#### Backends USB et Wireless
- **Pattern** : Backends séparés mais interface unifiée
- **Approche** : Détection automatique du type de connexion (USB vs wireless)
- **Innovation** : Fallback gracieux si un backend échoue
- **Technique** : Factory pattern pour instanciation backend

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a RobotAPI unifié avec backends séparés
- 💡 **À améliorer** : Détection automatique type connexion (actuellement choix manuel)
- 💡 **À améliorer** : Fallback automatique sim → robot si robot disponible

---

### 2. @apirrone - Simulation MuJoCo & Modèles 3D

**Travail technique principal** :

#### Modèles 3D Officiels
- **Pattern** : Modèles XML MuJoCo avec assets STL séparés
- **Approche** : Modèle simplifié (7 joints) + modèle complet (16 joints)
- **Innovation** : Chargement conditionnel selon besoins (performance vs précision)
- **Technique** : Assets STL référencés relativement dans XML

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA utilise `reachy_mini_REAL_OFFICIAL.xml` (modèle complet)
- ✅ **FAIT** (8 Déc 2025) : Support modèle simplifié pour tests rapides (flag `--fast` implémenté)
- 💡 **À améliorer** : Chargement lazy des assets STL (actuellement tout chargé au démarrage)

#### Intégration Physique Réaliste
- **Pattern** : Physique MuJoCo avec masses, inerties, collisions
- **Approche** : Timestep fixe 0.01s (100Hz) pour stabilité
- **Innovation** : Support headless pour CI/CD (pas besoin d'affichage)
- **Technique** : Viewer MuJoCo optionnel (mode graphique vs headless)

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a simulation MuJoCo complète avec physique
- 💡 **À améliorer** : Optimisation timestep adaptatif (actuellement fixe 0.01s)
- 💡 **À améliorer** : Support scènes complexes avec objets interactifs (actuellement scène vide)

#### Optimisations Performance Simulation
- **Pattern** : Cache modèles préchargés, batch processing
- **Approche** : Limite steps pour éviter boucles infinies
- **Innovation** : Déchargement modèle après arrêt pour libérer RAM
- **Technique** : Monitoring performance (FPS, latence) intégré

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a limite 10000 steps et déchargement modèle
- 💡 **À améliorer** : Cache plus agressif pour modèles fréquemment utilisés
- 💡 **À améliorer** : Batch processing pour mouvements multiples simultanés

---

### 3. @FabienDanieau - Dashboard Web & API REST

**Travail technique principal** :

#### Dashboard Web Simple
- **Pattern** : Interface web minimaliste avec FastAPI + templates Jinja2
- **Approche** : Contrôles de base (on/off, mouvements simples)
- **Innovation** : Intégration Hugging Face Spaces pour recherche apps
- **Technique** : StaticFiles pour assets, WebSocket pour temps réel

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a 4 dashboards (supérieur à l'officiel)
- 💡 **À améliorer** : Intégration Hugging Face Spaces plus poussée (actuellement basique)
- 💡 **À améliorer** : Interface plus simple pour débutants (BBIA est très complet mais complexe)

#### Endpoints API REST
- **Pattern** : RESTful API avec OpenAPI/Swagger documentation
- **Approche** : Endpoints séparés par domaine (motion, state, media, etc.)
- **Innovation** : Rate limiting et authentification Bearer Token
- **Technique** : Pydantic models pour validation entrées/sorties

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a API REST complète avec 50+ endpoints
- 💡 **À améliorer** : Rate limiting plus granulaire (actuellement global)
- 💡 **À améliorer** : Documentation OpenAPI plus détaillée avec exemples

#### Communication WebSocket
- **Pattern** : WebSocket pour télémétrie temps réel
- **Approche** : Batching optimisé pour réduire overhead réseau
- **Innovation** : Support multi-clients simultanés
- **Technique** : Heartbeat pour détecter déconnexions

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a WebSocket temps réel avec batching
- 💡 **À améliorer** : Heartbeat plus robuste (actuellement basique)
- 💡 **À améliorer** : Support reconnection automatique côté client

---

### 4. @RemiFabre - Tests & CI/CD

**Travail technique principal** :

#### Suite de Tests Automatisés
- **Pattern** : Tests unitaires + intégration + E2E
- **Approche** : Fixtures pytest pour setup/teardown
- **Innovation** : Tests de conformité SDK (validation API)
- **Technique** : Mocking pour tests sans hardware

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a 1,743 tests (supérieur à l'officiel)
- 💡 **À améliorer** : Tests de conformité SDK plus exhaustifs (actuellement 37 tests)
- 💡 **À améliorer** : Tests de performance avec baselines (actuellement basiques)

#### Pipeline CI/CD GitHub Actions
- **Pattern** : Workflow multi-étapes (lint, tests, e2e, artifacts)
- **Approche** : Matrice Python (3.11, 3.12) pour compatibilité
- **Innovation** : Tests headless MuJoCo en CI (pas besoin d'affichage)
- **Technique** : Artifacts upload (coverage, logs) sur échec

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a CI/CD complet avec matrice Python
- 💡 **À améliorer** : Tests headless MuJoCo plus robustes (actuellement parfois instables)
- 💡 **À améliorer** : Sharding tests si durée > 10 min (actuellement séquentiel)

#### Qualité Code (Black, Ruff, MyPy)
- **Pattern** : Pre-commit hooks + CI validation
- **Approche** : Formatage automatique + linting strict
- **Innovation** : Type checking MyPy pour sécurité types
- **Technique** : Configuration partagée via `pyproject.toml`

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a Black, Ruff, MyPy, Bandit configurés
- 💡 **À améliorer** : MyPy strict mode (actuellement permissive)
- 💡 **À améliorer** : Pre-commit hooks plus complets (actuellement basiques)

---

### 5. @askurique - Documentation & Exemples

**Travail technique principal** :

#### Guides d'Utilisation
- **Pattern** : Documentation Markdown avec exemples code
- **Approche** : Guides par niveau (débutant, intermédiaire, avancé)
- **Innovation** : Exemples exécutables (scripts Python complets)
- **Technique** : Liens croisés entre docs pour navigation

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a 219 fichiers MD (supérieur à l'officiel)
- 💡 **À améliorer** : Guides par niveau plus clairs (actuellement tout mélangé)
- 💡 **À améliorer** : Exemples exécutables avec validation automatique

#### Exemples de Base
- **Pattern** : Exemples simples → complexes (progression)
- **Approche** : Un exemple = une fonctionnalité
- **Innovation** : Exemples avec erreurs communes et solutions
- **Technique** : Commentaires détaillés dans code

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a 67 exemples (supérieur à l'officiel)
- 💡 **À améliorer** : Progression plus claire (débutant → expert)
- 💡 **À améliorer** : Exemples avec erreurs communes documentées

---

## 🧪 PROJETS COMMUNAUTAIRES - INSPIRATION

### 1. reachy-mini-plugin (LAURA-agent)

**Travail technique** :
- **Pattern** : Plugin pour mouvements émotionnels naturels pendant conversation
- **Approche** : Synchronisation fine mouvements ↔ timing parole
- **Innovation** : Micro-mouvements pendant écoute (antennes, tête)
- **Technique** : États conversationnels (IDLE, LISTENING, THINKING, SPEAKING)

**Ce que BBIA peut s'inspirer** :
- ✅ **Déjà fait** : BBIA a `bbia_emotional_sync.py` avec synchronisation fine
- ✅ **Déjà fait** : BBIA a états conversationnels (IDLE, LISTENING, THINKING, SPEAKING, REACTING)
- ✅ **FAIT** (8 Déc 2025) : Micro-mouvements plus subtils pendant écoute (0.01-0.02 rad, effet respiration)
- ✅ **FAIT** (8 Déc 2025) : Timing adaptatif selon rythme parole (analyse pauses, mots courts)

---

### 2. reachy-mini-mcp (OriNachum)

**Travail technique** :
- **Pattern** : Serveur MCP (Model Context Protocol) pour contrôle robot
- **Approche** : Interface standardisée pour intégration LLM
- **Innovation** : Contrôle robot via FastMCP (protocole standardisé)
- **Technique** : Bridge MCP ↔ SDK Reachy Mini

**Ce que BBIA peut s'inspirer** :
- ⚠️ **Optionnel** : BBIA a déjà API REST + WebSocket (supérieur à MCP)
- 💡 **À évaluer** : Intégration MCP si besoin standardisation (actuellement pas nécessaire)

---

## 📊 RÉSUMÉ - CE QUE BBIA PEUT S'INSPIRER

### ✅ Déjà Supérieur à l'Officiel

1. **Architecture** : RobotAPI unifié (officiel n'a pas ça)
2. **Modules IA** : 15+ modules vs basiques officiels
3. **Tests** : 1,743 tests vs standards officiels
4. **Documentation** : 219 fichiers MD vs complète officielle
5. **Dashboards** : 4 dashboards vs 1 officiel

### 💡 AMÉLIORATIONS POSSIBLES - Inspiration Contributeurs

#### 🏗️ Architecture & Infrastructure

1. **Découverte automatique robots** (inspiration @pierre-rouanet)
   - **État actuel** : Configuration manuelle (`BBIA_HOSTNAME`, `BBIA_PORT`)
   - **À faire** : Détection automatique robots sur réseau local via Zenoh
   - **Technique** : Utiliser `zenoh.discover()` pour lister robots disponibles
   - **Bénéfice** : Plus besoin de configurer manuellement, découverte automatique
   - **Priorité** : 🟡 Moyenne
   - **Temps estimé** : 4-6h

2. **Support simultané sim/robot réel** (inspiration @pierre-rouanet)
   - **État actuel** : BBIA choisit un backend (sim OU robot)
   - **À faire** : Support simultané via même daemon (sim + robot réel)
   - **Technique** : Multi-backends avec routing selon commande
   - **Bénéfice** : Tests sim pendant utilisation robot réel
   - **Priorité** : 🟡 Moyenne
   - **Temps estimé** : 6-8h

3. **Fallback automatique sim → robot** (inspiration @pierre-rouanet)
   - **État actuel** : Choix manuel du backend
   - **À faire** : Détection automatique robot, fallback vers sim si absent
   - **Technique** : Try robot réel, catch → sim automatiquement
   - **Bénéfice** : Expérience utilisateur améliorée (pas de config)
   - **Priorité** : 🟡 Moyenne
   - **Temps estimé** : 2-3h

#### 🎮 Simulation MuJoCo

4. ✅ **Modèle simplifié pour tests rapides** (inspiration @apirrone) - **FAIT** (8 Déc 2025)
   - **État actuel** : ✅ Flag `--fast` implémenté
   - **Réalisé** : Support modèle 7 joints pour tests rapides
   - **Technique** : Flag `--fast` pour charger `reachy_mini.xml` (7 joints)
   - **Bénéfice** : Tests 2-3x plus rapides (moins de joints)
   - **Fichiers** : `__main__.py` (flag ajouté), `robot_factory.py` (support)

5. **Chargement lazy assets STL** (inspiration @apirrone)
   - **État actuel** : Tous les assets STL chargés au démarrage
   - **À faire** : Chargement à la demande (lazy loading)
   - **Technique** : Charger assets seulement si nécessaire pour rendu
   - **Bénéfice** : Démarrage plus rapide, moins de RAM
   - **Priorité** : 🟢 Basse
   - **Temps estimé** : 3-4h

6. **Scènes complexes avec objets interactifs** (inspiration @apirrone)
   - **État actuel** : Scène vide (minimal.xml)
   - **À faire** : Scènes avec objets (tables, objets à manipuler)
   - **Technique** : Créer scènes XML avec objets MuJoCo
   - **Bénéfice** : Tests manipulation objets, interactions
   - **Priorité** : 🟢 Basse
   - **Temps estimé** : 4-6h

7. **Timestep adaptatif** (inspiration @apirrone)
   - **État actuel** : Timestep fixe 0.01s (100Hz)
   - **À faire** : Timestep adaptatif selon complexité scène
   - **Technique** : Ajuster timestep dynamiquement (0.005s-0.02s)
   - **Bénéfice** : Performance optimale selon scène
   - **Priorité** : 🟢 Basse
   - **Temps estimé** : 3-4h

#### 🌐 Dashboard & API

8. **Mode débutant avec contrôles basiques** (inspiration @FabienDanieau)
   - **État actuel** : Interface complète mais complexe
   - **À faire** : Mode "débutant" avec contrôles simplifiés (on/off, mouvements basiques)
   - **Technique** : Toggle mode débutant/expert dans dashboard
   - **Bénéfice** : Accessibilité pour nouveaux utilisateurs
   - **Priorité** : 🟡 Moyenne
   - **Temps estimé** : 4-6h

9. **Intégration Hugging Face Spaces plus poussée** (inspiration @FabienDanieau)
   - **État actuel** : Intégration basique (recherche apps)
   - **À faire** : Installation apps directement depuis dashboard
   - **Technique** : API HF Hub pour téléchargement/installation apps
   - **Bénéfice** : Écosystème apps plus riche
   - **Priorité** : 🟡 Moyenne
   - **Temps estimé** : 6-8h

10. **Rate limiting plus granulaire** (inspiration @FabienDanieau)
    - **État actuel** : Rate limiting global
    - **À faire** : Rate limiting par endpoint (motion, state, media, etc.)
    - **Technique** : Middleware FastAPI avec limites par route
    - **Bénéfice** : Protection plus fine, meilleure UX
    - **Priorité** : 🟢 Basse
    - **Temps estimé** : 2-3h

11. **Documentation OpenAPI plus détaillée** (inspiration @FabienDanieau)
    - **État actuel** : Documentation OpenAPI basique
    - **À faire** : Exemples complets dans OpenAPI (request/response)
    - **Technique** : Ajouter `examples` dans Pydantic models
    - **Bénéfice** : Meilleure compréhension API pour développeurs
    - **Priorité** : 🟢 Basse
    - **Temps estimé** : 3-4h

12. **Heartbeat WebSocket plus robuste** (inspiration @FabienDanieau)
    - **État actuel** : Heartbeat basique (30s)
    - **À faire** : Heartbeat adaptatif + reconnection automatique
    - **Technique** : Heartbeat selon latence, auto-reconnect côté client
    - **Bénéfice** : Connexions plus stables, récupération automatique
    - **Priorité** : 🟡 Moyenne
    - **Temps estimé** : 3-4h

#### 🧪 Tests & Qualité

13. **Tests de performance avec baselines** (inspiration @RemiFabre)
    - **État actuel** : Tests de performance basiques (pas de validation)
    - **À faire** : Baselines p50/p95/p99 avec validation automatique
    - **Technique** : Exporter métriques JSONL, valider fourchette en CI
    - **Bénéfice** : Détection régression performance automatique
    - **Priorité** : 🟡 Moyenne
    - **Temps estimé** : 4-6h

14. **Tests de conformité SDK plus exhaustifs** (inspiration @RemiFabre)
    - **État actuel** : 37 tests de conformité
    - **À faire** : Tests edge cases, limites, erreurs
    - **Technique** : Ajouter tests limites joints, erreurs réseau, timeouts
    - **Bénéfice** : Conformité SDK garantie à 100%
    - **Priorité** : 🟡 Moyenne
    - **Temps estimé** : 6-8h

15. **Sharding tests si durée > 10 min** (inspiration @RemiFabre)
    - **État actuel** : Tests séquentiels (long si beaucoup de tests)
    - **À faire** : Sharding avec pytest-xdist pour parallélisation
    - **Technique** : `pytest -n auto` pour tests parallèles
    - **Bénéfice** : CI plus rapide (2-3x plus rapide)
    - **Priorité** : 🟢 Basse
    - **Temps estimé** : 2-3h

16. **Tests headless MuJoCo plus robustes** (inspiration @RemiFabre)
    - **État actuel** : Tests headless parfois instables
    - **À faire** : Retry automatique, meilleure gestion erreurs
    - **Technique** : Fixtures pytest avec retry, timeout plus longs
    - **Bénéfice** : CI plus stable, moins de flaky tests
    - **Priorité** : 🟡 Moyenne
    - **Temps estimé** : 3-4h

17. **MyPy strict mode** (inspiration @RemiFabre)
    - **État actuel** : MyPy permissive (beaucoup de `# type: ignore`)
    - **À faire** : MyPy strict mode progressif (fichier par fichier)
    - **Technique** : Activer strict mode progressivement, corriger types
    - **Bénéfice** : Sécurité types garantie, moins de bugs
    - **Priorité** : 🟢 Basse
    - **Temps estimé** : 8-12h (progressif)

18. **Pre-commit hooks plus complets** (inspiration @RemiFabre)
    - **État actuel** : Pre-commit hooks basiques
    - **À faire** : Ajouter tests unitaires rapides, validation docs
    - **Technique** : Hook pour lancer tests rapides avant commit
    - **Bénéfice** : Détection erreurs avant push
    - **Priorité** : 🟢 Basse
    - **Temps estimé** : 2-3h

#### 📚 Documentation & Exemples

19. **Guides par niveau** (inspiration @askurique)
    - **État actuel** : Documentation tout mélangé (débutant/expert)
    - **À faire** : Organiser guides par niveau (débutant → intermédiaire → expert)
    - **Technique** : Structure `docs/beginner/`, `docs/intermediate/`, `docs/advanced/`
    - **Bénéfice** : Navigation plus claire, progression naturelle
    - **Priorité** : 🟡 Moyenne
    - **Temps estimé** : 4-6h

20. **Exemples avec erreurs communes** (inspiration @askurique)
    - **État actuel** : Exemples basiques (fonctionnent toujours)
    - **À faire** : Exemples avec erreurs communes et solutions
    - **Technique** : Ajouter section "Erreurs communes" dans exemples
    - **Bénéfice** : Apprentissage plus rapide, moins de frustration
    - **Priorité** : 🟢 Basse
    - **Temps estimé** : 3-4h

21. **Exemples exécutables avec validation** (inspiration @askurique)
    - **État actuel** : Exemples Python (pas de validation automatique)
    - **À faire** : Validation automatique exemples (tests)
    - **Technique** : Tests qui exécutent exemples et valident sortie
    - **Bénéfice** : Garantie exemples toujours fonctionnels
    - **Priorité** : 🟢 Basse
    - **Temps estimé** : 4-6h

#### 🎭 Mouvements & Synchronisation

22. ✅ **Timing adaptatif selon rythme parole** (inspiration LAURA-agent) - **FAIT** (8 Déc 2025)
    - **État actuel** : ✅ Timing adaptatif implémenté
    - **Réalisé** : Analyse rythme réel parole, ajustement dynamique
    - **Technique** : Détection pauses, accélérations dans parole
    - **Bénéfice** : Synchronisation plus naturelle, mouvements adaptés
    - **Fichiers** : `bbia_emotional_sync.py`, tests (4 tests)

23. ✅ **Micro-mouvements plus subtils pendant écoute** (inspiration LAURA-agent) - **FAIT** (8 Déc 2025)
    - **État actuel** : ✅ Micro-mouvements subtils (0.01-0.02 rad)
    - **Réalisé** : Animations subtiles (micro-expressions, respiration)
    - **Technique** : Micro-mouvements très petits (0.01-0.02 rad), effet respiration
    - **Bénéfice** : Robot plus vivant, interactions plus naturelles
    - **Fichiers** : `bbia_emotional_sync.py` (amélioré)

24. **Cache plus agressif pour modèles fréquents** (inspiration @apirrone)
    - **État actuel** : Cache basique
    - **À faire** : Cache LRU pour modèles MuJoCo fréquemment utilisés
    - **Technique** : `functools.lru_cache` pour modèles XML
    - **Bénéfice** : Chargement modèles plus rapide
    - **Priorité** : 🟢 Basse
    - **Temps estimé** : 2-3h

25. **Batch processing mouvements multiples** (inspiration @apirrone)
    - **État actuel** : Mouvements séquentiels
    - **À faire** : Batch processing pour mouvements simultanés
    - **Technique** : Grouper mouvements, exécuter en batch
    - **Bénéfice** : Performance améliorée (moins d'appels SDK)
    - **Priorité** : 🟢 Basse
    - **Temps estimé** : 4-6h

---

## 📋 PRIORISATION DES AMÉLIORATIONS

### 🟡 Priorité Moyenne (Impact Utilisateur)

1. ⏳ **Découverte automatique robots** (4-6h) - Infrastructure créée
2. ⏳ **Support simultané sim/robot réel** (6-8h) - Infrastructure créée
3. **Fallback automatique sim → robot** (2-3h)
4. ✅ **Modèle simplifié pour tests** (2-3h) - **FAIT**
5. **Mode débutant dashboard** (4-6h)
6. **Intégration HF Spaces plus poussée** (6-8h)
7. **Heartbeat WebSocket robuste** (3-4h)
8. **Tests performance avec baselines** (4-6h)
9. **Tests conformité SDK exhaustifs** (6-8h)
10. **Tests headless MuJoCo robustes** (3-4h)
11. **Guides par niveau** (4-6h)
12. ✅ **Timing adaptatif parole** (4-6h) - **FAIT**
13. ✅ **Micro-mouvements subtils** (3-4h) - **FAIT**

**Total priorité moyenne** : ~50-70h

### 🟢 Priorité Basse (Améliorations Futures)

14. **Chargement lazy assets STL** (3-4h)
15. **Scènes complexes** (4-6h)
16. **Timestep adaptatif** (3-4h)
17. **Rate limiting granulaire** (2-3h)
18. **Documentation OpenAPI détaillée** (3-4h)
19. **Sharding tests** (2-3h)
20. **MyPy strict mode** (8-12h)
21. **Pre-commit hooks complets** (2-3h)
22. **Exemples erreurs communes** (3-4h)
23. **Exemples exécutables validés** (4-6h)
24. **Cache modèles agressif** (2-3h)
25. **Batch processing mouvements** (4-6h)

**Total priorité basse** : ~40-60h

---

**Dernière mise à jour** : 8 Décembre 2025  
**Voir aussi** :
- `CE_QUI_MANQUE_VRAIMENT_BBIA_DEC2025.md` - Ce qui manque vraiment
- `TACHES_RESTANTES_CONSOLIDEES.md` - Tâches restantes consolidées
- `AUDIT_REACHY_MINI_DECEMBRE_2025.md` - Audit complet décembre 2025
