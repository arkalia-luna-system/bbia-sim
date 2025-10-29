## Veille Reachy Mini — Profils/Projets similaires

Objectif: détecter des développeurs et dépôts explicitement liés à Reachy Mini (pas Reachy 1/2) présentant des éléments d'architecture avancée (API réseau, dashboard, CI/coverage, simu, moteur cognitif/émotions, etc.).

### Mots-clés principaux (FR/EN + variantes)
- Reachy Mini, ReachyMini, Pollen Robotics Reachy Mini
- SDK Reachy Mini, MuJoCo Reachy Mini, simulator, simulation
- WebSocket, REST API, backend, middleware, telemetry, monitoring, metrics
- Dashboard, web UI, frontend, orchestration
- Emotion engine, affective, mood, cognitive architecture, autonomy, behavior trees
- LLM, RAG, TTS, STT, ONNX, local inference
- Unity, ROS2, robotics integration
- CI, coverage, testing, end-to-end, E2E

### Requêtes prêtes à l'emploi

#### GitHub
- "Reachy Mini" in:name,description,readme language:Python
- "Reachy Mini" in:readme language:TypeScript OR language:JavaScript
- "Reachy Mini" AND (WebSocket OR REST OR API) in:readme
- "Reachy Mini" AND (dashboard OR "web UI" OR frontend) in:readme
- "Reachy Mini" AND (MuJoCo OR simulation OR simulator) in:readme
- "Reachy Mini" AND (CI OR coverage OR tests) in:readme
- "Reachy Mini" AND (emotion OR affective OR mood) in:readme

#### Reddit
- site:reddit.com "Reachy Mini"
- site:reddit.com "Reachy Mini" (dashboard OR API OR WebSocket OR CI OR MuJoCo)

#### Hugging Face (Spaces/Repos)
- "Reachy Mini" in:spaces
- "Reachy Mini" AND (API OR WebSocket OR backend)
- "Reachy Mini" AND (simulation OR MuJoCo)

#### LinkedIn
- Posts/Articles: "Reachy Mini" + (API, dashboard, MuJoCo, CI, emotions, cognitive)

#### YouTube
- "Reachy Mini" demo OR "end-to-end" OR dashboard OR API OR MuJoCo

### Critères de similarité (scoring base 0–5 + bonus)
- API réseau (REST/WebSocket) exposée: 0–1
- CI visible (badge, workflow) et/ou coverage: 0–1
- Dashboard/UI d'orchestration: 0–1
- Simu (MuJoCo ou équivalent) et/ou exécution réelle: 0–1
- Moteur émotions/architecture cognitive non triviale: 0–1

Bonus:
- Présence de tests (répertoires/configs): +1
- Topics GitHub incluant Reachy Mini: +1

Score total: somme (plus le score est élevé, plus c'est proche).

### Tableau de suivi (renseigner au fil de l'eau)

| Candidat | Lien | Type (repo/post/space) | Points forts | Manques | Score (0–5) | Statut |
|---------|------|-------------------------|--------------|---------|-------------|--------|
| Exemple | https://... | repo | API REST, WebSocket | pas de CI/coverage | 2 | à suivre |

Notes:
- Inclure captures/vidéos si dispo (lien YT/Twitter/X) dans la colonne Points forts.
- Utiliser la colonne Statut: à évaluer / à suivre / contacter / ignoré.

### Fréquence de revue
- Recommandé: hebdomadaire. Ajuster si activité accrue.

### Historique des mises à jour
- 2025-10-29: création du document et des requêtes.
- 2025-10-29: ajout scoring étendu (+tests, topics) et export JSON.


