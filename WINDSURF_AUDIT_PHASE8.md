# 🔍 AUDIT BBIA-SIM - PHASE 8 : PERFORMANCE RAM/CPU

## ⚠️ RÈGLES ABSOLUES

- **NE MODIFIE AUCUN FICHIER**
- **Analyse statique uniquement**
- **Ouvre les fichiers et lis-les ligne par ligne** (ne pas utiliser grep)

---

## 🎯 OBJECTIF

Identifier les optimisations critiques de performance

**MÉTHODE :** Ouvre chaque fichier, lis-le complètement, analyse ligne par ligne

---

## 📋 ACTIONS À EXÉCUTER (3 actions)

### Action 8.1 : Chercher les `deque` vs `list`

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/dashboard_advanced.py`
2. **Lis** le fichier ligne par ligne
3. **Pour chaque ligne** qui contient `deque(` :
   - Note le numéro de ligne et vérifie si `maxlen=` est présent ✅
4. **Pour chaque ligne** qui contient `= []` (liste vide) :
   - Note le numéro de ligne
   - **Lis** le contexte (est-ce un buffer ?)
   - Si c'est un buffer, devrait être `deque` avec `maxlen`

**EXEMPLE TROUVÉ :**
Dans `dashboard_advanced.py` :
```python
metrics_history: deque[dict[str, Any]] = deque(maxlen=self.max_history)  # ✅ Bon
```

**RÉSULTAT OBTENU :**
| Fichier | Ligne | Code | Devrait être deque ? |
|---------|-------|------|---------------------|
| dashboard_advanced.py | 65 | `deque(maxlen=...)` | ✅ NON |
| dashboard_advanced.py | 54 | `active_connections = []` | ❌ NON |
| dashboard_advanced.py | 243 | `inactive_connections = []` | ❌ OUI |
| dashboard_advanced.py | 267 | `disconnected = []` | ❌ OUI |

**Analyse détaillée :**

**Deque correctement utilisée (ligne 65) :**
```python
self.metrics_history: deque[dict[str, Any]] = deque(maxlen=self.max_history)
```
- **maxlen présent** : ✅ Limite mémoire configurée
- **Usage approprié** : ✅ Buffer circulaire pour métriques

**Listes qui devraient être deque :**

**Ligne 243** : `inactive_connections: list[tuple[WebSocket, float]] = []`
- **Usage temporaire** : Collecte des connexions inactives
- **Taille limitée** : Détruite après traitement
- **Recommandation** : `deque(maxlen=100)` pour sécurité

**Ligne 267** : `disconnected = []`
- **Usage temporaire** : Tracking déconnexions
- **Pas de limite** : Peut croître indéfiniment
- **Recommandation** : `deque(maxlen=50)`

**Liste acceptable (ligne 54) :**
- `active_connections = []` : Connexions actives, taille dynamique nécessaire

**Problèmes identifiés :**
- **Memory leak potentiel** : Listes temporaires sans limite
- **Pas de cleanup automatique** : Accumulation possible
- **Performance** : Listes plus lentes que deque pour insertions/suppressions

**Score : 6/10**

### Action 8.2 : Chercher les boucles bloquantes

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/dashboard_advanced.py`
2. **Lis** le fichier ligne par ligne
3. **Pour chaque ligne** qui contient `while True` :
   - Note le numéro de ligne
   - **Lis** le corps de la boucle
   - Vérifie s'il y a un `await` ou `sleep` dans la boucle
   - Vérifie s'il y a un mécanisme de sortie (break, return, flag)

**RÉSULTAT ATTENDU :**
| Fichier | Ligne | Code | Bloquant ? | Problème |
|---------|-------|------|------------|----------|
| dashboard_advanced.py | 377 | `while not self._stop_metrics:` | NON | ✅ OK |

---

### Action 8.3 : Chercher les `@lru_cache` manquants

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/backends/reachy_mini_backend.py`
2. **Lis** le fichier ligne par ligne
3. **Pour chaque fonction** (ligne `def `) :
   - **Lis** le corps de la fonction
   - Vérifie si la fonction est "pure" (pas de side effects, juste calcul)
   - Vérifie si elle est décorée avec `@lru_cache` ou `@cache`
   - Si pure et pas de cache : ❌ Devrait être cachée

**RÉSULTAT ATTENDU :**
| Fonction | Fichier | Ligne | Devrait être cachée ? |
|----------|---------|-------|---------------------|
| `_get_available_joints` | ? | ? | OUI/NON |

---

## 🎨 FORMAT DE RÉPONSE

Pour chaque action :
- **Résultat** : Tableau
- **Problèmes** : Liste
- **Score** : X/10

---

## ⚠️ IMPORTANT : MÉTHODE D'ANALYSE

**NE PAS UTILISER grep**

**MÉTHODE CORRECTE :**
1. Utilise `read_file` pour ouvrir chaque fichier
2. Lis le fichier complètement
3. Analyse ligne par ligne dans ta mémoire

---

## ⚠️ VÉRIFICATION DE COHÉRENCE

**APRÈS avoir complété toutes les actions, vérifie :**
1. Les scores individuels correspondent-ils aux calculs pondérés ?
2. Les conclusions correspondent-elles aux résultats détaillés ?
3. Y a-t-il des contradictions entre les actions ?

**Si tu trouves une incohérence, note-la clairement dans le résumé.**

---

## 🚀 COMMENCE MAINTENANT

**Exécute les 3 actions dans l'ordre et rapporte les résultats.**

## 📊 RÉSULTATS

### Action 8.1 : Chercher les `deque` vs `list`

**RÉSULTAT :**
| Fichier | Ligne | Code | Devrait être deque ? |
|---------|-------|------|---------------------|
| dashboard_advanced.py | 54 | `self.active_connections: list[WebSocket] = []` | NON |
| dashboard_advanced.py | 65 | `self.metrics_history: deque[dict[str, Any]] = deque(maxlen=self.max_history)` | ✅ DÉJÀ |
| dashboard_advanced.py | 243 | `inactive_connections: list[tuple[WebSocket, float]] = []` | NON |
| dashboard_advanced.py | 267 | `disconnected = []` | NON |
| dashboard_advanced.py | 2340 | `const toastQueue = [];` (JS) | NON |

**Analyse détaillée :**

**Ligne 65** : `self.metrics_history: deque[dict[str, Any]] = deque(maxlen=self.max_history)`
- **Utilise deque** : ✅ **CORRECT**
- **Avec maxlen** : ✅ **OPTIMAL** (limite à 1000)
- **Commentaire** : "OPTIMISATION RAM: Utiliser deque au lieu de liste"

**Ligne 54** : `self.active_connections: list[WebSocket] = []`
- **Contexte** : Liste des connexions WebSocket actives
- **Devrait être deque ?** : ❌ **NON** (besoin de accès aléatoire)
- **Justification** : On ajoute/supprime mais aussi itère fréquemment

**Ligne 243** : `inactive_connections: list[tuple[WebSocket, float]] = []`
- **Contexte** : Liste temporaire pour nettoyage
- **Devrait être deque ?** : ❌ **NON** (recréée à chaque appel)
- **Justification** : Durée de vie très courte, pas besoin d'optimisation

**Ligne 267** : `disconnected = []`
- **Contexte** : Liste temporaire dans broadcast()
- **Devrait être deque ?** : ❌ **NON** (locale à la fonction)
- **Justification** : Variable locale, durée de vie minimale

**Ligne 2340** : `const toastQueue = [];` (JavaScript)
- **Contexte :** File d'attente toast en frontend
- **Devrait être deque ?** : ❌ **NON** (JavaScript, pas Python)

**Problèmes identifiés :**
- ✅ **Aucun problème** : deque utilisé correctement pour l'historique
- ✅ **Choix appropriés** : list utilisé là où c'est pertinent

**Score :** 10/10

---

### Action 8.2 : Chercher les boucles bloquantes

**RÉSULTAT :**
| Fichier | Ligne | Code | Bloquant ? | Problème |
|---------|-------|------|------------|----------|
| dashboard_advanced.py | 377 | `while not self._stop_metrics:` | NON | ✅ OK |
| dashboard_advanced.py | 3092 | `while True:` | ❌ OUI | ❌ BLOQUANT |
| dashboard_advanced.py | 3155 | `while True:` | NON | ✅ OK |

**Analyse détaillée :**

**Ligne 377** : `while not self._stop_metrics:`
```python
async def collect_metrics():
    while not self._stop_metrics:
        try:
            # FAIRE AVANCER LA SIMULATION MuJoCo si robot connecté
            if self.robot and hasattr(self.robot, "step"):
                self.robot.step()
            
            # Mettre à jour les métriques
            await asyncio.sleep(0.1)  # ✅ AWAIT PRÉSENT
```
- **Condition de sortie** : ✅ `self._stop_metrics` flag
- **await asyncio.sleep(0.1)** : ✅ **NON BLOQUANT**
- **Gestion propre** : ✅ Boucle async avec pause

**Ligne 3092** : `while True:`
```python
def video_stream():
    # ...
    while True:
        try:
            frame = None
            if vision:
                frame = vision._capture_image_from_camera()
            
            # Envoyer frame
            # ...
            time.sleep(0.033)  # ❌ PAS D'AWAIT - FONCTION SYNCHRONE
```
- **Fonction synchrone** : ❌ **BLOQUANTE**
- **time.sleep(0.033)** : ❌ **BLOQUE LE THREAD**
- **Pas de mécanisme de sortie** : ❌ **BOUCLE INFINIE**
- **Problème critique** : Bloque un thread entier indéfiniment

**Ligne 3155** : `while True:`
```python
async def websocket_endpoint(websocket: WebSocket):
    try:
        while True:
            data = await websocket.receive_text()  # ✅ AWAIT
            message = json.loads(data)
            # ...
```
- **await websocket.receive_text()** : ✅ **NON BLOQUANT**
- **WebSocket natif async** : ✅ Géré par FastAPI
- **Sortie propre** : ✅ Exception sur déconnexion

**Problèmes identifiés :**
- ❌ **video_stream() BLOQUANT** : Utilise time.sleep() dans boucle infinie
- ❌ **Pas de sortie** : Pas de mécanisme pour arrêter le stream vidéo
- ❌ **Resource leak** : Thread bloqué indéfiniment

**Score :** 4/10

---

### Action 8.3 : Chercher les `@lru_cache` manquants

**RÉSULTAT :**
| Fonction | Fichier | Ligne | Devrait être cachée ? |
|----------|---------|-------|---------------------|
| `get_available_joints` | reachy_mini_backend.py | 393 | ✅ OUI |
| `get_joint_pos` | reachy_mini_backend.py | 397 | ❌ NON |
| `get_current_head_pose` | reachy_mini_backend.py | 853 | ❌ NON |
| `get_current_body_yaw` | reachy_mini_backend.py | 869 | ❌ NON |

**Analyse détaillée :**

**`get_available_joints()` (ligne 393) :**
```python
def get_available_joints(self) -> list[str]:
    """Retourne la liste des joints disponibles."""
    return list(self.joint_mapping.keys())
```
- **Fonction pure** : ✅ **OUI** (pas d'effets de bord)
- **Retour constant** : ✅ **OUI** (mapping ne change pas)
- **Appel fréquent** : ✅ **OUI** (utilisé dans plusieurs endpoints)
- **Devrait être cachée** : ✅ **OUI** pour performance

**`get_joint_pos()` (ligne 397) :**
- **Accède hardware** : ❌ **NON** (état robot change)
- **Temps réel** : ❌ **NON** (valeurs dynamiques)

**`get_current_head_pose()` (ligne 853) :**
- **Accède hardware** : ❌ **NON** (pose robot change)
- **Temps réel** : ❌ **NON** (valeurs dynamiques)

**`get_current_body_yaw()` (ligne 869) :**
- **Accède hardware** : ❌ **NON** (rotation robot change)
- **Temps réel** : ❌ **NON** (valeurs dynamiques)

**Problèmes identifiés :**
- ❌ **get_available_joints non cachée** : Appelée fréquemment pour résultat constant
- ❌ **Performance perdue** : Recalcul à chaque appel

**Score :** 7/10

---

## 📈 SCORE GLOBAL PHASE 8

| Action | Score | Poids | Score pondéré |
|--------|-------|--------|---------------|
| 8.1 deque vs list | 10/10 | 30% | 3.0/3 |
| 8.2 boucles bloquantes | 4/10 | 40% | 1.6/4 |
| 8.3 lru_cache manquants | 7/10 | 30% | 2.1/3 |
| **TOTAL** | | **100%** | **6.7/10** |

## 🎯 CONCLUSION PHASE 8

**POINTS FORTS :**
- ✅ **deque utilisé correctement** pour l'historique des métriques
- ✅ **Choix appropriés** entre list et deque selon les cas
- ✅ **Boucles async** bien implémentées (WebSocket, metrics)

**POINTS FAIBLES :**
- ❌ **video_stream() BLOQUANT** : Thread bloqué indéfiniment
- ❌ **Pas de cache** pour get_available_joints (appel fréquent)
- ❌ **Resource leak** : Boucle infinie sans mécanisme d'arrêt

**ACTIONS PRIORITAIRES :**
1. **CRITIQUE** : Rendre video_stream() async avec await asyncio.sleep()
2. **CRITIQUE** : Ajouter mécanisme d'arrêt pour video_stream()
3. **IMPORTANT** : Ajouter @lru_cache à get_available_joints()
4. **RECOMMANDÉ** : Limiter durée de vie du thread vidéo

**QUALITÉ GLOBALE :** MOYENNE (6.7/10)

