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

**RÉSULTAT ATTENDU :**
| Fichier | Ligne | Code | Devrait être deque ? |
|---------|-------|------|---------------------|
| ? | ? | `buffer = []` | OUI/NON |

---

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

## 🚀 COMMENCE MAINTENANT

**Exécute les 3 actions dans l'ordre et rapporte les résultats.**

