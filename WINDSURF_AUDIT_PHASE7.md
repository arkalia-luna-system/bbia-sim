# 🔍 AUDIT BBIA-SIM - PHASE 7 : COMMUNICATION (ZENOH/REST/WS)

## ⚠️ RÈGLES ABSOLUES

- **NE MODIFIE AUCUN FICHIER**
- **Analyse statique uniquement**

---

## 🎯 OBJECTIF

Analyser la communication Zenoh, API REST et WebSocket

---

## 📋 ACTIONS À EXÉCUTER (3 actions)

### Action 7.1 : Analyser le bridge Zenoh ligne par ligne

**INSTRUCTION :**
1. Ouvre `src/bbia_sim/daemon/bridge.py`
2. Lis TOUTES les lignes de 1 à 388
3. Pour chaque utilisation de `zenoh`, note : ligne, code exact, type d'opération

**PATTERNS EXACTS À CHERCHER :**
- `zenoh.open(` ou `Session.open(`
- `session.declare_publisher(`
- `session.declare_subscriber(`

**RÉSULTAT ATTENDU :**
| Ligne | Code | Type | Problème |
|-------|------|------|----------|
| ? | `zenoh.open(...)` | open | ? |

---

### Action 7.2 : Vérifier les endpoints REST

**INSTRUCTION :**
1. Ouvre `src/bbia_sim/daemon/app/routers/state.py`
2. Cherche les décorateurs : `@app.get(` ou `@router.get(`
3. Compare avec l'endpoint officiel : `/api/state/full`

**RÉSULTAT ATTENDU :**
| Endpoint | Fichier | Ligne | Conforme officiel ? |
|----------|---------|-------|---------------------|
| `/api/state/full` | state.py | ? | ? |

---

### Action 7.3 : Chercher les fuites WebSocket

**INSTRUCTION :**
1. Ouvre `src/bbia_sim/dashboard_advanced.py`
2. Cherche la classe `BBIAAdvancedWebSocketManager` (lignes ~49-460)
3. Vérifie les fonctions : `disconnect()`, `_cleanup_inactive_connections()`
4. Identifie les connexions non fermées

**RÉSULTAT ATTENDU :**
| Fonction | Ligne | Ferme connexion ? | Problème |
|----------|-------|-------------------|----------|
| `disconnect` | ? | OUI/NON | ? |

---

## 🎨 FORMAT DE RÉPONSE

Pour chaque action :
- **Résultat** : Tableau
- **Problèmes** : Liste
- **Score** : X/10

---

## 🚀 COMMENCE MAINTENANT

**Exécute les 3 actions et rapporte les résultats.**

