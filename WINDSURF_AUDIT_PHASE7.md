# 🔍 AUDIT BBIA-SIM - PHASE 7 : COMMUNICATION (ZENOH/REST/WS)

## ⚠️ RÈGLES ABSOLUES

- **NE MODIFIE AUCUN FICHIER**
- **Analyse statique uniquement**
- **Ouvre les fichiers et lis-les ligne par ligne** (ne pas utiliser grep)

---

## 🎯 OBJECTIF

Analyser la communication Zenoh, API REST et WebSocket

**MÉTHODE :** Ouvre chaque fichier, lis-le complètement, analyse ligne par ligne

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

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/daemon/app/routers/state.py`
2. **Lis** le fichier complètement ligne par ligne
3. **Pour chaque ligne** qui contient `@app.get(` ou `@router.get(` :
   - Note le numéro de ligne
   - Copie la ligne complète (décorateur)
   - **Lis** la fonction suivante pour voir le chemin de l'endpoint
   - Compare avec l'endpoint officiel : `/api/state/full`

**RÉSULTAT ATTENDU :**
| Endpoint | Fichier | Ligne | Conforme officiel ? |
|----------|---------|-------|---------------------|
| `/api/state/full` | state.py | ? | ? |

---

### Action 7.3 : Chercher les fuites WebSocket

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/dashboard_advanced.py`
2. **Lis** les lignes 49 à 460 (classe `BBIAAdvancedWebSocketManager`)
3. **Pour chaque ligne** qui contient `def disconnect(` ou `def _cleanup_inactive_connections(` :
   - Note le numéro de ligne
   - **Lis** le corps de la fonction
   - Vérifie si la fonction ferme les connexions WebSocket (close(), await websocket.close(), etc.)

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

## ⚠️ IMPORTANT : MÉTHODE D'ANALYSE

**NE PAS UTILISER grep**

**MÉTHODE CORRECTE :**
1. Utilise `read_file` pour ouvrir chaque fichier
2. Lis le fichier complètement (ou la section demandée)
3. Analyse ligne par ligne dans ta mémoire

---

## 🚀 COMMENCE MAINTENANT

**Exécute les 3 actions dans l'ordre et rapporte les résultats.**

