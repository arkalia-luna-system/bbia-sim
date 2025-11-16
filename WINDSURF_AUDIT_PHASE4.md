# 🔍 AUDIT BBIA-SIM - PHASE 4 : TESTS ET COUVERTURE

## ⚠️ RÈGLES ABSOLUES

- **NE MODIFIE AUCUN FICHIER**
- **Analyse statique uniquement**
- **Ouvre les fichiers et lis-les ligne par ligne** (ne pas utiliser grep)

---

## 🎯 OBJECTIF

Évaluer la stratégie de tests et identifier les zones non couvertes

**MÉTHODE :** Ouvre chaque fichier, lis-le complètement, analyse ligne par ligne

---

## 📋 ACTIONS À EXÉCUTER (3 actions)

### Action 4.1 : Vérifier la couverture par module

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/backends/reachy_mini_backend.py` (note le nom)
2. **Ouvre** `src/bbia_sim/backends/mujoco_backend.py` (note le nom)
3. **Ouvre** le dossier `tests/` et **liste** tous les fichiers `test_*.py`
4. **Pour chaque fichier backend**, vérifie s'il existe un `test_<nom_fichier>.py` correspondant
5. Exemple : `reachy_mini_backend.py` → cherche `test_reachy_mini_backend.py` dans la liste

**RÉSULTAT ATTENDU :**
| Module | Test existe ? | Nom du test | Couverture estimée |
|--------|---------------|-------------|-------------------|
| backends/reachy_mini_backend.py | OUI | test_reachy_mini_backend.py | ?% |
| backends/mujoco_backend.py | ? | ? | ?% |

---

### Action 4.2 : Vérifier la qualité des tests

**INSTRUCTION SIMPLE :**
1. **Ouvre** `tests/test_reachy_mini_backend.py`
2. **Lis** le fichier complètement ligne par ligne
3. **Pour chaque ligne** qui commence par `def test_` :
   - Note le numéro de ligne et le nom du test
   - **Lis** le corps de la fonction (jusqu'à la prochaine `def`)
   - Vérifie si la fonction contient au moins un `assert`
   - Vérifie s'il y a un `try/except` qui masque les erreurs (ex: `except: pass`)

**RÉSULTAT ATTENDU :**
| Test | Ligne | Assertions ? | Problème |
|------|-------|--------------|----------|
| `test_connect` | ? | OUI/NON | ? |

---

### Action 4.3 : Identifier les tests manquants critiques

**INSTRUCTION SIMPLE :**
1. **Ouvre** le dossier `tests/` et **liste** tous les fichiers `test_*.py`
2. **Vérifie** si ces tests existent :
   - `test_bridge.py` ou `test_daemon_bridge.py` (pour `daemon/bridge.py`)
   - `test_reachy_mini_backend.py` (pour `backends/reachy_mini_backend.py`)
3. **Si un test n'existe pas**, note-le comme manquant

**RÉSULTAT ATTENDU :**
| Module | Test manquant | Priorité | Scénario |
|--------|---------------|----------|----------|
| daemon/bridge.py | Test reconnexion Zenoh | Haute | Perte connexion réseau |

---

## 🎨 FORMAT DE RÉPONSE

Pour chaque action :
- **Résultat** : Tableau
- **Problèmes** : Liste
- **Score** : X/10

---

## ⚠️ IMPORTANT : MÉTHODE D'ANALYSE

**NE PAS UTILISER grep ou recherche dans tout le projet**

**MÉTHODE CORRECTE :**
1. Utilise `list_dir` pour lister les fichiers dans `tests/`
2. Utilise `read_file` pour ouvrir chaque fichier
3. Lis le fichier complètement ligne par ligne
4. Analyse dans ta mémoire

---

## 🚀 COMMENCE MAINTENANT

**Exécute les 3 actions dans l'ordre et rapporte les résultats.**

