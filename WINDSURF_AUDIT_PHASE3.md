# 🔍 AUDIT BBIA-SIM - PHASE 3 : QUALITÉ CODE PYTHON

## ⚠️ RÈGLES ABSOLUES

- **NE MODIFIE AUCUN FICHIER**
- **Analyse statique uniquement**

---

## 🎯 OBJECTIF

Audit de la qualité du code selon les standards industriels

---

## 📋 ACTIONS À EXÉCUTER (4 actions)

### Action 3.1 : Compter les fonctions sans type hints

**INSTRUCTION :**
1. Pour chaque fichier Python dans `src/bbia_sim/`
2. Cherche les définitions : `def nom_fonction(`
3. Vérifie si la fonction a `-> ReturnType`
4. Compte : avec type hint / total

**FICHIERS PRIORITAIRES :**
- `src/bbia_sim/backends/reachy_mini_backend.py` (715 lignes)
- `src/bbia_sim/daemon/bridge.py` (388 lignes)

**EXEMPLE :**
Ligne 132 : `def __init__(self, ...) -> None:` ✅
Ligne 200 : `def connect(self) -> bool:` ✅

**RÉSULTAT ATTENDU :**
| Fichier | Fonctions totales | Avec type hint | % | Problème |
|---------|-------------------|----------------|---|----------|
| reachy_mini_backend.py | ? | ? | ?% | ? |

---

### Action 3.2 : Chercher les fonctions trop longues

**INSTRUCTION :**
1. Pour chaque fonction dans `src/bbia_sim/backends/reachy_mini_backend.py`
2. Compte les lignes entre `def` et le prochain `def` ou `class`
3. Liste les fonctions > 50 lignes

**RÉSULTAT ATTENDU :**
| Fonction | Ligne début | Lignes | Problème |
|----------|-------------|--------|----------|
| `goto_target` | 600 | ? | > 50 lignes ? |

---

### Action 3.3 : Chercher les `Any` utilisés

**INSTRUCTION :**
1. Cherche EXACTEMENT : `: Any` dans TOUT le projet
2. Cherche EXACTEMENT : `Any |` dans TOUT le projet
3. Note : fichier, ligne, contexte

**RÉSULTAT ATTENDU :**
| Fichier | Ligne | Code | Contexte | Problème |
|---------|-------|------|----------|----------|
| bridge.py | 39 | `ReachyMini = cast(Any, None)` | Import conditionnel | Acceptable |

---

### Action 3.4 : Chercher les imports inutilisés

**INSTRUCTION :**
1. Pour chaque fichier Python, liste TOUS les imports
2. Pour chaque import, vérifie si la fonction/classe est utilisée
3. Identifie les imports jamais appelés

**RÉSULTAT ATTENDU :**
| Fichier | Import inutilisé | Ligne | Action |
|---------|------------------|-------|--------|
| ? | `from x import y` | ? | Supprimer |

---

## 🎨 FORMAT DE RÉPONSE

Pour chaque action :
- **Résultat** : Tableau
- **Exemples** : Code avec lignes
- **Problèmes** : Liste
- **Score** : X/10

---

## 🚀 COMMENCE MAINTENANT

**Exécute les 4 actions et rapporte les résultats.**

