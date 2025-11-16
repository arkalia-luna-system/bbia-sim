# 🔍 AUDIT BBIA-SIM - PHASE 3 : QUALITÉ CODE PYTHON

## ⚠️ RÈGLES ABSOLUES

- **NE MODIFIE AUCUN FICHIER**
- **Analyse statique uniquement**
- **Ouvre les fichiers et lis-les ligne par ligne** (ne pas utiliser grep)

---

## 🎯 OBJECTIF

Audit de la qualité du code selon les standards industriels

**MÉTHODE :** Ouvre chaque fichier, lis-le complètement, analyse ligne par ligne

---

## 📋 ACTIONS À EXÉCUTER (4 actions)

### Action 3.1 : Compter les fonctions sans type hints

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/backends/reachy_mini_backend.py`
2. **Lis** le fichier de la ligne 1 à la ligne 715
3. **Pour chaque ligne** qui commence par `def ` :
   - Note le numéro de ligne
   - Note le nom de la fonction
   - Vérifie si la ligne contient `->` (type hint de retour)
   - Si oui : ✅ Avec type hint
   - Si non : ❌ Sans type hint
4. **Compte** : total fonctions, avec type hint, sans type hint

**EXEMPLE CONCRET :**
Ligne 132 : `def __init__(self, ...) -> None:` ✅ (contient `-> None`)
Ligne 200 : `def connect(self) -> bool:` ✅ (contient `-> bool`)
Ligne 250 : `def some_function(self):` ❌ (pas de `->`)

**FICHIERS À ANALYSER (dans l'ordre) :**
1. `src/bbia_sim/backends/reachy_mini_backend.py` (715 lignes)
2. `src/bbia_sim/daemon/bridge.py` (388 lignes)

**RÉSULTAT ATTENDU :**
| Fichier | Fonctions totales | Avec type hint | Sans type hint | % avec hints |
|---------|------------------|----------------|----------------|---------------|
| reachy_mini_backend.py | ? | ? | ? | ?% |
| bridge.py | ? | ? | ? | ?% |

---

### Action 3.2 : Chercher les fonctions trop longues

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/backends/reachy_mini_backend.py`
2. **Lis** le fichier ligne par ligne
3. **Pour chaque fonction** (ligne commençant par `def `) :
   - Note la ligne de début (ex: ligne 600)
   - Note le nom de la fonction
   - **Compte les lignes** jusqu'à la prochaine ligne `def ` ou `class `
   - Si > 50 lignes : ❌ Fonction trop longue
4. **Liste** toutes les fonctions > 50 lignes

**EXEMPLE CONCRET :**
```
Ligne 600 : def goto_target(self, ...):
Ligne 601 :     # code
Ligne 602 :     # code
...
Ligne 680 :     # fin de la fonction
Ligne 681 : def autre_fonction(self):  # ← prochaine fonction
```
Si ligne 680 - ligne 600 = 80 lignes → ❌ Trop long (> 50)

**RÉSULTAT ATTENDU :**
| Fonction | Ligne début | Ligne fin | Nombre lignes | Problème |
|----------|-------------|-----------|---------------|----------|
| `goto_target` | 600 | 680 | 80 | > 50 lignes |
| `connect` | ? | ? | ? | ? |

---

### Action 3.3 : Chercher les `Any` utilisés

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/backends/reachy_mini_backend.py`
2. **Lis** le fichier ligne par ligne
3. **Pour chaque ligne** qui contient le mot `Any` :
   - Note le numéro de ligne
   - Copie la ligne complète
   - Vérifie le contexte (import conditionnel ? type hint ?)
4. **Répète** pour `src/bbia_sim/daemon/bridge.py`

**EXEMPLES À CHERCHER :**
- Ligne contenant `: Any` (type hint)
- Ligne contenant `Any |` (union type Python 3.10+)
- Ligne contenant `cast(Any,` (type casting)

**EXEMPLE CONCRET :**
Ligne 39 dans `bridge.py` :
```python
ReachyMini = cast(Any, None)  # Import conditionnel - Acceptable
```

**RÉSULTAT ATTENDU :**
| Fichier | Ligne | Code complet | Contexte | Acceptable ? |
|---------|-------|--------------|----------|--------------|
| bridge.py | 39 | `ReachyMini = cast(Any, None)` | Import conditionnel | ✅ OUI |
| ? | ? | ? | ? | ? |

---

### Action 3.4 : Chercher les imports inutilisés (SIMPLIFIÉE)

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/backends/reachy_mini_backend.py`
2. **Lis** les lignes 1-30 (section imports uniquement)
3. **Note** tous les imports trouvés dans un tableau
4. **Lis** le reste du fichier (lignes 31-715)
5. **Pour chaque import** noté :
   - Extrais le nom principal (ex: `ReachyMini` de `from reachy_mini import ReachyMini`)
   - **Cherche** ce nom dans le reste du fichier (lignes 31-715)
   - Si le nom apparaît au moins une fois : ✅ Utilisé
   - Si le nom n'apparaît jamais : ❌ Potentiellement inutilisé
6. **Répète** pour `src/bbia_sim/daemon/bridge.py` (lignes 1-20 pour imports, 21-388 pour usage)

**ATTENTION :**
- Ne compte PAS les occurrences dans les commentaires ou docstrings
- Ne compte PAS les occurrences dans les chaînes de caractères (`"ReachyMini"`)
- Compte SEULEMENT les utilisations réelles du nom (variables, fonctions, classes)

**EXEMPLE CONCRET :**
Ligne 15 : `from reachy_mini import ReachyMini`
- Nom à chercher : `ReachyMini`
- Cherche `ReachyMini` dans les lignes 31-715 (hors commentaires/strings)
- Si trouvé ligne 204 : `self.robot = ReachyMini(...)` → ✅ Utilisé
- Si jamais trouvé → ❌ Potentiellement inutilisé

**RÉSULTAT ATTENDU :**
| Fichier | Ligne | Import | Nom cherché | Utilisé ? | Action |
|---------|-------|--------|-------------|-----------|--------|
| reachy_mini_backend.py | 15 | `from reachy_mini import ReachyMini` | ReachyMini | ✅ OUI | Garder |
| ? | ? | `from x import y` | y | ❌ NON | Vérifier |

---

## 🎨 FORMAT DE RÉPONSE

Pour chaque action :
- **Résultat** : Tableau
- **Exemples** : Code avec lignes
- **Problèmes** : Liste
- **Score** : X/10

---

## ⚠️ IMPORTANT : MÉTHODE D'ANALYSE

**NE PAS UTILISER grep ou recherche dans tout le projet**

**MÉTHODE CORRECTE :**
1. Utilise l'outil `read_file` pour ouvrir chaque fichier
2. Lis le fichier complètement (toutes les lignes)
3. Analyse ligne par ligne dans ta mémoire
4. Note les résultats au fur et à mesure

**EXEMPLE :**
```
1. read_file("src/bbia_sim/backends/reachy_mini_backend.py")
2. Lis toutes les lignes de 1 à 715
3. Pour chaque ligne qui commence par "def ", note-la
4. Vérifie si cette ligne contient "->"
```

**ÉVITE :**
- ❌ Chercher "def " dans tout le projet (grep)
- ❌ Utiliser des commandes de recherche complexes
- ❌ Chercher plusieurs patterns en même temps

**FAIS :**
- ✅ Ouvre un fichier à la fois
- ✅ Lis-le complètement
- ✅ Analyse ligne par ligne
- ✅ Note les résultats

---

## 🚀 COMMENCE MAINTENANT

**Exécute les 4 actions dans l'ordre :**
1. Action 3.1 : Type hints (2 fichiers) - **Lis chaque fichier complètement**
2. Action 3.2 : Fonctions longues (1 fichier) - **Compte les lignes entre chaque `def`**
3. Action 3.3 : Usage de Any (2 fichiers) - **Cherche le mot "Any" ligne par ligne**
4. Action 3.4 : Imports inutilisés (2 fichiers) - **Compare imports vs usage dans le fichier**

**IMPORTANT :**
- Pour l'Action 3.4, si tu n'es pas sûr qu'un import est utilisé, note-le comme "À vérifier" plutôt que "Inutilisé"
- Il vaut mieux être prudent et ne pas marquer un import comme inutilisé s'il y a un doute

**Rapporte les résultats pour chaque action.**

