# 🔍 AUDIT BBIA-SIM - PHASE 2B : MICRO-DÉTAILS CRITIQUES

## ⚠️ RÈGLES ABSOLUES

- **NE MODIFIE AUCUN FICHIER**
- **Analyse statique uniquement**

---

## 🎯 OBJECTIF

Identifier les petits détails qui causent des bugs subtils mais critiques

---

## 📋 ACTIONS À EXÉCUTER (4 actions)

### Action 2B.1 : Chercher les exceptions silencieuses

**INSTRUCTION :**
1. Ouvre `src/bbia_sim/backends/reachy_mini_backend.py`
2. Cherche EXACTEMENT : `except Exception as e:`
3. Pour chaque occurrence, vérifie :
   - Y a-t-il un `logger.error()` ou `logger.warning()` après ?
   - Y a-t-il un `pass` sans log ?

**PATTERNS EXACTS À CHERCHER :**
- `except Exception:` suivi de `pass`
- `except Exception as e:` sans `logger.`
- `except:` (bare except)

**EXEMPLE TROUVÉ :**
Ligne 239 :
```python
except Exception as e:
    logger.warning(f"Erreur: {e}")  # ✅ Loggé
```

**RÉSULTAT ATTENDU :**
| Ligne | Code | Problème | Impact |
|-------|------|----------|--------|
| 430   | `except (AttributeError, Exception):` | Pas de log | Moyen |

---

### Action 2B.2 : Chercher les timeouts manquants

**INSTRUCTION :**
1. Cherche EXACTEMENT : `time.sleep(` dans TOUT le projet
2. Cherche EXACTEMENT : `timeout=` dans TOUT le projet
3. Pour chaque `time.sleep()`, vérifie si c'est dans une boucle `while True`

**RÉSULTAT ATTENDU :**
| Fichier | Ligne | Code | Dans boucle ? | Timeout global ? |
|---------|-------|------|---------------|------------------|
| dashboard_advanced.py | 397 | `await asyncio.sleep(0.1)` | OUI | NON |

---

### Action 2B.3 : Chercher les context managers manquants

**INSTRUCTION :**
1. Cherche EXACTEMENT : `ReachyMini(` (instanciation)
2. Vérifie si c'est dans un `with` statement
3. Cherche `with ReachyMini(`

**EXEMPLE OFFICIEL :**
```python
with ReachyMini() as reachy_mini:
    # code
```

**EXEMPLE TROUVÉ DANS BBIA :**
Ligne 204 :
```python
self.robot = ReachyMini(...)  # ❌ PAS de with
```

**RÉSULTAT ATTENDU :**
| Ligne | Code | Avec `with` ? | Problème |
|-------|------|---------------|----------|
| 204   | `self.robot = ReachyMini(...)` | NON | Ressource non fermée |

---

### Action 2B.4 : Chercher les validations manquantes

**INSTRUCTION :**
1. Ouvre `src/bbia_sim/backends/reachy_mini_backend.py`
2. Cherche les fonctions publiques (pas `_private`)
3. Pour chaque fonction, vérifie :
   - Validation des paramètres ?
   - Gestion de `None` ?
   - Vérification de type ?

**FONCTIONS À VÉRIFIER :**
- `goto_target()` - ligne ~600
- `get_joint_pos()` - ligne ~?
- `set_joint_pos()` - ligne ~?

**RÉSULTAT ATTENDU :**
| Fonction | Ligne | Validation ? | Gestion None ? | Problème |
|----------|-------|--------------|----------------|----------|
| `goto_target` | 600 | ? | ? | ? |

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

