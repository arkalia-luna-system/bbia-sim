# 🔍 AUDIT BBIA-SIM - PHASE 5 : SIMULATION MUJOCO

## ⚠️ RÈGLES ABSOLUES

- **NE MODIFIE AUCUN FICHIER**
- **Analyse statique uniquement**
- **Ouvre les fichiers et lis-les ligne par ligne** (ne pas utiliser grep)

---

## 🎯 OBJECTIF

Audit de l'intégration MuJoCo et optimisation de la simulation

**MÉTHODE :** Ouvre chaque fichier, lis-le complètement, analyse ligne par ligne

---

## 📋 ACTIONS À EXÉCUTER (3 actions)

### Action 5.1 : Comparer les modèles XML

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/sim/models/reachy_mini.xml`
2. **Lis** le fichier complètement
3. **Compte** le nombre de balises `<joint` (nombre de joints)
4. **Note** les balises `<mass` (masses des corps)
5. **Répète** pour `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`
6. **Compare** les deux fichiers

**RÉSULTAT ATTENDU :**
| Propriété | reachy_mini.xml | REAL_OFFICIAL.xml | Différence |
|-----------|-----------------|-------------------|------------|
| Nombre joints | ? | ? | ? |
| Masses | ? | ? | ? |

---

### Action 5.2 : Analyser la performance de simulation

**INSTRUCTION :**
1. Ouvre `src/bbia_sim/sim/simulator.py`
2. Cherche les fonctions : `step()`, `render()`
3. Identifie les calculs redondants (IK/FK calculés plusieurs fois)

**RÉSULTAT ATTENDU :**
| Fonction | Ligne | Calculs redondants ? | Optimisation possible |
|----------|-------|----------------------|----------------------|
| `step` | ? | ? | ? |

---

### Action 5.3 : Vérifier la cohérence sim vs réel

**INSTRUCTION SIMPLE :**
1. **Ouvre** `src/bbia_sim/backends/mujoco_backend.py`
2. **Lis** le fichier ligne par ligne
3. **Pour chaque fonction** `goto_target`, `get_joint_pos`, `get_image` :
   - Note la signature complète (paramètres)
4. **Ouvre** `src/bbia_sim/backends/reachy_mini_backend.py`
5. **Lis** le fichier ligne par ligne
6. **Pour chaque fonction** correspondante :
   - Note la signature complète
   - **Compare** avec la version mujoco_backend

**RÉSULTAT ATTENDU :**
| Méthode | mujoco_backend | reachy_mini_backend | Cohérent ? |
|---------|----------------|---------------------|------------|
| `goto_target` | `(head, duration)` | `(head, duration)` | ? |

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

