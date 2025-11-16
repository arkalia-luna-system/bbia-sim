# 🔍 AUDIT BBIA-SIM - PHASE 5 : SIMULATION MUJOCO

## ⚠️ RÈGLES ABSOLUES

- **NE MODIFIE AUCUN FICHIER**
- **Analyse statique uniquement**

---

## 🎯 OBJECTIF

Audit de l'intégration MuJoCo et optimisation de la simulation

---

## 📋 ACTIONS À EXÉCUTER (3 actions)

### Action 5.1 : Comparer les modèles XML

**INSTRUCTION :**
1. Ouvre `src/bbia_sim/sim/models/reachy_mini.xml`
2. Ouvre `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`
3. Compare :
   - Nombre de joints
   - Masses des corps
   - Propriétés des geoms

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

**INSTRUCTION :**
1. Compare `mujoco_backend.py` vs `reachy_mini_backend.py`
2. Vérifie si les méthodes ont la même signature :
   - `goto_target()` : même signature ?
   - `get_joint_pos()` : même comportement ?
   - `get_image()` : même format ?

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

## 🚀 COMMENCE MAINTENANT

**Exécute les 3 actions et rapporte les résultats.**

