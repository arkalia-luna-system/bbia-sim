# Optimisations expert - SDK Reachy Mini

> Référence performance
>
> Voir `docs/reference/project-status.md` → "Performance" pour l’état actuel, les tests présents (latence/jitter), et les baselines/axes futurs.

**Date :** Oct / Nov. 2025
**Référence :** SDK officiel GitHub https://github.com/pollen-robotics/reachy_mini

---

## Objectif

Optimisations appliquées à BBIA‑SIM basées sur les pratiques du SDK officiel Reachy Mini.

---

## Optimisations appliquées

### **1. Utilisation de `goto_target()` au lieu de `set_joint_pos()` répétés**

**Problème détecté :**
Les mouvements utilisant plusieurs `set_joint_pos()` successifs créent des mouvements saccadés et moins fluides.

**Solution :** utiliser `goto_target()` avec interpolation automatique (`method="minjerk"`) pour des mouvements plus fluides et synchronisés.

**Fichiers concernés :**

- `bbia_behavior.py` - WakeUpBehavior, GreetingBehavior, AntennaAnimationBehavior, HideBehavior
- `bbia_integration.py` - apply_emotion_to_robot(), sync_voice_with_movements()

**Exemple avant :**

```python 🐍
self.robot_api.set_joint_pos("yaw_body", 0.15)
time.sleep(0.5)
self.robot_api.set_joint_pos("yaw_body", -0.15)
time.sleep(0.5)
self.robot_api.set_joint_pos("yaw_body", 0.0)

```

**Exemple après :**

```python 🐍
# Mouvement fluide avec interpolation automatique
self.robot_api.goto_target(
    body_yaw=0.15,
    duration=0.8,
    method="minjerk"
)
time.sleep(1.0)
self.robot_api.goto_target(
    body_yaw=-0.15,
    duration=0.8,
    method="minjerk"
)

```

---

### **2. Mouvements combinés tête + corps avec `goto_target()`**

**Problème détecté :**
Les mouvements de tête et corps appliqués séparément créent une désynchronisation visuelle.

**Solution :** combiner tête et corps dans un seul appel `goto_target()` pour une meilleure synchronisation.

**Fichiers concernés :**

- `bbia_integration.py` - apply_emotion_to_robot()
- `bbia_behavior.py` - HideBehavior

**Exemple avant :**

```python 🐍
self.robot_api.set_emotion("happy", 0.6)
self.robot_api.set_joint_pos("yaw_body", 0.1)

```

**Exemple après :**

```python 🐍
pose = create_head_pose(pitch=0.08, yaw=0.0, degrees=False)
self.robot_api.goto_target(
    head=pose,
    body_yaw=0.1,
    duration=0.6,
    method="minjerk"
)

```

---

### **3. Utilisation de `look_at_world()` et `look_at_image()` pour suivi visuel**

**Problème détecté :**
Le suivi visuel utilisait directement `set_joint_position()` sans utiliser les capacités SDK avancées.

**Solution :** utiliser `look_at_world()` pour les positions 3D et `look_at_image()` pour les coordonnées pixel avec validation des limites.

**Fichiers concernés :**

- `bbia_behavior.py` - VisionTrackingBehavior
- `bbia_integration.py` - react_to_vision_detection()

**Exemple avant ❌:**

```python 🐍
head_turn = face_position[0] * 0.3
self.simulation_service.set_joint_position("yaw_body", head_turn)

```

**Exemple après ✅:**

```python 🐍
# Validation et utilisation SDK optimisée
if hasattr(robot_api, 'look_at_world'):
    pos_3d = face_data.get("position_3d", {})
    if pos_3d:
        x, y, z = float(pos_3d.get("x")), float(pos_3d.get("y")), float(pos_3d.get("z"))
        if -2.0 <= x <= 2.0 and -2.0 <= y <= 2.0 and -1.0 <= z <= 1.0:
            robot_api.look_at_world(x, y, z, duration=1.0, perform_movement=True)

```

---

### **4. Intégration RobotAPI dans BBIABehaviorManager**

**Problème détecté :**
Les comportements BBIA ne pouvaient pas contrôler directement le robot physique.

**Solution :**
Passer `robot_api` au `BBIABehaviorManager` pour contrôle direct du robot via SDK.

**Fichiers corrigés :**

- ✅ `bbia_integration.py` - __init__() passe robot_api au BBIABehaviorManager
- ✅ `bbia_behavior.py` - Tous les comportements acceptent et utilisent robot_api

**Impact :**

- ✅ Contrôle direct du robot par les comportements
- ✅ Utilisation automatique des optimisations SDK
- ✅ Fallbacks gracieux si SDK non disponible

---

### **5. Gestion d'erreurs avec fallbacks**

**Problème détecté :**
Erreurs non gérées si SDK non disponible ou méthodes manquantes.

**Solution :**
Système de fallbacks à 3 niveaux : SDK optimisé → SDK basique → Simulation.

**Exemple de pattern :**

```python 🐍
try:
    # Méthode 1 (préférée): goto_target avec interpolation
    if hasattr(robot_api, 'goto_target'):
        robot_api.goto_target(...)
    # Méthode 2 (fallback): set_target_head_pose
    elif hasattr(robot_api, 'set_target_head_pose'):
        robot_api.set_target_head_pose(...)
    # Méthode 3 (fallback final): set_joint_pos
    else:
        robot_api.set_joint_pos(...)
except (ValueError, RuntimeError, ConnectionError) as e:
    logger.warning(f"Erreur (fallback): {e}")
    # Fallback final vers simulation

```

---

## Bénéfices performance

### **Fluidité des mouvements**

- Avant : mouvements saccadés (3-5 appels SDK séparés)
- Après : mouvements plus fluides (1 appel SDK avec interpolation)

### **Synchronisation**

- Avant : tête et corps désynchronisés (2 appels séparés)
- Après : synchronisation améliorée (1 appel combiné)

### **Précision suivi visuel**

- Avant : rotation approximative (set_joint_pos direct)
- Après : utilisation IK via SDK (`look_at_world`)

### **Résilience**

- Avant : erreurs si SDK non disponible
- Après : fallbacks pour maintenir le fonctionnement

---

## Fichiers modifiés

1. `src/bbia_sim/bbia_behavior.py`
   - WakeUpBehavior : goto_target pour réveil fluide
   - GreetingBehavior : goto_target pour hochement fluide
   - VisionTrackingBehavior : look_at_world/look_at_image
   - AntennaAnimationBehavior : goto_target pour expressivité (✅ Antennes animables avec limites -0.3 à 0.3 rad)
   - HideBehavior : goto_target combiné tête+corps

2. `src/bbia_sim/bbia_integration.py`
   - __init__() : passe robot_api au BBIABehaviorManager
   - apply_emotion_to_robot() : goto_target combiné
   - react_to_vision_detection() : look_at_world/look_at_image
   - sync_voice_with_movements() : goto_target pour fluidité

---

## Validation

- Tests : tests de conformité au vert
- SDK : conformité avec le SDK Reachy Mini
- Performance : mouvements fluides (interpolation minjerk)
- Compatibilité : fallbacks si SDK non disponible

---

## Prêt pour robot physique

Ces optimisations apportent :

- des mouvements plus naturels et fluides
- une meilleure synchronisation tête+corps
- un suivi visuel via IK
- une résilience accrue (avec ou sans SDK)
