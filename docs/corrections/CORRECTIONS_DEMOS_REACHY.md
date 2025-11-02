# 🎯 CORRECTIONS DÉMOS REACHY MINI - PLAN D'ACTION

**Date:** octobre 2025
**Objectif:** Aligner toutes les démos avec le SDK officiel Reachy-Mini

## 📊 PROBLÈMES IDENTIFIÉS DANS LES DÉMOS

### ❌ Problèmes communs:

1. **Amplitudes excessives:**
   - `demo_behavior_ok.py`: Ligne 87 `wave: 0.5 rad` ❌ (limite 0.3 rad)
   - `demo_behavior_ok.py`: Ligne 90 `emotional: 0.4 rad` ❌ (limite 0.3 rad)
   - `demo_emotion_ok.py`: Ligne 33 `surprised: 0.5 rad` ❌ (limite 0.3 rad)
   - `demo_emotion_ok.py`: Ligne 32 `angry: 0.4 rad` ❌ (limite 0.3 rad)

2. **Mouvements non optimisés:**
   - Sinusoïdes simples sans contexte
   - Pas d'utilisation de `goto_target()` avec interpolation
   - Pas d'utilisation de `create_head_pose()` pour poses complexes

3. **Joints Stewart non utilisés correctement:**
   - Les démos n'utilisent pas toutes les 6 articulations Stewart
   - Pas de mouvements combinés tête+corps

## ✅ SOLUTIONS À APPLIQUER

### 1. Réduire toutes les amplitudes à 0.3 rad max

**Fichiers à corriger:**
- `examples/demo_behavior_ok.py` - Lignes 83-91
- `examples/demo_emotion_ok.py` - Lignes 28-34

**Changements:**
```python
# AVANT (❌ Trop fort)
"wave": lambda t: 0.5 * math.sin(6 * math.pi * t),
"emotional": lambda t: 0.4 * math.sin(2 * math.pi * 0.8 * t),
"surprised": lambda t: 0.5 * math.sin(2 * math.pi * 0.2 * t),
"angry": lambda t: 0.4 * math.sin(2 * math.pi * 0.8 * t),

# APRÈS (✅ Sécurisé)
"wave": lambda t: 0.3 * math.sin(6 * math.pi * t),
"emotional": lambda t: 0.25 * math.sin(2 * math.pi * 0.8 * t),
"surprised": lambda t: 0.3 * math.sin(2 * math.pi * 0.2 * t),
"angry": lambda t: 0.25 * math.sin(2 * math.pi * 0.8 * t),
```

### 2. Utiliser goto_target() pour mouvements fluides

**Fichiers à corriger:**
- `examples/demo_reachy_mini_corrigee.py`
- `examples/demo_behavior_ok.py`

**Changements:**
```python
# Utiliser goto_target() au lieu de set_joint_pos() individuel
from reachy_mini.utils import create_head_pose

# Pour mouvements tête complexe
pose = create_head_pose(pitch=0.15, yaw=0.1, roll=0.05, degrees=False)
robot.goto_target(head=pose, duration=1.0)

# Pour mouvements corps + tête combinés
robot.goto_target(
    head=create_head_pose(pitch=0.1),
    body_yaw=0.15,
    duration=1.0
)
```

### 3. Améliorer les mouvements émotionnels avec poses complexes

**Fichiers à corriger:**
- `examples/demo_emotion_ok.py`
- `examples/demo_behavior_ok.py`

**Changements:**
```python
# Utiliser poses tête complètes au lieu de mouvements simples
emotion_poses = {
    "happy": create_head_pose(pitch=0.15, yaw=0.05, degrees=False),
    "excited": create_head_pose(pitch=0.2, yaw=0.1, roll=0.05, degrees=False),
    "curious": create_head_pose(pitch=0.05, yaw=0.15, degrees=False),
    "sad": create_head_pose(pitch=-0.15, yaw=0.0, degrees=False),
    "calm": create_head_pose(pitch=-0.05, yaw=0.0, degrees=False),
}
```

## 🎯 PLAN D'EXÉCUTION

1. ✅ Corriger `demo_chat_bbia_3d.py` - FAIT
2. ⏳ Corriger `demo_behavior_ok.py` - À FAIRE
3. ⏳ Corriger `demo_emotion_ok.py` - À FAIRE
4. ⏳ Corriger `demo_reachy_mini_corrigee.py` - À FAIRE
5. ⏳ Validation finale avec tests

## 📝 VALIDATION

Après corrections:
- ✅ Black (formatage)
- ✅ Ruff (linting)
- ✅ Amplitudes < 0.3 rad
- ✅ Utilisation goto_target() où approprié
- ✅ Mouvements fluides et réalistes

