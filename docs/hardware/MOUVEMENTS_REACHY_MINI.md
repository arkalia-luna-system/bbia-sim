# Mouvements Reachy Mini - Analyse et Améliorations

## État actuel des joints

D'après l'analyse du modèle officiel `reachy_mini_REAL_OFFICIAL.xml` :

### Joints mobiles (7 joints)

- **yaw_body**: [-2.793, 2.793] rad - Rotation corps principale
- **stewart_1**: [-0.838, 1.396] rad - Plateforme Stewart joint 1
- **stewart_2**: [-1.396, 1.222] rad - Plateforme Stewart joint 2
- **stewart_3**: [-0.838, 1.396] rad - Plateforme Stewart joint 3
- **stewart_4**: [-1.396, 0.838] rad - Plateforme Stewart joint 4
- **stewart_5**: [-1.222, 1.396] rad - Plateforme Stewart joint 5
- **stewart_6**: [-1.396, 0.838] rad - Plateforme Stewart joint 6

### Joints bloqués (7 joints)

- **passive_1 à passive_7**: [0.000, 0.000] rad (bloqués - joints passifs)

### Antennes animables (2 joints)

- **left_antenna**: [-0.300, 0.300] rad (animable avec limites de sécurité)
- **right_antenna**: [-0.300, 0.300] rad (animable avec limites de sécurité)

---

## Problèmes identifiés

### Problèmes dans les démos originales

1. **Tentatives d'animation des antennes** - Les antennes sont maintenant animables avec limites (-0.3 à 0.3 rad)
2. **Mouvements trop basiques** - Sinusoïdes simples sans contexte émotionnel
3. **Amplitudes inappropriées** - Ne respectent pas les limites réelles des joints
4. **Pas de transitions fluides** - Mouvements saccadés

---

## Améliorations appliquées

### Fichiers modifiés

#### 1. `examples/demo_chat_bbia_3d.py`

**Améliorations :**

- antennes maintenant utilisables avec limites de sécurité (-0.3 à 0.3 rad)
- mouvements expressifs selon le contexte émotionnel
  - **Salutations** : hochement tête expressif (stewart_1 + stewart_2 + yaw_body)
  - **Positif/Joyeux** : rotation corps + mouvement tête expressif
  - **Question/Curieux** : inclinaison tête + rotation corps
  - **Triste/Empathique** : tête baissée empathique
- amplitudes ajustées selon les limites réelles des joints
- ajout de `animate_with_viewer()` avec transitions fluides

#### 2. Alignement avec SDK officiel Reachy Mini

**Documentation de référence:**

- GitHub: https://github.com/pollen-robotics/reachy_mini
- SDK disponible depuis Novembre 2025
- ~125 unités en beta test
- ~3000 unités prévues avant Noël

---

## Recommandations pour futures démos

### À faire

1. **Toujours vérifier les joints mobiles avant d'animer :**

```bash
python scripts/check_joints.py

```

2. **Utiliser les 9 joints mobiles :**
   - `yaw_body` pour les rotations corps
   - `stewart_1-6` pour les mouvements tête
   - `left_antenna`, `right_antenna` pour expressivité fine (limites -0.3 à 0.3 rad)

3. **Respecter les limites des joints :**
   - `yaw_body`: [-2.793, 2.793] rad
   - `stewart_1-6`: voir limites spécifiques ci-dessus

4. **Implémenter des mouvements expressifs :**
   - Considérer le contexte émotionnel
   - Utiliser des transitions fluides (sinusoïdes)
   - Éviter les mouvements saccadés

### À éviter

1. ne pas dépasser les limites des antennes (-0.3 à 0.3 rad) — protection hardware
2. ne pas animer les joints passifs (passive_1-7) — bloqués
3. ne pas dépasser les limites officielles
4. ne pas utiliser des amplitudes excessives (> 0.3 rad pour antennes, > 0.5 rad pour autres)

---

## Patterns de mouvements recommandés

### 1. Salutations

```python
# Hochement de tête expressif
stewart_1: 0.2-0.3 rad (pitch up)
stewart_2: 0.1-0.15 rad (yaw latéral)
yaw_body: 0.15 rad (rotation vers utilisateur)

```

### 2. Émotion Positive/Joyeuse

```python
# Mouvements euphoriques
stewart_1: 0.3-0.4 rad (pitch up fort)
stewart_3: 0.15-0.2 rad (roll expressif)
yaw_body: 0.2-0.25 rad (rotation joyeuse)

```

### 3. Curiosité/Question

```python
# Inclinaison tête curieuse
stewart_2: 0.2-0.25 rad (yaw latéral fort)
yaw_body: 0.15 rad (rotation curiosité)

```

### 4. Tristesse/Empathie

```python
# Position basse empathique
stewart_1: -0.2 à -0.3 rad (pitch down)
yaw_body: -0.15 rad (rotation lente)

```

---

## Ressources

- SDK officiel: https://github.com/pollen-robotics/reachy_mini
- Démo corrigée: `examples/demo_chat_bbia_3d.py`
- Script vérification: `scripts/check_joints.py`
- Documentation complète: `docs/ARCHITECTURE.md`

---

## Validation

**Tests effectués :**

- Black (formatage) : OK
- Ruff (linting) : OK
- Joints mobiles identifiés : 9/16 (yaw_body + stewart_1-6 + 2 antennes)
- Antennes animables confirmées : left_antenna, right_antenna (limites -0.3 à 0.3 rad)
- Mouvements expressifs implémentés
- Amplitudes sécurisées selon limites officielles

**Dernière mise à jour** : 26 Janvier 2026
