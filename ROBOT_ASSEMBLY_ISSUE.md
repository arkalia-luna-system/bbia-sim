# Problème d'Assemblage du Robot Reachy Mini

## 🚨 Problème Identifié

Le robot Reachy Mini apparaît **"en pièces détachées"** dans la simulation MuJoCo au lieu d'être correctement assemblé.

## 🔍 Causes Identifiées

### 1. **Assets STL Manquants ou Corrompus**
- **Git LFS** : Les fichiers STL du dépôt officiel utilisent Git LFS (Large File Storage)
- **Pointeurs** : Les fichiers téléchargés sont des pointeurs, pas les vrais fichiers binaires
- **Format** : MuJoCo ne peut pas lire les fichiers STL ASCII ou corrompus

### 2. **Positions des Corps Incorrectes**
- **Centres STL** : Les fichiers STL ont des centres différents qui ne sont pas compensés
- **Hiérarchie** : La structure parent-enfant des corps n'est pas optimale
- **Dimensions** : Les positions relatives entre les pièces ne correspondent pas à la réalité

### 3. **Modèles de Test Multiples**
- **Prolifération** : Trop de modèles de test créés (`reachy_mini_*.xml`)
- **Confusion** : Difficile de savoir quel modèle utiliser
- **Maintenance** : Code dupliqué et difficile à maintenir

## 📊 Modèles Disponibles

| Modèle | Statut | Articulations | Problème Principal |
|--------|--------|---------------|-------------------|
| `reachy_mini.xml` | ❌ | 7 | Pièces détachées |
| `reachy_mini_assembled.xml` | ⚠️ | 15 | Positions trop petites |
| `reachy_mini_perfect.xml` | ⚠️ | 15 | Dimensions incorrectes |
| `reachy_mini_official_latest.xml` | ❌ | 42 | Assets manquants |
| `reachy_mini_perfect_assembled.xml` | ✅ | 15 | **Modèle de référence** |

## 🎯 Solution Recommandée

### Modèle de Référence : `reachy_mini_perfect_assembled.xml`

**Avantages :**
- ✅ Hiérarchie des corps correcte
- ✅ Positions réalistes (hauteur ~0.55m, largeur ~0.35m)
- ✅ Compensation des centres STL
- ✅ 15 articulations fonctionnelles
- ✅ Matériaux et couleurs réalistes

**Spécifications :**
- **Hauteur totale** : ~0.55m (dans la plage 0.4-0.7m)
- **Largeur totale** : ~0.35m (dans la plage 0.2-0.4m)
- **Articulations** : 15 (3 tête + 6 bras droit + 6 bras gauche)
- **Assets** : Utilise uniquement les STL disponibles et fonctionnels

## 🔧 Actions Correctives Appliquées

### 1. **Nettoyage du Code**
- ✅ `ruff check` : Aucune erreur de style
- ✅ `black` : Formatage automatique appliqué
- ✅ `mypy` : Aucune erreur de type
- ✅ Suppression des fichiers macOS cachés (`._*`)

### 2. **Organisation des Fichiers**
- ✅ Modèles dans `src/bbia_sim/sim/models/`
- ✅ Assets dans `src/bbia_sim/sim/assets/reachy_official/`
- ✅ Scripts de validation dans `scripts/`

### 3. **Documentation**
- ✅ Ce fichier de documentation
- ✅ Scripts de validation avec scores de fidélité
- ✅ Commentaires dans les modèles XML

## 🚀 Prochaines Étapes

1. **Valider** le modèle `reachy_mini_perfect_assembled.xml`
2. **Tester** l'assemblage visuel dans MuJoCo
3. **Ajuster** les positions si nécessaire
4. **Commit** des changements propres
5. **Push** vers le dépôt distant

## 📝 Notes Techniques

### Centres STL Identifiés
```xml
<!-- Compensation des centres STL -->
<geom pos="0 0 0.188"/>     <!-- body_down_3dprint.stl -->
<geom pos="0 0 0.105"/>     <!-- body_top_3dprint.stl -->
<geom pos="0 0.033 0"/>     <!-- head_front_3dprint.stl -->
<geom pos="0 -0.012 0"/>    <!-- head_back_3dprint.stl -->
<geom pos="0 -0.009 0.186"/> <!-- stewart_main_plate_3dprint.stl -->
<geom pos="0.023 0.045 0.153"/> <!-- mp01062_stewart_arm_3.stl -->
<geom pos="0 -0.009 0.148"/> <!-- stewart_tricap_3dprint.stl -->
```

### Hiérarchie des Corps
```
base (0, 0, 0.20)
└── torso (0, 0, 0.30)
    ├── head (0, 0, 0.25)
    ├── right_arm (0, -0.175, 0.20)
    │   └── right_forearm (0, 0, 0.12)
    │       └── right_gripper (0, 0, 0.08)
    └── left_arm (0, 0.175, 0.20)
        └── left_forearm (0, 0, 0.12)
            └── left_gripper (0, 0, 0.08)
```

---

**Date de création** : 2025-10-23  
**Statut** : En cours de résolution  
**Priorité** : Haute
