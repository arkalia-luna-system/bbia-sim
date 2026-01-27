# 🖨️ Fichiers STL pour Impression 3D

**Dernière mise à jour** : 26 Janvier 2026

## 📁 Organisation

- `stewart_arms/` - Bras Stewart pour plateforme Stewart
- `parts/` - Autres pièces à imprimer

## ⚠️ **IMPORTANT : Unités des fichiers STL**

**Tous les fichiers STL sont en MÈTRES** (format standard STL).

Pour l'impression 3D :
- Les logiciels de slicing (Cura, PrusaSlicer, etc.) détectent automatiquement l'unité
- Si besoin de conversion manuelle : **1 unité STL = 1000 mm**

## 📐 Fichiers disponibles

### Bras Stewart

| Fichier | Dimensions (mm) | Description |
|---------|----------------|-------------|
| `pp01062_stewart_arm.stl` | 49.25 x 5.50 x 16.50 | Bras Stewart (téléchargé 26 jan 2026) |
| `mp01062_stewart_arm_3.stl` | 27.48 x 46.95 x 15.99 | Bras Stewart (projet) |

**Note** : Les deux fichiers ont des dimensions différentes. Vérifier lequel correspond aux moteurs reçus.

## 🖨️ Compatibilité Imprimante H2S

- **Plateau** : 220 x 220 x 250 mm
- **Tous les fichiers** : ✅ Compatibles (dimensions < plateau)

### Paramètres recommandés

- **Résolution** : 0.2mm (standard) ou 0.15mm (qualité)
- **Support** : Généralement non nécessaire pour les bras Stewart
- **Remplissage** : 20-30% (suffisant pour la résistance)
- **Matériau** : PLA ou PETG (PLA recommandé pour débuter)

## 🔧 Outils d'analyse

Un script Python est disponible pour analyser les fichiers STL :
```bash
python scripts/analyze_stl.py <fichier.stl>
```

## 📝 Notes

- Les fichiers STL du projet sont dans `assets/visual/` (pour visualisation 3D)
- Les fichiers d'impression sont dans `assets/3dprint/` (pour impression réelle)
- Toujours vérifier les dimensions avant impression
