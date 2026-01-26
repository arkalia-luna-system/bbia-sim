# 🖨️ Impression 3D - Bras Stewart

**Date** : 26 Janvier 2026  
**Imprimante** : H2S  
**Matériau** : PLA ou PETG recommandé

---

## 📦 Fichiers STL disponibles

### Bras Stewart (pp01062_stewart_arm.stl)

**Dimensions** : 49.25 x 5.50 x 16.50 mm  
**Source** : Téléchargé le 26 janvier 2026  
**Emplacement** : `assets/3dprint/stewart_arms/pp01062_stewart_arm.stl`

### Comparaison avec fichier projet

| Fichier | Dimensions (mm) | Triangles |
|---------|----------------|-----------|
| `pp01062_stewart_arm.stl` (téléchargé) | 49.25 x 5.50 x 16.50 | 2,502 |
| `mp01062_stewart_arm_3.stl` (projet) | 27.48 x 46.95 x 15.99 | 3,110 |

**⚠️ Note** : Les deux fichiers ont des dimensions différentes. Vérifier lequel correspond aux moteurs reçus avant impression.

---

## ✅ Compatibilité Imprimante H2S

**Plateau H2S** : 220 x 220 x 250 mm

| Fichier | Compatible | Marges |
|---------|------------|--------|
| `pp01062_stewart_arm.stl` | ✅ OUI | X: 170.8mm, Y: 214.5mm, Z: 233.5mm |

---

## ⚙️ Paramètres d'impression recommandés

### Orientation
- **Face large** : 49.2 mm (poser sur cette face)
- **Hauteur d'impression** : 5.5 mm
- **Support** : NON nécessaire

### Paramètres slicer

| Paramètre | Valeur |
|-----------|--------|
| **Résolution** | 0.2 mm (standard) ou 0.15 mm (qualité) |
| **Remplissage** | 20-30% |
| **Périmètres** | 3-4 |
| **Température** | 200-210°C (PLA) ou 230-240°C (PETG) |
| **Plateau** | 60°C (PLA) ou 70-80°C (PETG) |
| **Vitesse** | 50-60 mm/s |

---

## 🔧 Vérification avant impression

1. **Vérifier les dimensions** avec le script :
   ```bash
   python scripts/analyze_stl.py assets/3dprint/stewart_arms/pp01062_stewart_arm.stl
   ```

2. **Comparer avec les anciens bras** (si disponibles) :
   - Mesurer les anciens bras
   - Vérifier que les dimensions correspondent

3. **Test d'impression** :
   - Imprimer 1 bras d'abord
   - Tester l'assemblage avec le moteur
   - Vérifier les tolérances

---

## 📝 Notes importantes

- **Unités STL** : Tous les fichiers STL sont en **MÈTRES** (format standard)
- **Conversion** : Les logiciels de slicing détectent automatiquement l'unité
- **Tolérances** : Prévoir 0.1-0.2mm de tolérance pour l'assemblage
- **Quantité** : 6 bras nécessaires (1 par moteur Stewart)

---

## 🗂️ Organisation fichiers STL

Tous les fichiers STL d'impression sont dans :
- `assets/3dprint/` - Fichiers pour impression 3D
- `assets/visual/` - Fichiers pour visualisation 3D (MuJoCo)

Voir `assets/3dprint/README.md` pour plus d'informations.
