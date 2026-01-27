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

### Paramètres slicer - PLA (MAXIMUM RÉSISTANCE)

**⚠️ IMPORTANT** : Ces paramètres sont optimisés pour la **résistance mécanique maximale** avec du PLA.

| Paramètre | Valeur | Pourquoi |
|-----------|--------|----------|
| **Résolution (hauteur couche)** | **0.15 mm** (qualité) ou **0.2 mm** (standard) | Couches plus fines = meilleure adhésion inter-couches = plus résistant |
| **Remplissage** | **40-50%** (au lieu de 20-30%) | Plus de matière = plus résistant |
| **Motif remplissage** | **Grid** ou **Triangles** (éviter Lines) | Meilleure résistance aux forces multidirectionnelles |
| **Périmètres (murs)** | **5-6** (au lieu de 3-4) | Plus de murs = plus résistant aux forces latérales |
| **Température buse** | **210-220°C** | Température plus élevée = meilleure fusion = plus résistant |
| **Température plateau** | **60-65°C** | Bonne adhésion sans déformation |
| **Vitesse d'impression** | **40-50 mm/s** (plus lent = mieux) | Impression plus lente = meilleure fusion = plus résistant |
| **Vitesse périmètres** | **30-40 mm/s** | Encore plus lent pour les murs = qualité maximale |
| **Refroidissement** | **Désactivé** pour les 3-4 premières couches, puis **30-50%** | Évite le warping, permet bonne adhésion |
| **Top/Bottom layers** | **5-6 couches** (au lieu de 3-4) | Plus de couches solides = plus résistant |
| **Température chambre** | **Ambiente** (pas de chambre chauffée) | PLA n'aime pas la chaleur excessive |

### ⚠️ PLA vs PETG - Résistance

**PLA** :
- ✅ **Avantages** : Facile à imprimer, pas de warping, bon pour débuter
- ⚠️ **Limites** : Moins résistant que PETG, peut se déformer à >60°C
- ✅ **Pour bras moteur** : **PLA fonctionne très bien** si bien imprimé avec ces paramètres

**PETG** (si tu en achètes plus tard) :
- ✅ Plus résistant mécaniquement
- ✅ Plus flexible (moins cassant)
- ✅ Résiste mieux à la chaleur
- ⚠️ Plus difficile à imprimer (adhésion, stringing)

**Conclusion** : Le PLA avec ces paramètres est **parfaitement adapté** pour les bras Stewart. Pas besoin de PETG pour l'instant.

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

## 🔧 Paramètres avancés pour résistance maximale

### Orientation d'impression (CRUCIAL)

**✅ RECOMMANDÉ** : Imprimer **debout** (hauteur = 49.25mm)
- Les forces du moteur sont principalement **verticales**
- Les couches horizontales résistent mieux aux forces verticales
- Éviter d'imprimer à plat (couches verticales = plus fragile)

### Post-traitement (optionnel mais recommandé)

1. **Lissage** : Vaporiser avec acétone (si PLA+ ou ABS) - **ATTENTION** : Ne fonctionne pas avec PLA standard
2. **Renfort** : Si besoin, ajouter des renforts métalliques dans les zones critiques
3. **Test** : Tester un bras avant d'imprimer les 6

### Vérifications après impression

- [ ] Pas de déformations visibles
- [ ] Pas de couches qui se décollent
- [ ] Les trous de fixation sont bien ronds (pas ovalisés)
- [ ] Test d'assemblage avec le moteur (doit s'emboîter sans forcer)

## 📝 Notes importantes

- **Unités STL** : Tous les fichiers STL sont en **MÈTRES** (format standard)
- **Conversion** : Les logiciels de slicing détectent automatiquement l'unité
- **Tolérances** : Prévoir 0.1-0.2mm de tolérance pour l'assemblage
- **Quantité** : 6 bras nécessaires (1 par moteur Stewart)
- **PLA** : Parfaitement adapté avec les bons paramètres (voir ci-dessus)

---

## 🗂️ Organisation fichiers STL

Tous les fichiers STL d'impression sont dans :
- `assets/3dprint/` - Fichiers pour impression 3D
- `assets/visual/` - Fichiers pour visualisation 3D (MuJoCo)

Voir `assets/3dprint/README.md` pour plus d'informations.
