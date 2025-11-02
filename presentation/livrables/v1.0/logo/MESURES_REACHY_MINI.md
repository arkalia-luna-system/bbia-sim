# 📏 Mesures Exactes Reachy Mini - Source XML Officiel

> **Extraction des dimensions réelles** depuis `reachy_mini_REAL_OFFICIAL.xml`

---

## Dimensions Globales (Référence Documentation)

- **Hauteur totale** : 280mm (28cm) mode actif / 230mm (23cm) mode veille
- **Largeur** : 160mm (16cm)
- **Poids** : 1.5kg

---

## Dimensions Extraites du XML (en mètres, converties en mm)

### Position Tête (Frame "head")
```
pos="-0.00611127 0.00370522 0.0291364"
```
**En mm** : 
- X : -6.11mm (décalage gauche)
- Y : 3.7mm (décalage avant)
- Z : 29.14mm (hauteur depuis corps)

### Antennes

**Antenne Droite** (right_antenna) :
```
pos antenne mesh: "8.00228e-15 -0.0588 -0.0103"
```
- Hauteur : -58.8mm (depuis base antenne)
- Profondeur : -10.3mm

**Antenne Gauche** (left_antenna) :
```
pos antenne mesh: "6.79838e-15 -0.0588 -0.0103"
```
- Hauteur : -58.8mm (identique)
- Profondeur : -10.3mm

**Position bases antennes** :
- Droite : `pos="-0.0948524 0.0197779 -0.00445785"`
- Gauche : `pos="-0.0764135 -0.0324475 0.0840224"`

### Corps

**Body_down_3dprint** (corps principal) :
```
pos="3.79972e-17 -3.70588e-18 0.195"
```
- Hauteur Z : 195mm (19.5cm depuis base)

**Body_top_3dprint** :
```
pos="6.43558e-16 9.98888e-16 0.195"
```
- Hauteur Z : 195mm (identique)

---

## Proportions Calculées (pour logo vue de face)

### Hauteur Totale
- Base à corps : ~195mm
- Corps à tête : ~29mm
- Tête elle-même : ~? (à estimer depuis mesh)
- Antennes : ~60mm de hauteur depuis base

### Largeur
- Corps largeur : ~160mm (16cm total)
- Tête : Estimation ~80-100mm de largeur
- Antennes : Positionnées à ~±75-95mm de part et d'autre du centre

### Ratio pour Logo (vue de face)

Si robot total = 280mm hauteur :
- **Tête seule** : ~80-100mm largeur x ~80-90mm hauteur
- **Antennes** : ~60mm longueur, positionnées environ à ±80mm du centre
- **Espacement antennes** : ~160mm entre bases

---

## Calcul pour SVG (si robot = 512px de hauteur)

**Échelle** : 280mm réels = 512px
- **1mm réel** = 1.83px

**Tête** (vue de face) :
- Largeur : ~90mm = 165px (rayon 82.5px)
- Hauteur : ~85mm = 156px (rayon 78px)

**Antennes** :
- Longueur : 60mm = 110px
- Position : ±80mm du centre = ±146px du centre
- Diamètre : ~2-3mm = 4-6px (fines)

**Yeux** (grands cercles noirs) :
- Diamètre estimé : ~30-35mm = 55-64px
- Espacement : ~40mm = 73px entre centres

---

*Mesures extraites le octobre 2025 depuis reachy_mini_REAL_OFFICIAL.xml*

