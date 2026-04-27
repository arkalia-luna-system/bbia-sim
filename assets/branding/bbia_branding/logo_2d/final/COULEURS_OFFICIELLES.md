# 🎨 COULEURS OFFICIELLES BBIA LOGO

**Date** : 2025-11-10  
**Source** : Analyse des fichiers SVG `_SOURCE.svg` depuis Inkscape  
**Statut** : ✅ Officiel et vérifié

---

## 📋 PALETTE PRINCIPALE

### 1. BBIA Blue (Fond)
- **Hex** : `#008181`
- **Hex avec alpha** : `#008181FF`
- **RGB** : `RGB(0, 129, 129)`
- **RGBA** : `RGBA(0, 129, 129, 255)`
- **Usage** : Fond carré derrière le robot
- **Échantillon** : <span style="display:inline-block;width:50px;height:50px;background-color:#008181;border:1px solid #ccc;"></span>

### 2. BBIA Blanc (Corps du robot)
- **Hex** : `#FFFFFF`
- **Hex avec alpha** : `#FFFFFFFF`
- **RGB** : `RGB(255, 255, 255)`
- **RGBA** : `RGBA(255, 255, 255, 255)`
- **Usage** : Corps principal du robot
- **Échantillon** : <span style="display:inline-block;width:50px;height:50px;background-color:#FFFFFF;border:1px solid #ccc;"></span>

### 3. BBIA Gris Clair (Yeux)
- **Hex** : `#CCCCCC`
- **Hex avec alpha** : `#CCCCCCFF`
- **RGB** : `RGB(204, 204, 204)`
- **RGBA** : `RGBA(204, 204, 204, 255)`
- **Usage** : Yeux du robot
- **Échantillon** : <span style="display:inline-block;width:50px;height:50px;background-color:#CCCCCC;border:1px solid #ccc;"></span>

### 4. BBIA Noir Foncé (Détails)
- **Hex** : `#020202`
- **Hex avec alpha** : `#020202FF`
- **RGB** : `RGB(2, 2, 2)`
- **RGBA** : `RGBA(2, 2, 2, 255)`
- **Usage** : Détails et ombres du robot
- **Échantillon** : <span style="display:inline-block;width:50px;height:50px;background-color:#020202;border:1px solid #ccc;"></span>

### 5. BBIA Noir Pur (Texte)
- **Hex** : `#000000`
- **Hex avec alpha** : `#000000FF`
- **RGB** : `RGB(0, 0, 0)`
- **RGBA** : `RGBA(0, 0, 0, 255)`
- **Usage** : Texte "BBIA"
- **Échantillon** : <span style="display:inline-block;width:50px;height:50px;background-color:#000000;border:1px solid #ccc;"></span>

---

## 🎯 COULEURS SECONDAIRES (Dégradés et nuances)

Ces couleurs sont utilisées pour les dégradés et les nuances dans le logo :

| Couleur | Hex | RGB | Usage |
|---------|-----|-----|-------|
| Gris très clair | `#FEFEFE` | RGB(254, 254, 254) | Dégradés blancs |
| Gris clair 2 | `#F9F9F9` | RGB(249, 249, 249) | Dégradés blancs |
| Gris clair 3 | `#EEEEEE` | RGB(238, 238, 238) | Dégradés blancs |
| Gris moyen clair | `#E6E6E6` | RGB(230, 230, 230) | Dégradés gris |
| Gris moyen | `#D1D1D1` | RGB(209, 209, 209) | Dégradés gris |
| Gris moyen foncé | `#C7C7C7` | RGB(199, 199, 199) | Dégradés gris |
| Gris foncé 1 | `#A2A2A2` | RGB(162, 162, 162) | Ombres |
| Gris foncé 2 | `#939393` | RGB(147, 147, 147) | Ombres |
| Bleu-gris | `#518EA1` | RGB(81, 142, 161) | Détails |
| Gris très foncé 1 | `#505050` | RGB(80, 80, 80) | Ombres |
| Gris très foncé 2 | `#333333` | RGB(51, 51, 51) | Ombres |
| Noir presque pur | `#010101` | RGB(1, 1, 1) | Ombres |

---

## 📐 UTILISATION

### Pour le développement web

```css
:root {
  --bbia-blue: #008181;
  --bbia-white: #FFFFFF;
  --bbia-gray-light: #CCCCCC;
  --bbia-black-dark: #020202;
  --bbia-black: #000000;
}
```

### Pour Python (PIL/Pillow)

```python
COLORS = {
    "blue": (0, 129, 129),      # #008181
    "white": (255, 255, 255),        # #FFFFFF
    "gray_light": (204, 204, 204),   # #CCCCCC
    "black_dark": (2, 2, 2),         # #020202
    "black": (0, 0, 0),              # #000000
}
```

### Pour Inkscape

- **Fond** : `#008181`
- **Corps** : `#FFFFFF`
- **Yeux** : `#CCCCCC`
- **Détails** : `#020202`
- **Texte** : `#000000`

---

## ⚠️ IMPORTANT

### Couleurs à NE PAS utiliser

- ❌ `#0066FF` (Bleu électrique) - **N'est PAS utilisé dans le logo**
- ❌ `#2C2C2C` (Gris neutre) - **N'est PAS utilisé dans le logo**

Ces couleurs étaient mentionnées dans l'ancienne documentation mais ne correspondent **PAS** aux couleurs réelles du logo.

---

## ✅ VÉRIFICATION

Les couleurs ont été extraites depuis :
- ✅ `bbia_mark_only_v2_SOURCE.svg`
- ✅ `bbia_logo_vertical_v2_SOURCE.svg`
- ✅ `bbia_logo_horizontal_SOURCE.svg`

**Script utilisé** : `extract_colors_from_svg.py`

---

**Dernière mise à jour** : 2025-11-10  
**Version** : 1.0 Officielle

