# 🎨 Codes Couleur - Reachy Mini Simulation MuJoCo

> **Document de référence** : Codes couleur utilisés dans la simulation MuJoCo du Reachy Mini
> **Date** : 2025
> **Projet** : BBIA - Simulation Reachy Mini

---

## 📋 Table des matières

1. [Palette BBIA Officielle](#palette-bbia-officielle)
2. [Couleurs MuJoCo - Modèle Officiel](#couleurs-mujoco---modèle-officiel)
3. [Couleurs MuJoCo - Modèle Minimal](#couleurs-mujoco---modèle-minimal)
4. [Conversions RGB ↔ HEX](#conversions-rgb--hex)
5. [Références dans le Code](#références-dans-le-code)

---

## 🎨 Palette BBIA Officielle

### Couleurs Principales

#### **Bleu Céleste / Néon**
- **HEX** : `#87bcfa` (principal) / `#3E6FFF` (sombre)
- **RGB** :
  - `#87bcfa` → `rgb(135, 188, 250)` → MuJoCo: `0.529 0.737 0.980`
  - `#3E6FFF` → `rgb(62, 111, 255)` → MuJoCo: `0.243 0.435 1.000`
- **Variations** :
  - Light: `#B0D4FF` → `0.690 0.831 1.000`
  - Medium: `#87bcfa` → `0.529 0.737 0.980`
  - Dark: `#3E6FFF` → `0.243 0.435 1.000`

#### **Violet Électrique**
- **HEX** : `#A680FF` (principal) / `#C082FF` (alternatif)
- **RGB** :
  - `#A680FF` → `rgb(166, 128, 255)` → MuJoCo: `0.651 0.502 1.000`
  - `#C082FF` → `rgb(192, 130, 255)` → MuJoCo: `0.753 0.510 1.000`
- **Variations** :
  - Light: `#D4B0FF` → `0.831 0.690 1.000`
  - Medium: `#A680FF` → `0.651 0.502 1.000`
  - Dark: `#8B5FD4` → `0.545 0.373 0.831`

#### **Turquoise Éthéré**
- **HEX** : `#60e9e1`
- **RGB** : `rgb(96, 233, 225)` → MuJoCo: `0.376 0.914 0.882`
- **Variations** :
  - Light: `#8FEFEA` → `0.561 0.937 0.918`
  - Medium: `#60e9e1` → `0.376 0.914 0.882`
  - Dark: `#3FB8B2` → `0.247 0.722 0.698`

#### **Gris Lunaire**
- **HEX** : `#eaeaed` (principal) / `#bfc9d9` (moyen)
- **RGB** :
  - `#eaeaed` → `rgb(234, 234, 237)` → MuJoCo: `0.918 0.918 0.929`
  - `#bfc9d9` → `rgb(191, 201, 217)` → MuJoCo: `0.749 0.788 0.851`
- **Variations** :
  - Light: `#eaeaed` → `0.918 0.918 0.929`
  - Medium: `#bfc9d9` → `0.749 0.788 0.851`
  - Dark: `#8A94A8` → `0.541 0.580 0.659`

#### **Rose Pastel**
- **HEX** : `#FFDAEC`
- **RGB** : `rgb(255, 218, 236)` → MuJoCo: `1.000 0.855 0.925`
- **Variations** :
  - Light: `#FFDAEC` → `1.000 0.855 0.925`
  - Medium: `#FFB5D8` → `1.000 0.710 0.847`
  - Dark: `#FF90C4` → `1.000 0.565 0.769`

---

## 🤖 Couleurs MuJoCo - Modèle Officiel

### Skybox BBIA
```xml
<texture name="skybox_bbia" type="skybox" builtin="gradient"
         rgb1="0.92 0.92 0.93" rgb2="0.53 0.74 0.98"/>
```
- **rgb1** (Gris lunaire) : `0.92 0.92 0.93` = `#EAEAED`
- **rgb2** (Bleu céleste) : `0.53 0.74 0.98` = `#87BCFA`

### Matériaux du Robot (RGBA)

#### Corps Principal
- **body_top_3dprint_material** : `rgba="1 1 1 1"` → **Blanc** `#FFFFFF`
- **body_down_3dprint_material** : `rgba="1 1 1 1"` → **Blanc** `#FFFFFF`
- **body_turning_3dprint_material** : `rgba="0.301961 0.301961 0.301961 1"` → **Gris moyen** `#4D4D4D`
- **body_foot_3dprint_material** : `rgba="0.301961 0.301961 0.301961 1"` → **Gris moyen** `#4D4D4D`

#### Tête
- **head_front_3dprint_material** : `rgba="1 1 1 1"` → **Blanc** `#FFFFFF`
- **head_back_3dprint_material** : `rgba="1 1 1 1"` → **Blanc** `#FFFFFF`
- **head_mic_3dprint_material** : `rgba="1 1 1 1"` → **Blanc** `#FFFFFF`
- **glasses_dolder_3dprint_material** : `rgba="0.196078 0.196078 0.196078 1"` → **Gris foncé** `#323232`
- **neck_reference_3dprint_material** : `rgba="0.901961 0.901961 0.901961 1"` → **Gris clair** `#E6E6E6`

#### Bras Stewart Platform
- **stewart_main_plate_3dprint_material** : `rgba="0.301961 0.301961 0.301961 1"` → **Gris moyen** `#4D4D4D`
- **stewart_tricap_3dprint_material** : `rgba="0.301961 0.301961 0.301961 1"` → **Gris moyen** `#4D4D4D`
- **mp01062_stewart_arm_3_material** : `rgba="0.301961 0.301961 0.301961 1"` → **Gris moyen** `#4D4D4D`
- **stewart_link_rod_material** : `rgba="0.811765 0.858824 0.898039 1"` → **Bleu-gris clair** `#CFDBE5`
- **stewart_link_ball_material** : `rgba="0.439216 0.47451 0.501961 1"` → **Gris-bleu** `#707980`
- **stewart_link_ball__2_material** : `rgba="0.439216 0.47451 0.501961 1"` → **Gris-bleu** `#707980`

#### Composants Électroniques
- **dc15_a01_horn_dummy_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`
- **dc15_a01_case_b_dummy_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`
- **dc15_a01_case_m_dummy_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`
- **dc15_a01_case_f_dummy_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`
- **dc15_a01_led_cap2_dummy_material** : `rgba="0.87451 0.447059 0.317647 1"` → **Orange saumon** `#DF7251`
- **phs_1_7x20_5_dc10_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`
- **phs_1_7x20_5_dc10_1_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`
- **phs_1_7x20_5_dc10_2_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`
- **phs_1_7x20_5_dc10_3_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`
- **b3b_eh_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`
- **b3b_eh_1_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`
- **bts2_m2_6x8_material** : `rgba="0.262745 0.282353 0.301961 1"` → **Gris-bleu foncé** `#43484D`

#### Caméras et Optiques
- **arducam_material** : `rgba="0.498039 0.498039 0.498039 1"` → **Gris moyen** `#7F7F7F`
- **pp01102_arducam_carter_material** : `rgba="0.301961 0.301961 0.301961 1"` → **Gris moyen** `#4D4D4D`
- **big_lens_d40_material** : `rgba="0.439216 0.47451 0.501961 0.301961"` → **Gris-bleu transparent** `#707980` (alpha: 0.30)
- **small_lens_d30_material** : `rgba="0.439216 0.47451 0.501961 0.301961"` → **Gris-bleu transparent** `#707980` (alpha: 0.30)
- **m12_fisheye_lens_1_8mm_material** : `rgba="0.498039 0.498039 0.498039 1"` → **Gris moyen** `#7F7F7F`
- **lens_cap_d30_3dprint_material** : `rgba="0.0980392 0.0980392 0.0980392 1"` → **Noir** `#191919`
- **lens_cap_d40_3dprint_material** : `rgba="0.0980392 0.0980392 0.0980392 1"` → **Noir** `#191919`

#### Antennes
- **antenna_material** : `rgba="0 0 0 1"` → **Noir** `#000000`
- **antenna_body_3dprint_material** : `rgba="1 1 1 1"` → **Blanc** `#FFFFFF`
- **antenna_interface_3dprint_material** : `rgba="0.713725 0.760784 0.8 1"` → **Bleu-gris clair** `#B6C2CC`
- **antenna_holder_l_3dprint_material** : `rgba="0.498039 0.498039 0.498039 1"` → **Gris moyen** `#7F7F7F`
- **antenna_holder_r_3dprint_material** : `rgba="0.498039 0.498039 0.498039 1"` → **Gris moyen** `#7F7F7F`

#### Autres Composants
- **5w_speaker_material** : `rgba="0.196078 0.196078 0.196078 1"` → **Gris foncé** `#323232`
- **bearing_85x110x13_material** : `rgba="0.768627 0.886275 0.952941 1"` → **Bleu clair** `#C4E2F3`

---

## 🎨 Couleurs MuJoCo - Modèle Minimal

### Matériaux de Base
```xml
<!-- Matériaux pour un rendu réaliste -->
<material name="torso_mat" rgba="0.2 0.2 0.8 1" shininess="0.3" specular="0.1"/>
<material name="head_mat" rgba="0.8 0.2 0.2 1" shininess="0.5" specular="0.2"/>
<material name="arm_mat" rgba="0.2 0.8 0.2 1" shininess="0.4" specular="0.15"/>
<material name="gripper_mat" rgba="0.5 0.5 0.5 1" shininess="0.6" specular="0.3"/>
```

- **torso_mat** : `rgba="0.2 0.2 0.8 1"` → **Bleu** `#3333CC`
- **head_mat** : `rgba="0.8 0.2 0.2 1"` → **Rouge** `#CC3333`
- **arm_mat** : `rgba="0.2 0.8 0.2 1"` → **Vert** `#33CC33`
- **gripper_mat** : `rgba="0.5 0.5 0.5 1"` → **Gris moyen** `#808080`

### Géométrie par défaut
- **Default geom** : `rgba="0.8 0.6 0.4 1"` → **Beige** `#CC9966`

### Sol
- **Floor** : `rgba="0.9 0.9 0.9 1"` → **Gris très clair** `#E6E6E6`

---

## 🔄 Conversions RGB ↔ HEX

### Formule de Conversion

#### HEX → RGB (0-255) → MuJoCo (0-1)
```
HEX: #RRGGBB
RGB: (R, G, B) = (hex_to_dec(RR), hex_to_dec(GG), hex_to_dec(BB))
MuJoCo: (R/255, G/255, B/255)
```

#### Exemples
- `#87bcfa` → RGB(135, 188, 250) → MuJoCo: `0.529 0.737 0.980`
- `#eaeaed` → RGB(234, 234, 237) → MuJoCo: `0.918 0.918 0.929`
- `#4D4D4D` → RGB(77, 77, 77) → MuJoCo: `0.302 0.302 0.302`

### Outil de Conversion Python
```python
def hex_to_mujoco_rgba(hex_color, alpha=1.0):
    """Convertit HEX en RGBA MuJoCo (0-1)"""
    hex_color = hex_color.lstrip('#')
    r = int(hex_color[0:2], 16) / 255.0
    g = int(hex_color[2:4], 16) / 255.0
    b = int(hex_color[4:6], 16) / 255.0
    return f"{r:.6f} {g:.6f} {b:.6f} {alpha}"

# Exemple
print(hex_to_mujoco_rgba("#87bcfa"))  # 0.529412 0.737255 0.980392 1.0
```

---

## 📍 Références dans le Code

### Fichiers Principaux

1. **Modèle Officiel** : `src/bbia_sim/sim/models/reachy_mini_REAL_OFFICIAL.xml`
   - Lignes 533-620 : Définition des matériaux et skybox

2. **Modèle Minimal** : `src/bbia_sim/sim/models/reachy_mini.xml`
   - Lignes 34-38 : Matériaux simplifiés

3. **Palette BBIA** : `presentation/palette_bbia.json`
   - Palette complète avec variations

4. **CSS Palette** : `presentation/palette_bbia.css`
   - Variables CSS pour utilisation web

5. **Documentation** : `docs/simulations/CONSEILS_PROCREATE_ROBOT.md`
   - Lignes 121-123 : Recommandations couleurs robot

---

## 🎯 Résumé des Couleurs Principales

| Élément | HEX | RGB (0-255) | MuJoCo RGBA (0-1) |
|---------|-----|-------------|-------------------|
| **Bleu Céleste** | `#87bcfa` | (135, 188, 250) | `0.529 0.737 0.980 1` |
| **Gris Lunaire** | `#eaeaed` | (234, 234, 237) | `0.918 0.918 0.929 1` |
| **Corps Robot** | `#FFFFFF` | (255, 255, 255) | `1.0 1.0 1.0 1` |
| **Bras Robot** | `#4D4D4D` | (77, 77, 77) | `0.302 0.302 0.302 1` |
| **Antennes** | `#000000` | (0, 0, 0) | `0.0 0.0 0.0 1` |

---

## 📝 Notes

- Les couleurs du modèle officiel sont extraites directement des fichiers STL originaux
- La palette BBIA est utilisée pour le skybox et les éléments de scène
- Les valeurs RGBA dans MuJoCo sont normalisées entre 0 et 1 (pas 0-255)
- L'alpha (transparence) est généralement à 1.0 pour les matériaux opaques

---

**Dernière mise à jour** : 2025
**Fichier source** : `docs/simulations/CODES_COULEUR_REACHY_MINI.md`

