# ⚡ Résumé Rapide - Procreate pour Scène MuJoCo

**Date** : 21 Novembre 2025

> **Guide ultra-rapide** : Les essentiels en 2 minutes

---

## 🎯 Configuration Rapide

**Document Procreate** :

- Taille : **4096 x 4096 px** (qualité max)
- Format : **RGB, sRGB**
- Fond : **Transparent** ou **Blanc**

---

## 🗂️ Structure Calques (Par Texture)

**Nombre optimal** : **5 calques**

1. **Base** (100%) : Couleur principale
2. **Texture** (40%, Overlay) : Motif/texture
3. **Ombres** (25%, Multiply) : Ombres bords
4. **Lumières** (20%, Screen) : Lumières centre
5. **Détails** (15%, Normal) : Imperfections

**Total par texture** : 5 calques = Équilibre parfait

---

## 🖌️ Textures à Créer

1. **`mur.png`** (4096x4096px)
   - Base + texture + ombres/lumières

2. **`sol.png`** (4096x4096px)
   - Base + parquet/carrelage + jointures

3. **`plafond.png`** (4096x4096px, optionnel)
   - Base simple + texture subtile

---

## 📤 Export

1. **Actions** → **Partager** → **PNG**
2. Qualité : **Maximum**
3. Placer dans : `assets/textures/`

---

## 🔄 Intégration MuJoCo

```xml
<texture name="mur" type="2d" file="../../../assets/textures/mur.png"/>
<material name="mat_mur" texture="mur"/>
<geom type="box" material="mat_mur"/>

```

---

## ✅ Checklist Express

- [ ] 4096x4096px, RGB, sRGB
- [ ] 5 calques par texture (Base, Texture, Ombres, Lumières, Détails)
- [ ] Export PNG maximum
- [ ] Placé dans `assets/textures/`
- [ ] Testé dans MuJoCo

**Guide complet** : `GUIDE_PROCREATE_SCENE_COMPLET.md`

---

## 🎯 Navigation

**Retour à** : [README Documentation](../README.md)  
**Voir aussi** : [Guide Procreate Complet](GUIDE_PROCREATE_SCENE_COMPLET.md) • [Index Simulations](INDEX_GUIDES_PROCREATE.md) • [Index Thématique](../reference/INDEX_THEMATIQUE.md)

---

*Résumé Rapide - BBIA-SIM - 21 Novembre 2025*
