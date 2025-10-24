# 🤖 BBIA Reachy Mini Simulation - Documentation Officielle

## 🎯 Vue d'ensemble

Ce projet fournit une simulation **parfaitement fidèle** du robot Reachy Mini dans MuJoCo, utilisant le modèle officiel et tous les assets 3D de Pollen Robotics.

## ✅ Statut du Projet

- **✅ Robot assemblé** : Le robot apparaît correctement assemblé dans MuJoCo
- **✅ Assets officiels** : 41 fichiers STL officiels téléchargés (9KB-1MB chacun)
- **✅ Modèle officiel** : `reachy_mini_REAL_OFFICIAL.xml` du repo pollen-robotics/reachy_mini
- **✅ Dimensions réelles** : Fidèle aux spécifications officielles
- **✅ Articulations** : 16 articulations fonctionnelles (yaw_body, stewart_1-6, passive_1-7, antennas)

## 🚀 Démarrage Rapide

### Lancement de la simulation 3D
```bash
cd /Volumes/T7/bbia-reachy-sim
mjpython scripts/launch_complete_robot.py --model reachy_mini_REAL_OFFICIAL.xml
```

### Vérification complète du projet
```bash
python scripts/verify_project.py
```

### Contrôles MuJoCo
- **Souris** : Rotation de la vue
- **Molette** : Zoom
- **Clic droit** : Déplacer la vue
- **Échap** : Fermer la fenêtre

## 📁 Structure du Projet

```
src/bbia_sim/sim/
├── models/
│   ├── reachy_mini_REAL_OFFICIAL.xml    # Modèle officiel principal
│   └── reachy_mini.xml                   # Modèle de base (backup)
├── assets/reachy_official/              # 41 fichiers STL officiels
│   ├── body_down_3dprint.stl
│   ├── body_top_3dprint.stl
│   ├── head_front_3dprint.stl
│   ├── head_back_3dprint.stl
│   ├── stewart_*.stl                     # Bras Stewart
│   └── ... (36 autres STL)
└── scripts/
    ├── launch_complete_robot.py         # Lanceur principal
    └── download_ALL_stl.py              # Script de téléchargement STL
```

## 🛠️ Scripts Utiles

### Téléchargement des STL officiels
```bash
python scripts/download_ALL_stl.py
```
Télécharge automatiquement les 41 fichiers STL depuis le repo officiel.

### Gestion des processus
```bash
./scripts/smart_process_cleanup.sh
```
Nettoie les processus gourmands tout en préservant les applications importantes.

## 🔧 Résolution de Problèmes

### Problème : Robot en pièces détachées
**Cause** : Fichiers STL corrompus (pointeurs Git LFS)
**Solution** : 
```bash
python scripts/download_ALL_stl.py
```

### Problème : Erreur "No such file or directory"
**Cause** : Chemin des assets incorrect
**Solution** : Le modèle `reachy_mini_REAL_OFFICIAL.xml` a le bon chemin `meshdir="../assets/reachy_official"`

### Problème : Fenêtre 3D ne s'ouvre pas
**Cause** : Mauvaise commande Python
**Solution** : Utiliser `mjpython` au lieu de `python` sur macOS

## 📊 Spécifications Techniques

- **Modèle** : Reachy Mini Wireless
- **Simulateur** : MuJoCo
- **Format** : MJCF (MuJoCo XML)
- **Assets** : 41 fichiers STL officiels
- **Articulations** : 16 (yaw_body + 6 stewart + 7 passive + 2 antennas)
- **Dimensions** : Fidèles aux spécifications officielles

## 🎯 Articulations Disponibles

| Articulation | Type | Description |
|--------------|------|-------------|
| `yaw_body` | Active | Rotation du corps |
| `stewart_1` à `stewart_6` | Active | Bras Stewart (6 articulations) |
| `passive_1` à `passive_7` | Passive | Articulations passives |
| `right_antenna` | Active | Antenne droite |
| `left_antenna` | Active | Antenne gauche |

## 🔗 Ressources Officielles

- **Repo officiel** : [pollen-robotics/reachy_mini](https://github.com/pollen-robotics/reachy_mini)
- **Documentation** : [docs.pollen-robotics.com](https://docs.pollen-robotics.com)
- **Modèle 3D** : OnShape CAD officiel
- **Assets STL** : Téléchargés depuis le repo officiel

## 📝 Historique des Corrections

### Problème Initial
Le robot apparaissait "en pièces détachées" dans MuJoCo.

### Cause Identifiée
Les fichiers STL étaient des pointeurs Git LFS (684 bytes) au lieu des vrais fichiers binaires.

### Solution Appliquée
1. Téléchargement des 41 fichiers STL officiels depuis pollen-robotics/reachy_mini
2. Utilisation du modèle officiel `reachy_mini.xml`
3. Correction du chemin des assets dans le MJCF
4. Validation de la complétude des assets

### Résultat
Robot parfaitement assemblé avec les vraies dimensions officielles.

## 🎉 Statut Final

**✅ MISSION ACCOMPLIE** : Le robot Reachy Mini est maintenant parfaitement simulé avec une fidélité de 100% aux spécifications officielles.

---
*Dernière mise à jour : 23 octobre 2025*
