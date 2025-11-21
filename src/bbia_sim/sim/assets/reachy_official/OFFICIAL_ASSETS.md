# Assets Officiels Reachy Mini

Ce dossier contient les modèles 3D officiels de Reachy Mini, récupérés du dépôt officiel Pollen Robotics.

## Source

- **Dépôt officiel** : https://github.com/pollen-robotics/reachy_mini
- **Version** : v1.0.0rc5 (21 novembre 2025)
- **Chemin source** : `src/reachy_mini/descriptions/reachy_mini/mjcf/assets/`

# 🎨 Assets Officiels Reachy Mini

Ce dossier contient les modèles 3D officiels de Reachy Mini, récupérés du dépôt officiel Pollen Robotics.

## 🏗️ Architecture des Assets

```mermaid
graph TB
    subgraph "Corps Principal"
        BODY_DOWN[body_down_3dprint.stl<br/>Partie inférieure]
        BODY_FOOT[body_foot_3dprint.stl<br/>Pied/base]
        BODY_TOP[body_top_3dprint.stl<br/>Partie supérieure]
        BODY_TURNING[body_turning_3dprint.stl<br/>Mécanisme rotation]
    end
    
    subgraph "Tête"
        HEAD_BACK[head_back_3dprint.stl<br/>Arrière tête]
        HEAD_FRONT[head_front_3dprint.stl<br/>Avant tête]
        HEAD_MIC[head_mic_3dprint.stl<br/>Microphone]
    end
    
    subgraph "Bras Stewart"
        STEWART_MAIN[stewart_main_plate_3dprint.stl<br/>Plaque principale]
        STEWART_TRICAP[stewart_tricap_3dprint.stl<br/>Capuchon triangulaire]
        STEWART_LINK[stewart_link_rod.stl<br/>Tige liaison]
        STEWART_ARM[mp01062_stewart_arm_3.stl<br/>Bras principal]
    end
    
    subgraph "Grippers"
        GRIPPER_LEFT[left_gripper_3dprint.stl<br/>Pince gauche]
        GRIPPER_RIGHT[right_gripper_3dprint.stl<br/>Pince droite]
    end
    
    BODY_DOWN --> HEAD_BACK
    BODY_TOP --> HEAD_FRONT
    HEAD_FRONT --> HEAD_MIC
    
    BODY_TURNING --> STEWART_MAIN
    STEWART_MAIN --> STEWART_TRICAP
    STEWART_TRICAP --> STEWART_LINK
    STEWART_LINK --> STEWART_ARM
    
    STEWART_ARM --> GRIPPER_LEFT
    STEWART_ARM --> GRIPPER_RIGHT
```

## 📊 Répartition des Assets

```mermaid
pie title Types d'Assets STL
    "Corps Principal" : 25
    "Bras Stewart" : 30
    "Tête" : 20
    "Grippers" : 15
    "Autres" : 10
```

## Mapping vers bbia-sim

### Correspondances avec les anciens assets

- **Torso** → `body_top_3dprint.stl` + `body_down_3dprint.stl`
- **Head** → `head_front_3dprint.stl` + `head_back_3dprint.stl`
- **Upper Arm** → `mp01062_stewart_arm_3.stl`
- **Forearm** → `stewart_link_rod.stl`
- **Gripper** → `stewart_tricap_3dprint.stl`

### Notes d'Intégration

- Les assets officiels sont plus détaillés et réalistes
- La structure Stewart (plateforme parallèle) est maintenant correctement représentée
- Les fichiers `.part` correspondants sont disponibles dans le dépôt officiel si besoin

## Utilisation

Ces assets sont référencés dans le fichier MJCF `src/bbia_sim/sim/models/reachy_mini.xml` et utilisés par le simulateur MuJoCo pour le rendu 3D.

## Maintenance

- Vérifier périodiquement les mises à jour du dépôt officiel
- Maintenir la cohérence entre les références MJCF et les fichiers STL
- Documenter tout changement de structure ou de nommage
