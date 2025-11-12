# 📁 Assets BBIA

> **Ressources et références pour BBIA - Brain-Based Interactive Agent**

## 🎯 **Contenu du Dossier Assets**

### 📹 **Vidéos de Démonstration**

- **3 nouvelles vidéos MuJoCo** (Oct / No2025025025025025) : Captures d'écran de la simulation 3D
  - `Enregistrement de l'écran Oct / No2025025025025025 à 17.00.52.mov` (2.7 MB)
  - `Enregistrement de l'écran Oct / No2025025025025025 à 17.01.27.mov` (3.4 MB)
  - `Enregistrement de l'écran Oct / No2025025025025025 à 17.01.56.mov` (3.1 MB)
- **Source** : Toutes enregistrées depuis `examples/demo_mujoco_continue.py` (robot en mouvement continu)
- **Inventaire complet** : Voir [`MEDIAS_INVENTAIRE.md`](./MEDIAS_INVENTAIRE.md)

### 🖼️ **Images**

- **Référence visuelle déplacée** : Voir `docs/reachy/REACHY_MINI_REFERENCE.md` - Référence visuelle du robot Reachy Mini
- **10 nouvelles captures d'écran** (Oct / No2025025025025025) : Séquence de captures montrant le robot en mouvement
- **5 captures d'écran** (Oct / No2025025025025025) : Captures de référence antérieures
- **Images finales** : `robot_3d_final.png`, `robot_animation.gif`
- **Texture robot Procreate** : `textures/robot_reachy_mini.png` (créé avec Procreate, 944x712px)
- **Inventaire complet** : Voir [`MEDIAS_INVENTAIRE.md`](./MEDIAS_INVENTAIRE.md)

---

## 🏗️ Architecture des Assets BBIA

```mermaid
graph TB
    subgraph "Assets Visuels"
        IMAGES[Images<br/>Références visuelles]
        REACHY_REF[REACHY_MINI_REFERENCE.md<br/>Référence robot]
    end
    
    subgraph "Robot Reachy Mini"
        DESIGN[Design<br/>Humanoïde simplifié]
        COULEUR[Couleur<br/>Blanc]
        YEUX[Yeux<br/>Cercles noirs expressifs]
        ANTENNES[Antennes<br/>Expressivité]
    end
    
    subgraph "Contexte BBIA"
        LIVRAISON[Livraison<br/>Fin 2025 / Début 2025]
        PRIX[Prix<br/>449$ (~500€)]
        FABRICANT[Fabricant<br/>Pollen Robotics]
    end
    
    IMAGES --> REACHY_REF
    REACHY_REF --> DESIGN
    
    DESIGN --> COULEUR
    COULEUR --> YEUX
    YEUX --> ANTENNES
    
    ANTENNES --> LIVRAISON
    LIVRAISON --> PRIX
    PRIX --> FABRICANT
```

## 📊 Répartition des Ressources

```mermaid
pie title Types de Ressources Assets
    "Images de référence" : 40
    "Documentation robot" : 30
    "Spécifications techniques" : 20
    "Liens externes" : 10
```

- **Statut** : Open source

---

## 🎮 **Utilisation pour le Développement**

### 🧠 **Simulation BBIA**

- **Référence visuelle** : Pour les simulations Unity
- **Design UI** : Interface utilisateur cohérente
- **Animations** : Expressions et mouvements fidèles
- **Testing** : Validation des fonctionnalités

### 🎯 **Fonctionnalités BBIA**

- **6 émotions** : Basées sur les "yeux" et antennes
- **4 microphones** : Reconnaissance vocale
- **Caméra grand angle** : Vision par ordinateur
- **Mouvements tête** : 6 DOF + animation antennes

### 📚 **Documentation**

- **Guides** : Support visuel
- **Tutoriels** : Exemples concrets
- **Présentation** : Communication projet
- **Marketing** : Support promotionnel

---

## 🎯 **Informations Techniques**

### 🔧 **Spécifications Hardware**

- **Processeur** : Raspberry Pi 5 intégré
- **Connectivité** : Wi-Fi intégré
- **Audio** : 4 microphones + haut-parleur 5W
- **Vision** : Caméra grand angle
- **Mouvements** : 6 DOF tête + rotation corps + 2 antennes
- **Batterie** : Intégrée + USB-C
- **Poids** : 1,5 kg
- **Dimensions** : 28cm (actif) / 23cm (veille) x 16cm

### 🎮 **Simulation Unity**

- **Modèle 3D** : Représentation fidèle
- **Expressions** : Animations faciales
- **Mouvements** : Fluides et naturels
- **Environnement** : 3D interactif

---

## 🌟 **Actualité Récente**

### 📰 **Couverture Média**

- **TechCrunch** : Article principal
- **YouTube** : Vidéos de présentation
- **Medium** : Articles techniques
- **Blogs personnels** : Retours d'expérience

### 🔗 **Partners**

- **Hugging Face** : Implication majeure
- **Pollen Robotics** : Fabricant
- **Open Source** : Communauté active

---

## 💡 **Observations Clés**

### ✅ **Points Positifs**

- **Design épuré** : Facile à reproduire
- **Expressivité** : "Yeux" expressifs + mouvements tête/corps (yaw_body + stewart joints)
- **Taille compacte** : Parfait pour bureau/maison
- **Open source** : Développement communautaire

### 🎯 **Pour BBIA**

- **Robot parfait** : Pour l'IA émotionnelle
- **Expressivité** : Idéal pour les émotions
- **Interactions** : Design adapté aux interactions
- **Futur** : Technologie prometteuse

---

## 🎯 **Navigation**

### 📁 **Structure**

```text
assets/
├── 📖 README.md                      # Ce fichier
├── 📹 MEDIAS_INVENTAIRE.md           # Inventaire complet vidéos/images
├── 🎬 videos/                        # Vidéos de démonstration
│   ├── Enregistrement de l'écran Oct / No2025025025025025 à 17.00.52.mov
│   ├── Enregistrement de l'écran Oct / No2025025025025025 à 17.01.27.mov
│   ├── Enregistrement de l'écran Oct / No2025025025025025 à 17.01.56.mov
│   └── (traces JSONL de démos)
├── 🖼️ images/                        # Captures d'écran et images
│   ├── Capture d'écran Oct / No2025025025025025 à 16.48.XX.png (série)
│   ├── Capture d'écran Oct / No2025025025025025 à 16.49.XX.png (série)
│   ├── robot_3d_final.png
│   └── robot_animation.gif
└── 🎨 textures/                      # Textures Procreate pour MuJoCo
```

### 🔍 **Accès Rapide**

- **Référence visuelle** : `docs/reachy/REACHY_MINI_REFERENCE.md`
- **Description complète** : Voir le fichier de référence

---

**BBIA** - Brain-Based Interactive Agent  
*Assets et références* 📁✨

**Robot** : Reachy Mini Wireless  
**Fabricant** : Pollen Robotics  
**Partenaire** : Hugging Face  
**Statut** : Open Source  
**Livraison** : Fin 2025 / Début 2025
