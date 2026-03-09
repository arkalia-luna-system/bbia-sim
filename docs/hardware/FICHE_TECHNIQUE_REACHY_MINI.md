# Fiche technique du matériel Reachy Mini

**Source** : Documentation officielle Reachy Mini (Hugging Face / Pollen Robotics), v1.5.0 (5 mars 2026).

---

## Description globale

### Dimensions

| Paramètre | Valeur |
|-----------|--------|
| Dimensions (déployé) | 30 × 20 × 15,5 cm |
| Masse | 1,475 kg |
| Matériaux | ABS, PC, aluminium, acier |

### Degrés de liberté

- **Tête** : 6 degrés de liberté (3 rotations et 3 translations)
- **Corps** : 1 rotation
- **Antennes** : 1 rotation × 2

### Tension d'alimentation

6,8 – 7,6 V

---

## Composants spécifiques

### Caractéristiques des moteurs

| Rôle | Référence | Quantité |
|------|-----------|----------|
| **Base (yaw_body)** | Dynamixel XC330-M288-PG personnalisé (XC330-M288-T avec engrenage en plastique) | 1 |
| **Antennes** | Dynamixel XL330-M077-T | 2 |
| **Plateforme Stewart (tête)** | Dynamixel XL330-M288-T | 6 |

### Carte de réseau de microphones

- 4 microphones numériques MEMS PDM
- Fréquence d'échantillonnage max : 16 kHz
- Sensibilité : -26 dB FS / Rapport signal/bruit : 64 dBA
- Basé sur le reSpeaker XMOS XVF3800 (Seeed Studio)

### Caméra

- Caméra Raspberry Pi v3 grand angle
- Sony IMX708, 12 MP
- Mise au point automatique
- Connexion DSI

### Son

- Haut-parleur 5 W à 4 ohms

### Carte électrique

- Tension d'entrée : 6,8 – 7,6 V
- Batterie LiFePO4 : 2000 mAh, 6,4 V, 12,8 Wh
- Protections : surcharge, décharge excessive, surintensité, court-circuit, capteur de température

### Électronique (Reachy Mini sans fil)

- **Carte contrôleur** : CM4
- Alimentation : 6,8 V – 7,6 V (carte d'alimentation)
- Connexion TTL des moteurs Dynamixel
- Connexion CSI caméra
- Connexion réseau de microphones
- Sortie USB-C (périphérique uniquement, pas de charge via ce port)
- **Module de calcul** : Raspberry Pi 4 – CM4104016 (Wi-Fi, 4 Go RAM, 16 Go stockage flash)
- Antenne Wi-Fi : patch double bande 2,4–5 GHz, 2,79 dBi, omnidirectionnelle

---

## Références documentation officielle

- Guide d'installation Reachy Mini (sans fil) : installation, Wi-Fi, mise à jour, SSH
- Diagnostic des moteurs : procédures de dépannage
- SSH : utilisateur `pollen`, mot de passe `root`
- Vérification système : `reachy-mini check` (ou équivalent selon la doc)

---

*Document créé à partir de la fiche technique Reachy Mini (Hugging Face / Pollen), janvier 2026.*
